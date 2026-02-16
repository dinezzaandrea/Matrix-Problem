import os
import time
import collections
import sys
import threading
import shutil
import destination_algo as dest
import pivot_algo as piv
from concurrent.futures import ProcessPoolExecutor, as_completed

def load_map(map_path):
    """Parses the .map file to extract obstacles, dimensions, and the pivot point."""
    obstacles = set()
    pivot = None
    width, height = 0, 0
    try:
        with open(map_path, 'r') as f:
            lines = f.readlines()
        map_started = False
        row = 0
        for i, line in enumerate(lines):
            if line.startswith("height"): height = int(line.split()[1])
            if line.startswith("width"): width = int(line.split()[1])
            if line.startswith("map"):
                map_started = True
                continue
            if map_started and row < height:
                for col, char in enumerate(line.strip()):
                    if char in ['@', 'T']:
                        obstacles.add((col, row))
                row += 1
            if line.startswith("pivot"):
                coords = lines[i+1].split()
                pivot = (int(coords[0]), int(coords[1]))
                break
    except Exception as e:
        print(f"Error loading map {map_path}: {e}")
    return obstacles, width, height, pivot

def load_scenario(scen_path):
    """Parses the processed .txt scenario file to get map path, starts, and goals."""
    with open(scen_path, 'r') as f:
        lines = [line.strip() for line in f.readlines()]

    map_rel_path = lines[1]
    starts_idx = lines.index("agent & start")
    dest_idx = lines.index("destination")

    # Get Initial Configuration
    initial_config = {}
    for i in range(starts_idx + 1, dest_idx):
        parts = lines[i].split()
        initial_config[int(parts[0])] = (int(parts[1]), int(parts[2]))

    # Get Destination Set
    destinations = set()
    for i in range(dest_idx + 1, len(lines)):
        parts = lines[i].split()
        destinations.add((int(parts[0]), int(parts[1])))

    return map_rel_path, initial_config, destinations

def check_pivot_reachability_without_bridges(width, height, obstacles, pivot, initial_config):
    """Verifica se tutti gli agenti sono nella stessa bridgeless component del pivot."""
    if not pivot or pivot in obstacles: return False
    adj = collections.defaultdict(list)
    nodes = set()
    for r in range(height):
        for c in range(width):
            if (c, r) not in obstacles:
                nodes.add((c, r))
                for dx, dy in [(0,1), (1,0)]:
                    nc, nr = c+dx, r+dy
                    if 0 <= nc < width and 0 <= nr < height and (nc, nr) not in obstacles:
                        adj[(c,r)].append((nc,nr))
                        adj[(nc,nr)].append((c,r))
    tin, low = {n: -1 for n in nodes}, {n: -1 for n in nodes}
    timer, bridges = 0, set()
    stack = [(pivot, None, iter(adj[pivot]))]
    tin[pivot] = low[pivot] = timer
    timer += 1
    while stack:
        u, p, children = stack[-1]
        try:
            v = next(children)
            if v == p: continue
            if tin[v] != -1: low[u] = min(low[u], tin[v])
            else:
                tin[v] = low[v] = timer
                timer += 1
                stack.append((v, u, iter(adj[v])))
        except StopIteration:
            stack.pop()
            if stack:
                parent, _, _ = stack[-1]
                low[parent] = min(low[parent], low[u])
                if low[u] > tin[parent]: bridges.add(tuple(sorted((parent, u))))
    visited, pivot_component = {pivot}, set()
    q = collections.deque([pivot])
    while q:
        u = q.popleft()
        pivot_component.add(u)
        for v in adj[u]:
            if v not in visited and tuple(sorted((u, v))) not in bridges:
                visited.add(v)
                q.append(v)
    for agent_pos in initial_config.values():
        if agent_pos not in pivot_component: return False
    return True

def process_single_file(args):
    folder_path, scen_file, res_folder_path = args
    scen_path = os.path.join(folder_path, scen_file)
    output_res_path = os.path.join(res_folder_path, f"res_{scen_file}")
    time_log_path = os.path.join(res_folder_path, "execution_times.csv")

    try:
        map_rel_path, init_config, dest_set = load_scenario(scen_path)
        actual_map_path = os.path.abspath(os.path.join(folder_path, "..", map_rel_path))
        obstacles, w, h, pivot = load_map(actual_map_path)

        is_safe = check_pivot_reachability_without_bridges(w, h, obstacles, pivot, init_config)
        exec_time = 0.0

        with open(output_res_path, 'w') as out_res:
            out_res.write(f"Safe-to-Pivot: {is_safe}\n")

            if is_safe:
                start_t = time.perf_counter()

                fmt_pos = lambda d: str(list(d.values())).replace('.', ',')
                out_res.write(f"0 {fmt_pos(init_config)}\n")
                agent_ids = list(init_config.keys())

                c_pivot = piv.pivot_visit_algorithm(agent_ids, init_config, pivot, obstacles, w, h)
                out_res.write(f"1 {fmt_pos(c_pivot)}\n")

                final_c = dest.extend_to_destination_set(agent_ids, c_pivot, dest_set, obstacles, w, h)
                out_res.write(f"2 {fmt_pos(final_c)}\n")

                exec_time = time.perf_counter() - start_t
            else:
                print(f"[NOT SAFE] Il file {scen_path} non può raggiungere il pivot senza attraversare bridge.")

        time_str = str(exec_time).replace('.', ',')
        with open(time_log_path, 'a') as tf:
            tf.write(f"{scen_file};{time_str}\n")

    except Exception as e:
        print(f"Error in {scen_file}: {e}")
    return True

def run_experiment_parallel():
    scenarios_root = "scenarios"
    results_root = "results"
    tasks = []

    if not os.path.exists(scenarios_root): return

    for map_folder in os.listdir(scenarios_root):
        folder_path = os.path.join(scenarios_root, map_folder)
        if not os.path.isdir(folder_path): continue

        res_folder_path = os.path.join(results_root, map_folder)
        os.makedirs(res_folder_path, exist_ok=True)

        time_log_path = os.path.join(res_folder_path, "execution_times.csv")

        should_reset = False
        if os.path.exists(time_log_path):
            with open(time_log_path, 'r') as f:
                header = f.readline()
                if ";" not in header:
                    should_reset = True

        if not should_reset and os.path.exists(res_folder_path):
            res_files = [f for f in os.listdir(res_folder_path) if f.startswith("res_")]
            if res_files:
                sample_file = os.path.join(res_folder_path, res_files[0])
                try:
                    with open(sample_file, 'r') as f:
                        content = f.read()
                        if "2-edge-connected" in content:
                            should_reset = True
                except Exception:
                    pass

        if should_reset:
            print(f"Reset cartella: {map_folder} (rilevati dati obsoleti 2-edge o CSV errato)")
            shutil.rmtree(res_folder_path)
            os.makedirs(res_folder_path, exist_ok=True)

        done_scenarios = set()
        if os.path.exists(time_log_path):
            with open(time_log_path, 'r') as f:
                done_scenarios = {line.split(';')[0] for line in f if ';' in line}
        else:
            with open(time_log_path, 'w') as f:
                f.write("scenario_file;execution_time_seconds\n")

        for scen_file in os.listdir(folder_path):
            if scen_file.endswith(".txt") and not scen_file.startswith("res_"):
                output_res_path = os.path.join(res_folder_path, f"res_{scen_file}")

                if scen_file in done_scenarios and os.path.exists(output_res_path):
                    continue

                tasks.append((folder_path, scen_file, res_folder_path))

    if not tasks:
        print("Tutti gli scenari sono già stati completati correttamente.")
        return

    with ProcessPoolExecutor() as executor:
        futures = [executor.submit(process_single_file, t) for t in tasks]
        completed = 0
        for _ in as_completed(futures):
            completed += 1
            if completed % 100 == 0 or completed == len(tasks):
                print(f"Avanzamento: {completed}/{len(tasks)}...")

    print("Fine. Tutti i tempi sono stati salvati 'live' nei CSV di cartella.")

if __name__ == "__main__":
    try:
        run_experiment_parallel()
    except KeyboardInterrupt:
        print("\nEsecuzione interrotta dall'utente.")
        sys.exit(0)
    except Exception as e:
        print(f"Errore critico durante l'esecuzione: {e}")
        sys.exit(1)
