import os
import pandas as pd
import matplotlib.pyplot as plt

def generate_plots_and_stats():
    results_root = "results"
    data_list = []

    if not os.path.exists(results_root):
        print(f"Errore: la cartella '{results_root}' non esiste.")
        return

    print(f"{'Cartella/Mappa':<35} | {'Stato':<6} | {'Min (s)':<10} | {'Max (s)':<10} | {'Media (s)'}")
    print("-" * 85)

    # 1. Raccolta dati e statistiche per file
    for folder_name in sorted(os.listdir(results_root)):
        folder_path = os.path.join(results_root, folder_name)

        if os.path.isdir(folder_path):
            csv_path = os.path.join(folder_path, "execution_times.csv")

            if os.path.exists(csv_path):
                try:
                    # Leggiamo il CSV usando il punto e virgola come separatore
                    # Gestiamo il fatto che i numeri nel CSV hanno la virgola decimale
                    df = pd.read_csv(csv_path, sep=';')

                    df['execution_time_seconds'] = pd.to_numeric(
                        df['execution_time_seconds'].astype(str).str.replace(',', '.'),
                        errors='coerce'
                    )

                    df = df.dropna(subset=['execution_time_seconds'])

                    df = df[df['execution_time_seconds'] > 0]

                    if not df.empty:
                        # Calcolo statistiche per questa cartella
                        min_t = df['execution_time_seconds'].min()
                        max_t = df['execution_time_seconds'].max()
                        mean_t = df['execution_time_seconds'].mean()

                        # Formattazione per la stampa (sostituiamo . con ,)
                        fmt = lambda x: str(round(x, 5)).replace('.', ',')
                        print(f"{folder_name:<35} | {'OK':<6} | {fmt(min_t):<10} | {fmt(max_t):<10} | {fmt(mean_t)}")

                        # Estrazione numero agenti dal nome cartella per l'aggregazione finale
                        parts = folder_name.split('-')
                        if len(parts) >= 2:
                            num_agents = int(parts[1])
                            data_list.append({
                                'agents': num_agents,
                                'mean_execution_time': mean_t
                            })
                    else:
                        print(f"{folder_name:<35} | VUOTO  | -          | -          | -")

                except Exception as e:
                    print(f"Errore nel processare {folder_name}: {e}")
            else:
                print(f"{folder_name:<35} | MANCA | -          | -          | -")

    if not data_list:
        print("\nNessun dato trovato per generare i grafici.")
        return

    # Creiamo un DataFrame con i risultati aggregati
    results_df = pd.DataFrame(data_list)

    # 2. Media delle medie sul numero di agenti
    final_stats = results_df.groupby('agents')['mean_execution_time'].mean().reset_index()
    final_stats = final_stats.sort_values('agents')

    # 3. Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(final_stats['agents'], final_stats['mean_execution_time'], marker='o', linestyle='-', color='b')

    plt.title('Tempo di esecuzione medio rispetto al numero di agenti')
    plt.xlabel('Numero di Agenti')
    plt.ylabel('Tempo Medio (secondi)')
    plt.grid(True, linestyle='--', alpha=0.7)

    plt.xticks(final_stats['agents'])

    plt.savefig('grafico_tempi_agenti.png')
    print("\n" + "="*40)
    print("Grafico generato: grafico_tempi_agenti.png")

    # Output dei dati aggregati (Media delle Medie)
    print("\nRIASSUNTO FINALE (Media delle Medie per numero agenti):")
    print(f"{'Agenti':<10} | {'Tempo Medio (s)':<15}")
    print("-" * 30)
    for _, row in final_stats.iterrows():
        agenti = int(row['agents'])
        # Formattazione finale con virgola
        tempo = str(round(row['mean_execution_time'], 6)).replace('.', ',')
        print(f"{agenti:<10} | {tempo:<15}")

if __name__ == "__main__":
    generate_plots_and_stats()
