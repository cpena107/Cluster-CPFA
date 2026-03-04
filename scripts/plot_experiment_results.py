import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import os

def plot_results():
    input_file = "all_experiment_results_summary.csv"
    output_dir = "plots_by_variable"
    os.makedirs(output_dir, exist_ok=True)
    
    if not os.path.exists(input_file):
        print(f"Error: {input_file} not found.")
        return

    print(f"Reading {input_file}...")
    df = pd.read_csv(input_file)
    
    # Identify configuration variables
    config_vars = ['VisitedTolerance', 'RecordingFreq', 'MaxVisited', 'MaxRadius']
    
    # Set plot style
    sns.set_theme(style="whitegrid")
    
    # Get unique ArenaSizes and Distributions for sorting if possible
    if 'ArenaSize' in df.columns:
        # Sort arena size naturally? 8x8, 10x10...
        pass 
        
    for var in config_vars:
        if var not in df.columns:
            print(f"Warning: {var} not found in dataframe.")
            continue
            
        print(f"Generating plot for {var}...")
        
        try:
            # We use relplot to create a grid of plots
            # x-axis: ResourceCount
            # y-axis: Mean (Time)
            # hue: The configuration variable we are analyzing
            # col: ArenaSize
            # row: Distribution
            
            # Using discrete color palette if few unique values, else continuous
            unique_vals = df[var].nunique()
            palette = "tab10" if unique_vals <= 10 else "viridis"
            
            g = sns.relplot(
                data=df,
                x="ResourceCount", 
                y="Mean",
                hue=var,
                kind="line", 
                marker="o",
                col="ArenaSize", 
                row="Distribution",
                height=3.5, 
                aspect=1.3,
                palette=palette,
                facet_kws={'sharey': False, 'sharex': True}
            )
            
            # Set ticks as requested
            g.set(xticks=[16, 32, 48, 64, 80])
            
            g.fig.subplots_adjust(top=0.92)
            g.fig.suptitle(f'Mean Time vs Resource Count (Hue: {var})', fontsize=16)
            
            # Save
            filename = f"MeanTime_vs_ResourceCount_by_{var}.png"
            filepath = os.path.join(output_dir, filename)
            g.savefig(filepath)
            print(f"Saved {filepath}")
            
            # Close figure to free memory
            plt.close(g.fig)
            
        except Exception as e:
            print(f"Error plotting {var}: {e}")

if __name__ == "__main__":
    plot_results()
