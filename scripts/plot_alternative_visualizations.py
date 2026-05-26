import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import os

def plot_alternatives():
    input_file = "all_experiment_results_summary.csv"
    output_dir = "plots_attribute_effects"
    os.makedirs(output_dir, exist_ok=True)
    
    if not os.path.exists(input_file):
        print(f"Error: {input_file} not found.")
        return

    print(f"Reading {input_file}...")
    df = pd.read_csv(input_file)
    
    # Set plot style
    sns.set_theme(style="whitegrid")
    
    config_vars = ['VisitedTolerance', 'RecordingFreq', 'MaxVisited', 'MaxRadius']
    
    # ---------------------------------------------------------
    # 1. Point Plots: Effect of Variable on Time
    # ---------------------------------------------------------
    print("Generating Point Plots (Variable on X-axis)...")
    
    for var in config_vars:
        print(f"Plotting spread for {var}...")
        try:
            # We use catplot (point) to show the trend of the mean with confidence intervals
            g = sns.catplot(
                data=df,
                x=var, 
                y="Mean", 
                hue="ResourceCount",
                col="ArenaSize",
                row="Distribution",
                kind="point",
                height=3, 
                aspect=1.5,
                palette="viridis",
                errorbar="sd", # Show standard deviation as error bars
                capsize=0.2,
                sharey=False 
            )
            
            g.fig.subplots_adjust(top=0.92)
            g.fig.suptitle(f'Effect of {var} on Mean Time', fontsize=16)
            
            # Rotate x labels if there are many
            if df[var].nunique() > 5:
                for axes in g.axes.flat:
                    _ = axes.set_xticklabels(axes.get_xticklabels(), rotation=45)

            filename = f"Effect_of_{var}_on_Time.png"
            filepath = os.path.join(output_dir, filename)
            g.savefig(filepath)
            print(f"Saved {filepath}")
            plt.close(g.fig)
            
        except Exception as e:
            print(f"Error for {var}: {e}")

    # ---------------------------------------------------------
    # 2. Correlation Heatmap
    # ---------------------------------------------------------
    print("Generating Correlation Heatmap...")
    try:
        # Select numerical columns of interest
        cols_of_interest = ['Mean'] + config_vars + ['ResourceCount', 'NumTests']
        corr_matrix = df[cols_of_interest].corr()
        
        plt.figure(figsize=(10, 8))
        sns.heatmap(corr_matrix, annot=True, cmap='coolwarm', fmt=".2f", linewidths=0.5)
        plt.title('Correlation Matrix: Variables vs Mean Time')
        plt.tight_layout()
        
        filepath = os.path.join(output_dir, "Correlation_Heatmap.png")
        plt.savefig(filepath)
        print(f"Saved {filepath}")
        plt.close()
        
    except Exception as e:
        print(f"Error generating heatmap: {e}")

if __name__ == "__main__":
    plot_alternatives()
