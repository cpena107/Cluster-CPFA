
def generate_latex():
    import os
    seconds_variants = [5, 10, 15, 20]
    site_groups = [
        [25, 50, 75, 100],
        [125, 150, 175, 200],
        [225, 250, 275, 300],
    ]

    header = r"""\documentclass{article}
\usepackage{graphicx}
\usepackage{svg}
\usepackage{subcaption}
\usepackage{geometry}
\usepackage[section]{placeins}
\geometry{a4paper, margin=0.5in}

\begin{document}
"""

    body = ""
    for sec in seconds_variants:
        for group in site_groups:
            # Build rows only for sites where assets exist; skip figure if none
            rows = []
            for s in group:
                base_cluster = f"analysis_plots/{s}_sites_{sec}_seconds_cluster.svg"
                base_powerlaw = f"analysis_plots/{s}_sites_{sec}_seconds_powerlaw.svg"
                base_random = f"analysis_plots/{s}_sites_{sec}_seconds_random.svg"
                if not (os.path.exists(base_cluster) and os.path.exists(base_powerlaw) and os.path.exists(base_random)):
                    continue
                row = f"""
    % Row for {s} sites ({sec} seconds)
    \\begin{{minipage}}[c]{{0.05\\textwidth}}
        \\centering \\textbf{{{s} Sites}}
    \\end{{minipage}}
    \\hfill
    \\begin{{minipage}}[c]{{0.3\\textwidth}}
        \\includesvg[width=\\linewidth]{{analysis_plots/{s}_sites_{sec}_seconds_cluster}}
    \\end{{minipage}}
    \\hfill
    \\begin{{minipage}}[c]{{0.3\\textwidth}}
        \\includesvg[width=\\linewidth]{{analysis_plots/{s}_sites_{sec}_seconds_powerlaw}}
    \\end{{minipage}}
    \\hfill
    \\begin{{minipage}}[c]{{0.3\\textwidth}}
        \\includesvg[width=\\linewidth]{{analysis_plots/{s}_sites_{sec}_seconds_random}}
    \\end{{minipage}}
    \\vspace{{0.3cm}}
"""
                rows.append(row)

            if not rows:
                continue

            body += r"""
\begin{figure}[p]
    \centering
    % Column headers
    \begin{minipage}{0.05\textwidth} \end{minipage}
    \begin{minipage}{0.3\textwidth} \centering \textbf{Clustered} \end{minipage}
    \begin{minipage}{0.3\textwidth} \centering \textbf{Powerlaw} \end{minipage}
    \begin{minipage}{0.3\textwidth} \centering \textbf{Random} \end{minipage}
    \vspace{0.2cm}
"""
            for r in rows:
                body += r
            body += f"""
    \\caption{{Resource collection analysis — {sec} seconds. Columns: Clustered, Powerlaw, Random; rows: {"/".join(map(str, group))} sites.}}
\\end{{figure}}
\\clearpage
"""

    footer = r"""
\end{document}
"""

    return header + body + footer

if __name__ == "__main__":
    with open("analysis_plots_figure.tex", "w") as f:
        f.write(generate_latex())
    print("Generated analysis_plots_figure.tex")
