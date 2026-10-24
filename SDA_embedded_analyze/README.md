# SDA embedded-penalty search analysis

This directory replays the SDA independently from Simulink. It is deliberately
separate from the production controller.

## Files

- `extract_embedded_H_cases.m`: extracts every run and every transaction from
  `../embedded_H.mldatx` into `data/embedded_H_sda_cases_2025b.mat`.
- `SDA_Solve_Discrete_Profiled.m`: wrapper for the explicit branch-penalty
  formulation.
- `SDA_Solve_Embedded_Profiled.m`: wrapper for the embedded-H formulation.
- `SDA_Solve_Profiled_Core.m`: exact MATLAB search loop plus node-level trace.
- `analyze_search_intermediates.m`: validates replay outputs and calculates
  depth-wise visit, pruning, margin, and penalty statistics.
- `plot_depth_statistics.m`: plots mean visited nodes and pruning rate versus
  SDA depth for the three paired torque conditions.
- `generate_diagnostic_figures.m`: produces full-data distributions and
  reproducibly selected Q5/Q50/Q95 representative search diagnostics.
- `enumerate_same_prefix_analysis.m`: enumerates every binary prefix for the
  selected cases and compares effective offsets, prefix slack, and initial-sphere
  membership on exactly matched prefixes.
- `SDA_Solve_Hybrid_Profiled.m`: uses the embedded Babai sequence to initialize
  the exact explicit-penalty search.
- `SDA_Solve_Oracle_Profiled.m`: offline-only optimum-incumbent diagnostic.
- `evaluate_hybrid_ablation.m`: validates and compares native explicit, native
  embedded, hybrid, and two oracle configurations.
- `plot_hybrid_ablation_results.m`: regenerates the ablation figures from the
  saved results without rerunning the search.
- `run_analysis.m`: example entry point.

## Recommended workflow

```matlab
cd SDA_embedded_analyze
extract_embedded_H_cases
report = run_analysis;
```

The extractor only reads the SDI archive; it does not simulate the plant. The
profiling solver records a complete node trace only for selected transactions.
For all other transactions the analysis should retain summaries only, because a
one-second 10 kHz run can otherwise create a very large MAT file.

The run name selects the formulation. Runs beginning with `embedded_H_` use the
embedded wrapper and therefore set the explicit zero-vector branch penalty to
zero. Runs beginning with `original_` use the logged explicit branch penalty.
This is intentional and independent of the historical `SDA_input.signal8`
logging-label issue.
