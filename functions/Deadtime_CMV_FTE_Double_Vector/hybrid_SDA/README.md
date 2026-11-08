# Hybrid SDA Simulink test functions

This folder leaves the production controller unchanged and provides three
drop-in MATLAB Function implementations for testing the hybrid strategy.

## Data flow

1. `QP_Precompute_10k_Simplified_Hybrid` outputs both `H_exp` and `H_emb`.
2. `QP_Runtime_40k_Simplified_Hybrid` outputs the matched pair
   `U_unc_exp` and `U_unc_emb`.
3. `SDA_Solve_Synchronized_Hybrid` computes Babai with the embedded pair,
   evaluates that binary sequence with the explicit objective, and performs
   the complete SDA search with the explicit pair.

The SDA input `lambdaZero` must retain the commanded explicit zero-vector
penalty. Do not set it to zero in the hybrid search: the embedded penalty is
used only to obtain the Babai candidate, while explicit zero-vector penalties
are still required at every completed three-bit branch.

## Diagnostic outputs

- `JbabaiEmbedded`: embedded quadratic distance of the Babai candidate.
- `JbabaiExplicit`: explicit quadratic distance of the same candidate before
  commanded zero-vector and dead-time penalties are added.
- `initialRadius`: complete explicit initial incumbent, including the
  commanded zero-vector and dead-time penalties.

For the first Simulink validation, compare the selected sequence, `Jopt`,
`nodes`, `JbabaiEmbedded`, `JbabaiExplicit`, and `initialRadius` against the
offline hybrid analysis. The implementation deliberately recomputes the
explicit Babai-candidate cost instead of assuming a fixed offset.

All three files are self-contained with respect to the alternative explicit
and embedded controller functions. The hybrid precomputation builds both
factors after one common model calculation, and the hybrid runtime performs
one current reconstruction before solving the two matched linear systems.
The explicit and embedded Simulink alternatives can therefore remain
commented out while the hybrid subsystem is active.

## Experimental current-constrained solver

`SDA_Solve_Synchronized_Hybrid_CurrentConstrained.m` is an independent
experimental extension; it does not replace the production hybrid solver. It
adds a phase-current peak limit and enforces it at the dead-time and vector
endpoints of the first two switching blocks that are applied before the next
controller transaction.

The ordinary embedded Babai sequence remains the fast path. If its applied
two-vector prefix is infeasible, the solver:

1. propagates the physical current for the at-most 64 two-vector prefixes;
2. rejects current-infeasible prefixes;
3. selects the feasible prefix with the smallest first-six-row embedded
   partial metric;
4. computes one conditional embedded Babai suffix for that prefix;
5. reevaluates the complete sequence with the explicit objective; and
6. uses the result as the finite incumbent for the explicit constrained SDA.

The prefix choice is initialization only. Other feasible prefixes remain in
the exact main search. The main search prunes with explicit partial metrics and
with branch-current infeasibility, not with the embedded prefix ranking.

Additional outputs expose `feasibleSolutionFound`, `babaiPrefixFeasible`,
`prefixRepairUsed`, `feasiblePrefixCount`, and `currentPruneCount`. If no
feasible complete sequence is found before termination, `Jopt` and
`initialRadius` remain infinite and `feasibleSolutionFound` is false. The
caller must supply a separately validated safety fallback before deployment.
