# SDA files

## Current production path

The active controller is the MATLAB Function block at:

`FCSMPC_double_vector_2025b/FCS_MPC/SDA_Solve/SDA Solve/ SDA_Solve_Synchronized`

`PSPL_TestFrame/SDA_PL_Framework` is retained for historical validation and
is not the production control path.

## Runtime

`runtime/defineSdaBuses.m` is called by `double_vector_ini_2025b.m` after
`FOC_params.sldd` is rebuilt. Files in this directory are normal startup
dependencies.

## Tools

`tools/` contains optional utilities for naming and rebuilding the current
reference-result routing. These tools modify and save the Simulink model and
must not be run as part of normal initialization.

## Legacy PS/PL

`legacy_pspl/` contains the discontinued multi-cycle PS/PL implementation:

- `build/` contains construction scripts. `pre_2025b/` targets the older
  `FCSMPC_double_vector` model.
- `migration/` contains one-time model upgrade and repair scripts.
- `validation/` contains comparison, diagnosis, and transaction-aligned
  reference utilities.

Legacy scripts are kept for traceability. They are not called by normal
model initialization and may modify `.slx` files when run.
