% FCS-MPC parameters for FCSMPC_double_vector.slx
FCS.Ts_BuildModel = 1/10e3;
FCS.Ts_SDA = 1/40e3;
FCS.lambda_fsw = 1e-3;
FCS.NmaxNode = 10000;
FCS.Np = 5;

% Variant selection
% 1: legacy controller variant
% 2: double_vector_matlab_FSM_Calc
MPC_VARIANT = 2;

% The current SDA PL Bus dimensions are fixed for Np = 5
% (3*Np = 15 binary decision levels).
assert(FCS.Np == 5, 'FCS_Parameter:UnsupportedHorizon', ...
    'The current double-vector PL framework requires FCS.Np == 5.');
