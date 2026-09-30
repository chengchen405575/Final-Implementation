%% DOUBLE_VECTOR_INI  Startup for FCSMPC_double_vector.slx
% This script contains the FOC_Ini initialization used by ver_2, adapted for
% the double-vector SDA model. It also registers and loads the PhaseShift
% model from the local Subsystems folder.

doubleVectorPreviousDir = pwd;
doubleVectorRoot = fileparts(mfilename('fullpath'));
cd(doubleVectorRoot);
doubleVectorRestoreDir = onCleanup(@() cd(doubleVectorPreviousDir));

% Paths below are deliberately relative to the ver_1 folder.
doubleVectorParameterDir = 'Parameter';
doubleVectorSubsystemDir = 'Subsystems_2025b';
doubleVectorFunctionDir = 'functions';
doubleVectorSdaFrameworkDir = 'SDA_Framework';
doubleVectorModelFile = 'FCSMPC_double_vector_2025b.slx';
doubleVectorModelName = 'FCSMPC_double_vector_2025b';
doubleVectorDictionary = 'FOC_params.sldd';

assert(isfile(doubleVectorModelFile), ...
    'double_vector_ini:ModelNotFound', ...
    'Model not found: %s', doubleVectorModelFile);
assert(isfolder(doubleVectorParameterDir), ...
    'double_vector_ini:ParameterFolderNotFound', ...
    'Parameter folder not found: %s', doubleVectorParameterDir);
assert(isfolder(doubleVectorSubsystemDir), ...
    'double_vector_ini:SubsystemFolderNotFound', ...
    'Subsystem folder not found: %s', doubleVectorSubsystemDir);
assert(isfolder(doubleVectorSdaFrameworkDir), ...
    'double_vector_ini:SdaFrameworkFolderNotFound', ...
    'SDA framework folder not found: %s', doubleVectorSdaFrameworkDir);

addpath('.');
addpath(doubleVectorParameterDir);
addpath(doubleVectorSubsystemDir);
addpath(genpath(doubleVectorFunctionDir));
addpath(doubleVectorSdaFrameworkDir);

% FOC_Ini initialization from ver_2. The ordering is intentional because
% later scripts consume fields created earlier.
% AltPWM must also be cleared: after a previous run it is a logical variant
% variable, which would otherwise shadow the Parameter/AltPWM.m script.
clear FOC Plant FCS FOC_Scaling rules AltPWM
FOC.DataType = 'double';

try
    cd(doubleVectorParameterDir);
    Testbenchdata;
    MTPA_MTPV_FFTable;
    PlantTable;
    ControllerParam;
    AltPWM;
    FilterParam;
    Flags;
    LutParamCalc;
    ScalingRules;
    FOC_Scaling = scaleFOCStruct(FOC, "FOC", rules, FOC.DataType);
    cd(doubleVectorRoot);
catch initializationError
    cd(doubleVectorPreviousDir);
    clear doubleVectorRestoreDir
    rethrow(initializationError);
end

% The top-level controller variant uses AltPWM == 0 for FCS-MPC.
AltPWM = false;

% For FCS-MPC. Keep the controller parameters in their original parameter
% script so the initialization structure remains consistent with FOC_Ini.
FCS_Parameter;

% Rebuild the dictionary on every initialization, as in ver_2/FOC_Ini.m.
% createFOC_SLDD removes the previous dictionary, so all Bus definitions
% must be restored immediately afterwards and before either model is loaded.
createFOC_SLDD(doubleVectorModelName, doubleVectorDictionary, FOC_Scaling);
defineSdaPlBuses(doubleVectorDictionary);

% Load PhaseShift explicitly from this version's Subsystems folder. The
% relative path prevents another PhaseShift model on the MATLAB path from
% being selected accidentally.
load_system(doubleVectorModelFile);
currentDictionary = get_param(doubleVectorModelName, 'DataDictionary');
if isempty(currentDictionary) || ...
        ~strcmpi(string(currentDictionary), "FOC_params.sldd")
    set_param(doubleVectorModelName, 'DataDictionary', 'FOC_params.sldd');
    save_system(doubleVectorModelName);
end

% Update Diagram catches missing parameters, invalid variants, Bus errors,
% and Stateflow/MATLAB Function interface errors without starting a run.
set_param(doubleVectorModelName, 'SimulationCommand', 'update');

fprintf('\nDouble-vector initialization complete.\n');
fprintf('  Model       : %s\n', doubleVectorModelFile);
fprintf('  Data type   : %s\n', FOC.DataType);
fprintf('  MPC variant : %d (double_vector_matlab_FSM_Calc)\n', MPC_VARIANT);
fprintf('  Horizon     : Np = %d (%d binary levels)\n', FCS.Np, 3*FCS.Np);
fprintf('  SDA period  : %.9g s\n', FCS.Ts_SDA);
fprintf('  Node limit  : %d\n', FCS.NmaxNode);
fprintf('  Dictionary  : %s\n\n', doubleVectorDictionary);

clear doubleVectorRestoreDir currentDictionary
