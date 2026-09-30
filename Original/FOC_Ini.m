clear all;
CurrentPath = cd;
DesPath = [CurrentPath '\Parameter'];   
SubsystemPath = [CurrentPath '\Subsystems']; 
addpath(SubsystemPath);
addpath(DesPath);
cd(DesPath)

FOC.DataType ='double' ; % fixpoint or double 
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
cd(CurrentPath)
slddPath = [CurrentPath '/FOC_params.sldd'];
createFOC_SLDD('FOC_PROC_Kara', slddPath, FOC_Scaling);
% createFOC_SLDD rebuilds the dictionary, so restore the SDA PL interfaces.
defineSdaPlBuses(slddPath);
AltPWM = false;

% For MPC
FCS_Parameter;
