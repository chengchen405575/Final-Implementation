function report=fix_sda_latched_iabc_initial_2025b()
%FIX_SDA_LATCHED_IABC_INITIAL_2025B Use the accepted transaction current.
rootDir=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
model='FCSMPC_double_vector_2025b'; modelFile=fullfile(rootDir,[model '.slx']);
pl=[model '/FCS_MPC/SDA_Solve/SDA Solve/SDA_PL_Framework'];
assert(strcmp(version('-release'),'2025b'),'SDA:WrongRelease','Use R2025b.');
initialize(rootDir);
backupDir=fullfile(rootDir,'backups'); if ~isfolder(backupDir),mkdir(backupDir);end
stamp=datestr(now,'yyyymmdd_HHMMSS');
backup=fullfile(backupDir,['FCSMPC_double_vector_2025b_before_latched_iabc_' stamp '.slx']);
copyfile(modelFile,backup);
load_system(modelFile); cleanup=onCleanup(@() close_if_loaded(model)); %#ok<NASGU>

selector=[pl '/LatchedIabcSelector'];
delete_if_present([pl '/From_latched_problem_iabc']); delete_if_present(selector);
add_block('simulink/Signal Routing/Bus Selector',selector, ...
    'OutputSignals','iabcCompensated','Position',[650 1000 660 1050]);
lp=get_param([pl '/ProblemLatch'],'PortHandles');
selp=get_param(selector,'PortHandles');
add_line(pl,lp.Outport(1),selp.Inport,'autorouting','on');

branch=[pl '/BranchMemory']; bp=get_param(branch,'PortHandles');
oldLine=get_param(bp.Inport(13),'Line'); if oldLine~=-1,delete_line(oldLine);end
sp=get_param(selector,'PortHandles');
add_line(pl,sp.Outport(1),bp.Inport(13),'autorouting','on');

set_param(model,'SimulationCommand','update'); save_system(model,modelFile);
report=struct('modelFile',modelFile,'backup',backup,'generatedAt',datetime('now'));
fprintf('BranchMemory iabcInitial now comes from the latched problem bus.\n');
end
function delete_if_present(path)
if getSimulinkBlockHandle(path)>0,delete_block(path);end
end
function initialize(rootDir)
p=fullfile(rootDir,'Parameter'); addpath(rootDir); addpath(fullfile(rootDir,'Subsystems_2025b')); addpath(genpath(fullfile(rootDir,'functions')));
evalin('base','clear FOC Plant FCS FOC_Scaling rules AltPWM; FOC.DataType=''double'';');
s={'Testbenchdata','MTPA_MTPV_FFTable','PlantTable','ControllerParam','AltPWM','FilterParam','Flags','LutParamCalc','ScalingRules'};
for k=1:numel(s),f=fullfile(p,[s{k} '.m']);evalin('base',sprintf('run(''%s'');',strrep(f,'''','''''')));end
evalin('base','AltPWM=false;');f=fullfile(p,'FCS_Parameter.m');evalin('base',sprintf('run(''%s'');',strrep(f,'''','''''')));
end
function close_if_loaded(model)
if bdIsLoaded(model),close_system(model,0);end
end
