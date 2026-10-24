function report=restore_sda_vector_write_2025b()
%RESTORE_SDA_VECTOR_WRITE_2025B Restore the validated original handshake.
rootDir=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
model='FCSMPC_double_vector_2025b'; modelFile=fullfile(rootDir,[model '.slx']);
pl=[model '/FCS_MPC/SDA_Solve/SDA Solve/SDA_PL_Framework'];
assert(strcmp(version('-release'),'2025b'),'Use R2025b.');
initialize(rootDir); load_system(modelFile);
cleanup=onCleanup(@() close_if_loaded(model)); %#ok<NASGU>
backupDir=fullfile(rootDir,'backups'); if ~isfolder(backupDir),mkdir(backupDir);end
stamp=datestr(now,'yyyymmdd_HHMMSS'); backup=fullfile(backupDir,['FCSMPC_double_vector_2025b_before_restore_vector_write_' stamp '.slx']);
copyfile(modelFile,backup);
rt=sfroot; c=rt.find('-isa','Stateflow.EMChart','Path',[pl '/BranchMemory']);
c.Script=strrep(c.Script,'if vectorWrite&&fsmState==uint8(12)','if vectorWrite');
set_param(model,'SimulationCommand','update'); save_system(model,modelFile);
report=struct('modelFile',modelFile,'backup',backup); fprintf('Original vectorWrite handshake restored.\n');
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
