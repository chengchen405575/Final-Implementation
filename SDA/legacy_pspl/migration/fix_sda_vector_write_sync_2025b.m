function report = fix_sda_vector_write_sync_2025b()
%FIX_SDA_VECTOR_WRITE_SYNC_2025B Prevent stale vector results being written.
rootDir=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
model='FCSMPC_double_vector_2025b';
modelFile=fullfile(rootDir,[model '.slx']);
pl=[model '/FCS_MPC/SDA_Solve/SDA Solve/SDA_PL_Framework'];
assert(strcmp(version('-release'),'2025b'),'SDA:WrongRelease','Use R2025b.');
initialize_base_workspace(rootDir);
addpath(rootDir); addpath(fullfile(rootDir,'Subsystems_2025b'));
addpath(genpath(fullfile(rootDir,'functions')));
backupDir=fullfile(rootDir,'backups'); if ~isfolder(backupDir), mkdir(backupDir); end
stamp=datestr(now,'yyyymmdd_HHMMSS');
backup=fullfile(backupDir,['FCSMPC_double_vector_2025b_before_vector_write_sync_' stamp '.slx']);
copyfile(modelFile,backup);
load_system(modelFile);
cleanup=onCleanup(@() close_if_loaded(model)); %#ok<NASGU>
rt=sfroot;
chart=rt.find('-isa','Stateflow.EMChart','Path',[pl '/BranchMemory']);
assert(~isempty(chart),'SDA:MissingBranchMemory','BranchMemory was not found.');
old='if vectorWrite';
new='if vectorWrite&&fsmState==uint8(12)';
assert(contains(chart.Script,old),'SDA:UnexpectedBranchMemory', ...
    'Expected vectorWrite condition was not found.');
chart.Script=strrep(chart.Script,old,new);
set_param(model,'SimulationCommand','update');
save_system(model,modelFile);
report=struct('modelFile',modelFile,'backup',backup,'generatedAt',datetime('now'));
fprintf('Vector-result write is now gated by VECTOR_WAIT.\n');
fprintf('backup: %s\n',backup);
end

function initialize_base_workspace(rootDir)
parameterDir=fullfile(rootDir,'Parameter');
evalin('base',sprintf('addpath(''%s'');',strrep(rootDir,'''','''''')));
evalin('base',sprintf('addpath(genpath(''%s''));',strrep(fullfile(rootDir,'functions'),'''','''''')));
evalin('base','clear FOC Plant FCS FOC_Scaling rules AltPWM; FOC.DataType=''double'';');
scripts={'Testbenchdata','MTPA_MTPV_FFTable','PlantTable','ControllerParam', ...
    'AltPWM','FilterParam','Flags','LutParamCalc','ScalingRules'};
for k=1:numel(scripts), file=fullfile(parameterDir,[scripts{k} '.m']); evalin('base',sprintf('run(''%s'');',strrep(file,'''',''''''))); end
evalin('base','AltPWM=false;'); file=fullfile(parameterDir,'FCS_Parameter.m');
evalin('base',sprintf('run(''%s'');',strrep(file,'''','''''')));
end
function close_if_loaded(model)
if bdIsLoaded(model), close_system(model,0); end
end
