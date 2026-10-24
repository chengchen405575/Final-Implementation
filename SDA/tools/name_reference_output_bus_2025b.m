function report=name_reference_output_bus_2025b()
%NAME_REFERENCE_OUTPUT_BUS_2025B Name the reference result signal and log.
rootDir=fileparts(fileparts(fileparts(mfilename('fullpath'))));
model='FCSMPC_double_vector_2025b';
modelFile=fullfile(rootDir,[model '.slx']);
block=[model '/FCS_MPC/SDA_Solve/SDA Solve/Reference_Result_Bus1'];
assert(strcmp(version('-release'),'2025b'),'SDA:WrongRelease','Use R2025b.');

backupDir=fullfile(rootDir,'backups');
if ~isfolder(backupDir),mkdir(backupDir);end
stamp=datestr(now,'yyyymmdd_HHMMSS');
backup=fullfile(backupDir, ...
    ['FCSMPC_double_vector_2025b_before_reference_bus_name_' stamp '.slx']);
copyfile(modelFile,backup);

load_system(modelFile);
cleanup=onCleanup(@() close_if_loaded(model)); %#ok<NASGU>
assert(getSimulinkBlockHandle(block)>0,'SDA:MissingBus', ...
    'Reference_Result_Bus1 was not found.');
ports=get_param(block,'PortHandles');
line=get_param(ports.Outport,'Line');
assert(line~=-1,'SDA:UnconnectedBus','Reference_Result_Bus1 output is unconnected.');

set_param(line,'Name','SDA_reference_output');
set_param(ports.Outport, ...
    'DataLogging','on', ...
    'DataLoggingNameMode','Custom', ...
    'DataLoggingName','SDA_reference_output');

save_system(model,modelFile);
report=struct('modelFile',modelFile,'backup',backup, ...
    'signalName','SDA_reference_output','logName','SDA_reference_output', ...
    'busType',get_param(block,'OutDataTypeStr'));
fprintf('Reference result signal/log renamed to SDA_reference_output.\n');
fprintf('backup: %s\n',backup);
end
function close_if_loaded(model)
if bdIsLoaded(model),close_system(model,0);end
end
