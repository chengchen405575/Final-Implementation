function report = complete_sda_comparison_interface_2025b()
%COMPLETE_SDA_COMPARISON_INTERFACE_2025B Build parallel SDA result logging.
% The production Uapply path remains driven by SDA_Solve_Synchronized.

rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
model = 'FCSMPC_double_vector_2025b';
modelFile = fullfile(rootDir,[model '.slx']);
parent = [model '/FCS_MPC/SDA_Solve/SDA Solve'];
pl = [parent '/SDA_PL_Framework'];
reference = [parent '/ SDA_Solve_Synchronized'];

assert(strcmp(version('-release'),'2025b'),'SDA:WrongRelease', ...
    'Run this update only in MATLAB R2025b.');
addpath(rootDir);
addpath(fullfile(rootDir,'Subsystems_2025b'));
addpath(genpath(fullfile(rootDir,'functions')));
addpath(fullfile(rootDir,'SDA','runtime'));
initialize_base_workspace(rootDir);
defineSdaBuses('FOC_params.sldd');

backupDir = fullfile(rootDir,'backups');
if ~isfolder(backupDir), mkdir(backupDir); end
stamp = datestr(now,'yyyymmdd_HHMMSS');
backup = fullfile(backupDir, ...
    ['FCSMPC_double_vector_2025b_before_comparison_' stamp '.slx']);
copyfile(modelFile,backup);

load_system(modelFile);
cleanup = onCleanup(@() close_if_loaded(model)); %#ok<NASGU>
assert(getSimulinkBlockHandle(pl)>0,'SDA:MissingPL','PL framework not found.');
assert(getSimulinkBlockHandle(reference)>0,'SDA:MissingReference', ...
    'SDA_Solve_Synchronized was not found.');

remove_incomplete_comparison(parent);
install_complete_result_buses(parent,pl,reference);
install_validation_start(parent,pl,reference);

set_param(model,'SimulationCommand','update');
save_system(model,modelFile);

report = struct('modelFile',modelFile,'backup',backup, ...
    'generatedAt',datetime('now'));
fprintf('\nSDA comparison interface complete.\n');
fprintf('  model  : %s\n',modelFile);
fprintf('  backup : %s\n',backup);
end

function remove_incomplete_comparison(parent)
% Remove only the two user-created incomplete result bundles and their scopes.
blocks = find_system(parent,'SearchDepth',1,'BlockType','BusCreator');
for k = 1:numel(blocks)
    name = get_param(blocks{k},'Name');
    if endsWith(name,{'Creator3','Creator4'})
        delete_block(blocks{k});
    end
end
for name = {'Scope2','Scope3'}
    path = [parent '/' name{1}];
    if getSimulinkBlockHandle(path)>0, delete_block(path); end
end
names = {'PL_Result_Bus','Reference_Result_Bus','PL_Handshake_Bus', ...
    'PL_Result_Log','Reference_Result_Log','PL_Handshake_Log', ...
    'PL_Handshake_Vector', ...
    'PL_Start_Log','PL_Start_Request','PL_NotBusy','PL_Start_Gate', ...
    'PL_Start_Boolean','PL_BusyDelay', ...
    'Ref_nodes_uint32','Ref_leafNodes_uint32', ...
    'Ref_switchCountApplied_uint32'};
for k = 1:numel(names)
    path = [parent '/' names{k}];
    if getSimulinkBlockHandle(path)>0, delete_block(path); end
end
end

function install_complete_result_buses(parent,pl,reference)
fields = {'Jopt','nodes','optimal','Uopt','Uapply', ...
    'predictionValidNext','iabcPredictedNext','leafNodes', ...
    'bestCMVCost','cmvMetricRatio','switchCountApplied', ...
    'debug_lower','debug_upper','Jround','Jshift','solutionToggle'};

plBus = [parent '/PL_Result_Bus'];
refBus = [parent '/Reference_Result_Bus'];
hsBus = [parent '/PL_Handshake_Bus'];
add_block('simulink/Signal Routing/Bus Creator',plBus, ...
    'Inputs','16','OutDataTypeStr','Bus: SdaComparisonResultBus', ...
    'NonVirtualBus','on','Position',[1260 515 1265 835]);
add_block('simulink/Signal Routing/Bus Creator',refBus, ...
    'Inputs','16','OutDataTypeStr','Bus: SdaComparisonResultBus', ...
    'NonVirtualBus','on','Position',[1260 85 1265 405]);
add_block('simulink/Signal Routing/Bus Creator',hsBus, ...
    'Inputs','3','Position',[1260 875 1265 935]);

plHandles = get_param(pl,'PortHandles');
refHandles = get_param(reference,'PortHandles');
plBusHandles = get_param(plBus,'PortHandles');
refBusHandles = get_param(refBus,'PortHandles');
hsHandles = get_param(hsBus,'PortHandles');

for k = 1:16
    add_named_branch(parent,plHandles.Outport(k+1), ...
        plBusHandles.Inport(k),fields{k});
    if any(k==[2 8 11])
        converterNames = containers.Map([2 8 11], ...
            {'Ref_nodes_uint32','Ref_leafNodes_uint32', ...
             'Ref_switchCountApplied_uint32'});
        converter = converterNames(k);
        y = 85+20*(k-1);
        add_block('simulink/Signal Attributes/Data Type Conversion', ...
            [parent '/' converter],'OutDataTypeStr','uint32', ...
            'Position',[1135 y 1200 y+22]);
        converterHandles = get_param([parent '/' converter],'PortHandles');
        add_named_branch(parent,refHandles.Outport(k), ...
            converterHandles.Inport,fields{k});
        add_named_branch(parent,converterHandles.Outport, ...
            refBusHandles.Inport(k),fields{k});
    else
        add_named_branch(parent,refHandles.Outport(k), ...
            refBusHandles.Inport(k),fields{k});
    end
end
handshakeFields = {'busy','resultValid','fault'};
for k = 1:3
    add_named_branch(parent,plHandles.Outport(17+k), ...
        hsHandles.Inport(k),handshakeFields{k});
end

add_block('simulink/Sinks/Terminator',[parent '/PL_Result_Log'], ...
    'Position',[1380 610 1400 630]);
add_block('simulink/Sinks/Terminator',[parent '/Reference_Result_Log'], ...
    'Position',[1380 180 1400 200]);
add_workspace_log(parent,'PL_Handshake_Log','sda_pl_handshake_log', ...
    [1360 890 1495 920]);
add_block('simulink/Signal Attributes/Bus to Vector', ...
    [parent '/PL_Handshake_Vector'],'Position',[1300 885 1335 925]);
add_logged_line(parent,'PL_Result_Bus/1','PL_Result_Log/1', ...
    'sda_pl_result_log');
add_logged_line(parent,'Reference_Result_Bus/1','Reference_Result_Log/1', ...
    'sda_reference_result_log');
add_line(parent,'PL_Handshake_Bus/1','PL_Handshake_Vector/1', ...
    'autorouting','on');
add_line(parent,'PL_Handshake_Vector/1','PL_Handshake_Log/1', ...
    'autorouting','on');

note = sprintf(['Validation only:\nReference is sampled when start rises.\n' ...
    'PL result is valid only when resultValid is high.']);
a = Simulink.Annotation(parent,note);
a.Position = [1300 420 1580 490];
end

function install_validation_start(parent,pl,reference)
% Reference solutionToggle rises every second 40-kHz transaction.  Gate it
% with ~busy so PL receives only a one-cycle request while idle.
add_block('simulink/Logic and Bit Operations/Detect Rise Positive', ...
    [parent '/PL_Start_Request'],'Position',[930 820 970 850]);
add_block('simulink/Logic and Bit Operations/Logical Operator', ...
    [parent '/PL_NotBusy'],'Operator','NOT','Position',[1010 875 1050 905]);
add_block('simulink/Discrete/Unit Delay',[parent '/PL_BusyDelay'], ...
    'InitialCondition','false','Position',[945 875 985 905]);
add_block('simulink/Logic and Bit Operations/Logical Operator', ...
    [parent '/PL_Start_Gate'],'Operator','AND','Inputs','2', ...
    'Position',[1090 825 1130 865]);
add_workspace_log(parent,'PL_Start_Log','sda_pl_start_log', ...
    [1170 790 1305 820]);

refHandles = get_param(reference,'PortHandles');
plHandles = get_param(pl,'PortHandles');
add_named_branch(parent,refHandles.Outport(16), ...
    get_param([parent '/PL_Start_Request'],'PortHandles').Inport, ...
    'solutionToggle');
add_named_branch(parent,plHandles.Outport(18), ...
    get_param([parent '/PL_BusyDelay'],'PortHandles').Inport,'busy');
add_line(parent,'PL_BusyDelay/1','PL_NotBusy/1','autorouting','on');
add_line(parent,'PL_Start_Request/1','PL_Start_Gate/1','autorouting','on');
add_line(parent,'PL_NotBusy/1','PL_Start_Gate/2','autorouting','on');
add_named_branch(parent, ...
    get_param([parent '/PL_Start_Gate'],'PortHandles').Outport, ...
    plHandles.Inport(12),'plStart');
add_named_branch(parent, ...
    get_param([parent '/PL_Start_Gate'],'PortHandles').Outport, ...
    get_param([parent '/PL_Start_Log'],'PortHandles').Inport,'plStart');
end

function add_workspace_log(parent,name,variable,position)
add_block('simulink/Sinks/To Workspace',[parent '/' name], ...
    'VariableName',variable,'SaveFormat','Structure With Time', ...
    'MaxDataPoints','inf','Position',position);
end

function add_named_branch(system,source,destination,name)
line = add_line(system,source,destination,'autorouting','on');
try, set_param(line,'Name',name); catch, end
end

function line = add_logged_line(system,source,destination,name)
line = add_line(system,source,destination,'autorouting','on');
sourcePort = get_param(line,'SrcPortHandle');
set_param(sourcePort,'DataLogging','on', ...
    'DataLoggingNameMode','Custom','DataLoggingName',name);
end

function initialize_base_workspace(rootDir)
parameterDir = fullfile(rootDir,'Parameter');
paths = {rootDir,parameterDir,fullfile(rootDir,'Subsystems_2025b')};
for k = 1:numel(paths)
    evalin('base',sprintf('addpath(''%s'');',strrep(paths{k},'''','''''')));
end
evalin('base',sprintf('addpath(genpath(''%s''));',strrep( ...
    fullfile(rootDir,'functions'),'''','''''')));
evalin('base','clear FOC Plant FCS FOC_Scaling rules AltPWM');
evalin('base','FOC.DataType=''double'';');
scripts = {'Testbenchdata','MTPA_MTPV_FFTable','PlantTable', ...
    'ControllerParam','AltPWM','FilterParam','Flags','LutParamCalc', ...
    'ScalingRules'};
for k = 1:numel(scripts)
    file = fullfile(parameterDir,[scripts{k} '.m']);
    evalin('base',sprintf('run(''%s'');',strrep(file,'''','''''')));
end
evalin('base','AltPWM=false;');
file = fullfile(parameterDir,'FCS_Parameter.m');
evalin('base',sprintf('run(''%s'');',strrep(file,'''','''''')));
end

function close_if_loaded(model)
if bdIsLoaded(model), close_system(model,0); end
end
