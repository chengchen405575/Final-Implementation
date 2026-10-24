function report = upgrade_sda_pl_interface_2025b()
%UPGRADE_SDA_PL_INTERFACE_2025B Complete the SDA PL interface and handshake.
% The legacy SDA_Solve_Synchronized path is not modified.  This function:
%   * adds solutionToggle to ResultCommit/SdaResultBus;
%   * replaces FrameworkStart with a rising-edge start transaction input;
%   * makes resultValid a one-cycle DONE pulse;
%   * exposes the legacy-compatible result ports plus busy/resultValid/fault;
%   * replaces long/fanout top-level PL wires with local Goto/From routing.

% Run only with MATLAB R2025b.

rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
model = 'FCSMPC_double_vector_2025b';
modelFile = fullfile(rootDir, [model '.slx']);
dictionaryFile = fullfile(rootDir, 'FOC_params.sldd');
pl = [model '/FCS_MPC/SDA_Solve/SDA Solve/SDA_PL_Framework'];

assert(strcmp(version('-release'),'2025b'), ...
    'SDA:WrongRelease','This upgrade must run in MATLAB R2025b.');

addpath(rootDir);
addpath(fullfile(rootDir,'Subsystems_2025b'));
addpath(genpath(fullfile(rootDir,'functions')));
addpath(genpath(fullfile(rootDir,'SDA')));
initialize_base_workspace(rootDir);

backupDir = fullfile(rootDir,'backups');
if ~isfolder(backupDir), mkdir(backupDir); end
stamp = datestr(now,'yyyymmdd_HHMMSS');
modelBackup = fullfile(backupDir, ...
    ['FCSMPC_double_vector_2025b_before_pl_interface_' stamp '.slx']);
dictionaryBackup = fullfile(backupDir, ...
    ['FOC_params_before_pl_interface_' stamp '.sldd']);
copyfile(modelFile,modelBackup);
copyfile(dictionaryFile,dictionaryBackup);

load_system(modelFile);
cleanup = onCleanup(@() close_if_loaded(model)); %#ok<NASGU>
assert(getSimulinkBlockHandle(pl)>0,'SDA:MissingFramework', ...
    'SDA_PL_Framework was not found in the current model.');

% Update the dictionary first, so ResultCommit can compile with the new field.
defineSdaBuses('FOC_params.sldd');

upgrade_result_commit(pl);
upgrade_fsm_done_pulse(pl);
install_start_handshake(pl);
install_result_interface(pl);
route_major_nets_with_tags(pl);

set_param(model,'SimulationCommand','update');
save_system(model,modelFile);

report = struct;
report.modelFile = modelFile;
report.modelBackup = modelBackup;
report.dictionaryBackup = dictionaryBackup;
report.framework = pl;
report.generatedAt = datetime('now');
fprintf('\nSDA PL interface upgrade complete.\n');
fprintf('  model      : %s\n',modelFile);
fprintf('  model copy : %s\n',modelBackup);
fprintf('  dict copy  : %s\n',dictionaryBackup);
end

function upgrade_result_commit(pl)
rt = sfroot;
charts = rt.find('-isa','Stateflow.EMChart');
chart = [];
for k = 1:numel(charts)
    if strcmp(charts(k).Path,[pl '/ResultCommit']) && ...
            strcmp(charts(k).Name,'ResultCommit')
        chart = charts(k); break;
    end
end
assert(~isempty(chart),'SDA:MissingResultCommit','ResultCommit chart not found.');
code = chart.Script;
if ~contains(code,'state.solutionToggle')
    code = strrep(code, ...
        ' state.Jround=0.0; state.Jshift=0.0; state.predictionValidNext=false;', ...
        [' state.Jround=0.0; state.Jshift=0.0; ' ...
         'state.predictionValidNext=false; state.solutionToggle=false;']);
    code = strrep(code, ...
        ' state.Jround=Jround; state.Jshift=Jshift; state.predictionValidNext=best.bestValid;', ...
        [' state.Jround=Jround; state.Jshift=Jshift; ' ...
         'state.predictionValidNext=best.bestValid;' newline ...
         ' state.solutionToggle=~state.solutionToggle;']);
    chart.Script = code;
end
end

function upgrade_fsm_done_pulse(pl)
rt = sfroot;
chart = rt.find('-isa','Stateflow.Chart','Path',[pl '/SearchFSM']);
assert(~isempty(chart),'SDA:MissingSearchFSM','SearchFSM chart not found.');
states = chart.find('-isa','Stateflow.State');
for k = 1:numel(states)
    if strcmp(states(k).Name,'DONE')
        states(k).LabelString = sprintf([ ...
            'DONE\nentry: loadProblem=false; initStart=false; ' ...
            'quadStart=false; vectorStart=false; memClear=false; ' ...
            'memWrite=false; bestWrite=false; commitResult=false; ' ...
            'busy=false; resultValid=true; fault=false;']);
    end
end
transitions = chart.find('-isa','Stateflow.Transition');
changed = false;
for k = 1:numel(transitions)
    src = transitions(k).Source;
    dst = transitions(k).Destination;
    if ~isempty(src) && ~isempty(dst) && ...
            isa(src,'Stateflow.State') && isa(dst,'Stateflow.State') && ...
            strcmp(src.Name,'DONE') && strcmp(dst.Name,'IDLE')
        transitions(k).LabelString = '';
        changed = true;
    end
end
assert(changed,'SDA:MissingDoneTransition','DONE-to-IDLE transition not found.');
end

function install_start_handshake(pl)
delete_if_present([pl '/FrameworkStart']);
delete_if_present([pl '/start']);
delete_if_present([pl '/StartDelay']);
delete_if_present([pl '/StartNot']);
delete_if_present([pl '/StartPulseAnd']);
delete_if_present([pl '/Goto_startPulse']);
delete_if_present([pl '/From_startPulse']);

add_block('simulink/Ports & Subsystems/In1',[pl '/start'], ...
    'Port','12','OutDataTypeStr','boolean','Position',[25 455 55 469]);
add_block('simulink/Discrete/Unit Delay',[pl '/StartDelay'], ...
    'InitialCondition','false','Position',[95 455 130 485]);
add_block('simulink/Logic and Bit Operations/Logical Operator', ...
    [pl '/StartNot'],'Operator','NOT','Position',[155 475 190 505]);
add_block('simulink/Logic and Bit Operations/Logical Operator', ...
    [pl '/StartPulseAnd'],'Operator','AND','Inputs','2', ...
    'Position',[220 440 255 480]);
add_block('simulink/Signal Routing/Goto',[pl '/Goto_startPulse'], ...
    'GotoTag','sda_start_pulse','TagVisibility','local', ...
    'Position',[285 447 390 473]);
add_block('simulink/Signal Routing/From',[pl '/From_startPulse'], ...
    'GotoTag','sda_start_pulse','Position',[700 35 805 61]);

add_named_line(pl,'start/1','StartDelay/1','start_request');
add_named_line(pl,'start/1','StartPulseAnd/1','start_request');
add_named_line(pl,'StartDelay/1','StartNot/1','start_previous');
add_named_line(pl,'StartNot/1','StartPulseAnd/2','start_previous_not');
add_named_line(pl,'StartPulseAnd/1','Goto_startPulse/1','start_pulse');
disconnect_inport([pl '/SearchFSM'],1);
add_named_line(pl,'From_startPulse/1','SearchFSM/1','start_pulse');
end

function install_result_interface(pl)
delete_if_present([pl '/ResultBusSelector']);

legacyFields = {'Jopt','nodes','optimal','Uopt','Uapply', ...
    'predictionValidNext','iabcPredictedNext','leafNodes', ...
    'bestCMVCost','cmvMetricRatio','switchCountApplied', ...
    'debugLower','debugUpper','Jround','Jshift','solutionToggle'};
legacyPortNames = {'Jopt','nodes','optimal','Uopt','Uapply', ...
    'predictionValidNext','iabcPredictedNext','leafNodes', ...
    'bestCMVCost','cmvMetricRatio','switchCountApplied', ...
    'debug_lower','debug_upper','Jround','Jshift','solutionToggle'};

add_block('simulink/Signal Routing/Bus Selector',[pl '/ResultBusSelector'], ...
    'OutputSignals',strjoin(legacyFields,','), ...
    'Position',[1190 690 1200 1060]);
add_named_line(pl,'ResultCommit/1','ResultBusSelector/1','sda_result_bus');

for k = 1:numel(legacyFields)
    tag = ['sda_result_' legacyFields{k}];
    gotoName = ['Goto_result_' legacyFields{k}];
    fromName = ['From_result_' legacyFields{k}];
    outName = legacyPortNames{k};
    delete_if_present([pl '/' gotoName]);
    delete_if_present([pl '/' fromName]);
    delete_if_present([pl '/' outName]);
    y = 690 + 23*(k-1);
    add_block('simulink/Ports & Subsystems/Out1',[pl '/' outName], ...
        'Port',num2str(k+1),'Position',[1395 y 1425 y+18]);
    add_named_line(pl,['ResultBusSelector/' num2str(k)], ...
        [outName '/1'],legacyFields{k});
end

handshake = { ...
    'busy',9,'sda_busy',18; ...
    'resultValid',10,'sda_result_valid',19; ...
    'fault',11,'sda_fault',20};
for k = 1:size(handshake,1)
    name = handshake{k,1}; sourcePort = handshake{k,2};
    tag = handshake{k,3}; outPort = handshake{k,4};
    gotoName = ['Goto_' name]; fromName = ['From_' name];
    delete_if_present([pl '/' gotoName]);
    delete_if_present([pl '/' fromName]);
    delete_if_present([pl '/' name]);
    y = 1085 + 30*(k-1);
    add_block('simulink/Signal Routing/Goto',[pl '/' gotoName], ...
        'GotoTag',tag,'TagVisibility','local', ...
        'Position',[1050 y 1165 y+18]);
    add_block('simulink/Signal Routing/From',[pl '/' fromName], ...
        'GotoTag',tag,'Position',[1395 y 1515 y+18]);
    add_block('simulink/Ports & Subsystems/Out1',[pl '/' name], ...
        'Port',num2str(outPort),'Position',[1550 y 1580 y+18]);
    add_named_line(pl,['SearchFSM/' num2str(sourcePort)], ...
        [gotoName '/1'],name);
    add_named_line(pl,[fromName '/1'],[name '/1'],name);
end
end

function route_major_nets_with_tags(pl)
% Convert only long or fanout top-level nets. Local calculation wires remain
% direct, which avoids replacing a readable short connection with two blocks.
sources = { ...
    'ProblemLatch',1,'problem_latched'; ...
    'BranchMemory',1,'branch_memory'; ...
    'BestMemory',1,'best_memory'; ...
    'ResultCommit',1,'result_bus'; ...
    'SdaInitializer',1,'init_done'; ...
    'SdaInitializer',2,'init_valid'; ...
    'QuadraticCalc',1,'quad_done'; ...
    'QuadraticCalc',2,'quad_valid'; ...
    'VectorCostCalc',1,'vector_done'; ...
    'VectorCostCalc',2,'vector_valid'; ...
    'MetricCompare',1,'current_metric'; ...
    'MetricCompare',2,'current_cmv_metric'; ...
    'MetricCompare',3,'current_better'; ...
    'MetricCompare',4,'current_prune'; ...
    'SearchCounter',1,'node_count'; ...
    'SearchCounter',2,'leaf_count'; ...
    'SearchCounter',3,'node_limit_reached'};
for k = 1:size(sources,1)
    convert_net_if_complex(pl,sources{k,1},sources{k,2},sources{k,3});
end
end

function convert_net_if_complex(system,block,port,tag)
path = [system '/' block];
if getSimulinkBlockHandle(path)<=0, return; end
ph = get_param(path,'PortHandles');
if numel(ph.Outport)<port, return; end
line = get_param(ph.Outport(port),'Line');
if line==-1, return; end
dst = get_param(line,'DstPortHandle');
dst = dst(dst~=-1);
if isempty(dst), return; end
srcPos = get_param(path,'Position');
isLong = false;
for k = 1:numel(dst)
    dstBlock = get_param(dst(k),'Parent');
    dstPos = get_param(dstBlock,'Position');
    if abs(dstPos(1)-srcPos(3))+abs(dstPos(2)-srcPos(2)) > 350
        isLong = true; break;
    end
end
if numel(dst)==1 && ~isLong, return; end

safeTag = matlab.lang.makeValidName(['sda_' tag]);
gotoName = unique_block_name(system,['Goto_' safeTag]);
add_block('simulink/Signal Routing/Goto',[system '/' gotoName], ...
    'GotoTag',safeTag,'TagVisibility','local', ...
    'Position',[srcPos(3)+25 srcPos(2) srcPos(3)+145 srcPos(2)+20]);

delete_line(line);
add_named_line(system,[block '/' num2str(port)],[gotoName '/1'],tag);
for k = 1:numel(dst)
    dstBlock = get_param(dst(k),'Parent');
    dstName = get_param(dstBlock,'Name');
    dstPort = get_param(dst(k),'PortNumber');
    dstPos = get_param(dstBlock,'Position');
    fromName = unique_block_name(system, ...
        ['From_' safeTag '_' matlab.lang.makeValidName(dstName)]);
    add_block('simulink/Signal Routing/From',[system '/' fromName], ...
        'GotoTag',safeTag, ...
        'Position',[max(5,dstPos(1)-145) dstPos(2) max(100,dstPos(1)-25) dstPos(2)+20]);
    add_named_line(system,[fromName '/1'], ...
        [dstName '/' num2str(dstPort)],tag);
end
end

function name = unique_block_name(system,base)
name = base; suffix = 1;
while getSimulinkBlockHandle([system '/' name])>0
    name = [base '_' num2str(suffix)]; suffix = suffix+1;
end
end

function line = add_named_line(system,source,destination,name)
line = add_line(system,source,destination,'autorouting','on');
if ~isempty(name)
    try
        set_param(line,'Name',name);
    catch exception
        % Bus Selector outputs inherit their element names and reject an
        % explicit label.  That inherited name is already the desired one.
        if ~contains(exception.message,'Bus Selector')
            rethrow(exception);
        end
    end
end
end

function delete_if_present(path)
if getSimulinkBlockHandle(path)>0, delete_block(path); end
end

function disconnect_inport(block,port)
ph = get_param(block,'PortHandles');
if numel(ph.Inport)<port, return; end
line = get_param(ph.Inport(port),'Line');
if line~=-1, delete_line(line); end
end

function close_if_loaded(model)
if bdIsLoaded(model), close_system(model,0); end
end

function initialize_base_workspace(rootDir)
% Populate only the base-workspace values required to compile the existing
% plant and controller.  Do not call double_vector_ini_2025b here because
% that script rebuilds the dictionary and saves the model independently.
parameterDir = fullfile(rootDir,'Parameter');
evalin('base',sprintf('addpath(''%s'');',strrep(rootDir,'''','''''')));
evalin('base',sprintf('addpath(''%s'');',strrep(parameterDir,'''','''''')));
evalin('base',sprintf('addpath(''%s'');', ...
    strrep(fullfile(rootDir,'Subsystems_2025b'),'''','''''')));
evalin('base',sprintf('addpath(genpath(''%s''));', ...
    strrep(fullfile(rootDir,'functions'),'''','''''')));
evalin('base','clear FOC Plant FCS FOC_Scaling rules AltPWM');
evalin('base','FOC.DataType = ''double'';');
scripts = {'Testbenchdata','MTPA_MTPV_FFTable','PlantTable', ...
    'ControllerParam','AltPWM','FilterParam','Flags','LutParamCalc', ...
    'ScalingRules'};
for k = 1:numel(scripts)
    file = fullfile(parameterDir,[scripts{k} '.m']);
    evalin('base',sprintf('run(''%s'');',strrep(file,'''','''''')));
end
evalin('base','AltPWM = false;');
evalin('base',sprintf('run(''%s'');',strrep( ...
    fullfile(parameterDir,'FCS_Parameter.m'),'''','''''')));
end
