function build_sda_pl_framework()
%BUILD_SDA_PL_FRAMEWORK Install an FPGA-oriented SDA PL scaffold.
%
% The existing MATLAB Function is retained as Legacy_SDA_Solve and remains
% connected to the public outputs.  The new framework receives the same
% inputs in parallel, so the model remains behaviorally unchanged until the
% new ResultCommit path is deliberately selected.

modelFile = fullfile(fileparts(mfilename('fullpath')), ...
    'FCSMPC_double_vector.slx');
modelName = 'FCSMPC_double_vector';
target = [modelName '/FCS_MPC/SDA_Solve/SDA Solve/SDA/' ...
    'double_vector_matlab_FSM_Calc/SDA_Solve'];

load_system(modelFile);
cleanup = onCleanup(@() close_system(modelName, 0));

if strcmp(get_param(target, 'BlockType'), 'SubSystem') && ...
        isempty(get_param(target, 'SFBlockType')) && ...
        ~isempty(find_system(target, 'SearchDepth', 1, ...
        'Name', 'PL_Framework'))
    error('SDA:FrameworkExists', ...
        'The PL framework is already installed at %s.', target);
end

backupDir = fullfile(fileparts(modelFile), 'backups');
if ~exist(backupDir, 'dir')
    mkdir(backupDir);
end
stamp = datestr(now, 'yyyymmdd_HHMMSS');
copyfile(modelFile, fullfile(backupDir, ...
    ['FCSMPC_double_vector_before_PL_framework_' stamp '.slx']));
dictionaryName = get_param(modelName, 'DataDictionary');
if ~isempty(dictionaryName)
    dictionaryFile = which(dictionaryName);
    if isempty(dictionaryFile)
        dictionaryFile = fullfile(fileparts(modelFile), dictionaryName);
    end
    copyfile(dictionaryFile, fullfile(backupDir, ...
        ['FOC_params_before_PL_framework_' stamp '.sldd']));
end

create_bus_objects(modelName);
replace_legacy_solver(target);
if evalin('base', "~exist('MPC_VARIANT','var')")
    assignin('base', 'MPC_VARIANT', 2);
end
set_param(modelName, 'SimulationCommand', 'update');
save_system(modelName, modelFile);
fprintf('Installed SDA PL framework in %s\n', modelFile);
end

function replace_legacy_solver(target)
parent = get_param(target, 'Parent');
oldPosition = get_param(target, 'Position');
oldPorts = get_param(target, 'PortHandles');

inSources = cell(1, numel(oldPorts.Inport));
for k = 1:numel(oldPorts.Inport)
    line = get_param(oldPorts.Inport(k), 'Line');
    if line == -1
        inSources{k} = [];
    else
        inSources{k} = get_param(line, 'SrcPortHandle');
    end
end
outDestinations = cell(1, numel(oldPorts.Outport));
outLines = -ones(1, numel(oldPorts.Outport));
for k = 1:numel(oldPorts.Outport)
    line = get_param(oldPorts.Outport(k), 'Line');
    outLines(k) = line;
    if line == -1
        outDestinations{k} = [];
    else
        outDestinations{k} = get_param(line, 'DstPortHandle');
    end
end

temporary = [parent '/SDA_Solve_PL_BUILD'];
if getSimulinkBlockHandle(temporary) > 0
    delete_block(temporary);
end
add_block('simulink/Ports & Subsystems/Subsystem', temporary, ...
    'Position', oldPosition, 'TreatAsAtomicUnit', 'on');
clear_subsystem(temporary);

inputNames = {'H','U_unc','NmaxNod','UoptPrev','iabcCompensated', ...
    'deltaIabcBase','deltaIabcSwitch','lambdaZero','lambdaDeadtime', ...
    'deadtime_ratio','currentEpsMargin'};
outputNames = {'Jopt','nodes','optimal','Uopt','Uapply','leafNodes', ...
    'bestCMVCost','cmvMetricRatio','switchCountApplied','debug_lower', ...
    'debug_upper','iabcPredictedNext','Jround','Jshift', ...
    'predictionValidNext'};

for k = 1:numel(inputNames)
    add_block('simulink/Ports & Subsystems/In1', ...
        [temporary '/' inputNames{k}], 'Port', num2str(k), ...
        'Position', [30 35+45*(k-1) 60 49+45*(k-1)]);
end
for k = 1:numel(outputNames)
    add_block('simulink/Ports & Subsystems/Out1', ...
        [temporary '/' outputNames{k}], 'Port', num2str(k), ...
        'Position', [1050 25+37*(k-1) 1080 39+37*(k-1)]);
end

legacy = [temporary '/Legacy_SDA_Solve'];
add_block(target, legacy, 'Position', [350 55 520 515]);
legacyPorts = get_param(legacy, 'PortHandles');
for k = 1:numel(inputNames)
    add_line(temporary, [inputNames{k} '/1'], ...
        ['Legacy_SDA_Solve/' num2str(k)], 'autorouting', 'on');
end
for k = 1:numel(outputNames)
    add_line(temporary, ['Legacy_SDA_Solve/' num2str(k)], ...
        [outputNames{k} '/1'], 'autorouting', 'on');
end

framework = [temporary '/PL_Framework'];
add_block('simulink/Ports & Subsystems/Subsystem', framework, ...
    'Position', [650 55 940 515], 'TreatAsAtomicUnit', 'on', ...
    'BackgroundColor', 'lightBlue');
clear_subsystem(framework);
build_framework(framework, inputNames);
for k = 1:numel(inputNames)
    add_line(temporary, [inputNames{k} '/1'], ...
        ['PL_Framework/' num2str(k)], 'autorouting', 'on');
end
add_block('simulink/Sinks/Terminator', [temporary '/FrameworkDebugTerminator'], ...
    'Position', [970 283 990 303]);
add_line(temporary, 'PL_Framework/1', 'FrameworkDebugTerminator/1', ...
    'autorouting', 'on');

note = sprintf(['PL scaffold runs in parallel.\n' ...
    'Legacy_SDA_Solve still drives all public outputs.\n' ...
    'Switch to ResultCommit only after numerical equivalence is proven.']);
annotation = Simulink.Annotation(temporary, note);
annotation.Position = [610 530 1010 595];

for k = 1:numel(outLines)
    if outLines(k) ~= -1 && ishandle(outLines(k))
        delete_line(outLines(k));
    end
end
delete_block(target);
set_param(temporary, 'Name', 'SDA_Solve');
newTarget = [parent '/SDA_Solve'];
newPorts = get_param(newTarget, 'PortHandles');
for k = 1:numel(inSources)
    if ~isempty(inSources{k})
        add_line(parent, inSources{k}, newPorts.Inport(k), 'autorouting', 'on');
    end
end
for k = 1:numel(outDestinations)
    destinations = outDestinations{k};
    for j = 1:numel(destinations)
        add_line(parent, newPorts.Outport(k), destinations(j), ...
            'autorouting', 'on');
    end
end
end

function build_framework(systemPath, inputNames)
for k = 1:numel(inputNames)
    add_block('simulink/Ports & Subsystems/In1', ...
        [systemPath '/' inputNames{k}], 'Port', num2str(k), ...
        'Position', [25 25+38*(k-1) 55 39+38*(k-1)]);
end
add_block('simulink/Ports & Subsystems/Out1', ...
    [systemPath '/DebugBus'], 'Port', '1', ...
    'OutDataTypeStr', 'Bus: SdaDebugBus', ...
    'Position', [1540 255 1570 269]);

add_block('simulink/Signal Routing/Bus Creator', ...
    [systemPath '/ProblemBusCreator'], 'Inputs', '11', ...
    'OutDataTypeStr', 'Bus: SdaProblemBus', ...
    'Position', [125 55 130 435]);
for k = 1:numel(inputNames)
    if k == 3
        add_block('simulink/Signal Attributes/Data Type Conversion', ...
            [systemPath '/NmaxNodeToUint32'], ...
            'OutDataTypeStr', 'uint32', ...
            'Position', [75 91 110 119]);
        add_line(systemPath, [inputNames{k} '/1'], ...
            'NmaxNodeToUint32/1', 'autorouting', 'on');
        add_line(systemPath, 'NmaxNodeToUint32/1', ...
            ['ProblemBusCreator/' num2str(k)], 'autorouting', 'on');
    else
        add_line(systemPath, [inputNames{k} '/1'], ...
            ['ProblemBusCreator/' num2str(k)], 'autorouting', 'on');
    end
end

add_matlab_function(systemPath, 'ProblemLatch', [205 55 385 145], ...
    problem_latch_script());
add_matlab_function(systemPath, 'SdaInitializer', [455 25 650 155], ...
    initializer_script());
add_matlab_function(systemPath, 'QuadraticCalc', [455 205 650 335], ...
    quadratic_script());
add_matlab_function(systemPath, 'VectorCostCalc', [455 385 650 515], ...
    vector_script());
add_matlab_function(systemPath, 'BranchMemory', [790 355 985 500], ...
    branch_memory_script());
add_matlab_function(systemPath, 'BestMemory', [790 535 985 635], ...
    best_memory_script());
add_matlab_function(systemPath, 'ResultCommit', [1190 535 1385 650], ...
    result_script());

chartPath = [systemPath '/SearchFSM'];
add_block('sflib/Chart', chartPath, 'Position', [790 35 1020 285]);
build_search_chart(chartPath);

% Core problem distribution.
add_line(systemPath, 'ProblemBusCreator/1', 'ProblemLatch/1', 'autorouting', 'on');
add_line(systemPath, 'ProblemLatch/1', 'SdaInitializer/1', 'autorouting', 'on');
add_line(systemPath, 'ProblemLatch/1', 'QuadraticCalc/1', 'autorouting', 'on');
add_line(systemPath, 'ProblemLatch/1', 'VectorCostCalc/1', 'autorouting', 'on');
add_line(systemPath, 'ProblemLatch/1', 'ResultCommit/1', 'autorouting', 'on');

% start is a temporary framework transaction source.  It is intentionally
% always true; SearchFSM returns to IDLE only after start is released when
% the real PL wrapper handshake is added.
add_block('simulink/Sources/Constant', [systemPath '/FrameworkStart'], ...
    'Value', 'false', 'OutDataTypeStr', 'boolean', ...
    'SampleTime', 'inf', 'Position', [700 45 735 75]);
add_line(systemPath, 'FrameworkStart/1', 'SearchFSM/1', 'autorouting', 'on');

% Chart commands to modules.
add_line(systemPath, 'SearchFSM/1', 'ProblemLatch/2', 'autorouting', 'on');
add_line(systemPath, 'SearchFSM/2', 'SdaInitializer/2', 'autorouting', 'on');
add_line(systemPath, 'SearchFSM/3', 'QuadraticCalc/2', 'autorouting', 'on');
add_line(systemPath, 'SearchFSM/4', 'VectorCostCalc/2', 'autorouting', 'on');
add_line(systemPath, 'SearchFSM/5', 'BranchMemory/1', 'autorouting', 'on');
add_line(systemPath, 'SearchFSM/6', 'BranchMemory/2', 'autorouting', 'on');
add_line(systemPath, 'SearchFSM/7', 'BestMemory/1', 'autorouting', 'on');
add_line(systemPath, 'SearchFSM/8', 'ResultCommit/2', 'autorouting', 'on');
add_line(systemPath, 'SearchFSM/12', 'QuadraticCalc/3', 'autorouting', 'on');
add_line(systemPath, 'SearchFSM/12', 'VectorCostCalc/3', 'autorouting', 'on');
add_line(systemPath, 'SearchFSM/13', 'QuadraticCalc/4', 'autorouting', 'on');
add_line(systemPath, 'SearchFSM/13', 'VectorCostCalc/4', 'autorouting', 'on');

% Registered status responses prevent combinational control loops.
statusSources = {'SdaInitializer/1','SdaInitializer/2', ...
    'QuadraticCalc/1','QuadraticCalc/2', ...
    'VectorCostCalc/1','VectorCostCalc/2'};
for k = 1:numel(statusSources)
    delayName = ['StatusDelay' num2str(k)];
    add_block('simulink/Discrete/Unit Delay', [systemPath '/' delayName], ...
        'InitialCondition', 'false', ...
        'Position', [690 115+38*(k-1) 725 145+38*(k-1)]);
    add_line(systemPath, statusSources{k}, [delayName '/1'], 'autorouting', 'on');
    add_line(systemPath, [delayName '/1'], ['SearchFSM/' num2str(k+1)], ...
        'autorouting', 'on');
end

% Memory snapshots and calculation results.
add_line(systemPath, 'QuadraticCalc/3', 'BranchMemory/3', 'autorouting', 'on');
add_line(systemPath, 'VectorCostCalc/3', 'BranchMemory/4', 'autorouting', 'on');
add_line(systemPath, 'SdaInitializer/3', 'BranchMemory/5', 'autorouting', 'on');
add_line(systemPath, 'SdaInitializer/4', 'BestMemory/2', 'autorouting', 'on');
add_line(systemPath, 'QuadraticCalc/3', 'BestMemory/3', 'autorouting', 'on');
add_line(systemPath, 'BestMemory/1', 'ResultCommit/3', 'autorouting', 'on');
add_block('simulink/Discrete/Unit Delay', ...
    [systemPath '/BranchMemoryStateDelay'], ...
    'InitialCondition', '0', ...
    'Position', [1015 405 1050 435]);
add_line(systemPath, 'BranchMemory/1', 'BranchMemoryStateDelay/1', ...
    'autorouting', 'on');
add_line(systemPath, 'BranchMemoryStateDelay/1', 'QuadraticCalc/5', ...
    'autorouting', 'on');
add_line(systemPath, 'BranchMemoryStateDelay/1', 'VectorCostCalc/5', ...
    'autorouting', 'on');

% Result bus is retained for the eventual output switchover.
add_block('simulink/Sinks/Terminator', [systemPath '/ResultBusTerminator'], ...
    'Position', [1415 575 1435 595]);
add_line(systemPath, 'ResultCommit/1', 'ResultBusTerminator/1', 'autorouting', 'on');

% Debug bus consists only of stable scalar signals at this scaffold stage.
add_block('simulink/Signal Routing/Bus Creator', ...
    [systemPath '/DebugBusCreator'], 'Inputs', '9', ...
    'OutDataTypeStr', 'Bus: SdaDebugBus', ...
    'Position', [1460 120 1465 405]);
debugSources = {'SearchFSM/14','SearchFSM/12','SearchFSM/13', ...
    'QuadraticCalc/4','QuadraticCalc/5','QuadraticCalc/3', ...
    'VectorCostCalc/3','SearchFSM/15','SearchFSM/11'};
for k = 1:numel(debugSources)
    add_line(systemPath, debugSources{k}, ...
        ['DebugBusCreator/' num2str(k)], 'autorouting', 'on');
end
add_line(systemPath, 'DebugBusCreator/1', 'DebugBus/1', 'autorouting', 'on');

annotation = Simulink.Annotation(systemPath, ...
    sprintf(['FPGA-oriented scaffold\n' ...
    'Blue control: SearchFSM\nOrange datapath: MATLAB Functions\n' ...
    'Status responses are registered by Unit Delay.']));
annotation.Position = [1040 30 1430 105];
end

function add_matlab_function(parent, name, position, script)
path = [parent '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', path, ...
    'Position', position);
root = sfroot;
chart = root.find('-isa', 'Stateflow.EMChart', 'Path', path);
chart.Script = script;
configure_function_interfaces(chart, name);
end

function configure_function_interfaces(chart, name)
switch name
    case 'ProblemLatch'
        set_data(chart, 'problemIn', 'Bus: SdaProblemBus', '[1 1]');
        set_data(chart, 'problem', 'Bus: SdaProblemBus', '[1 1]');
    case 'SdaInitializer'
        set_data(chart, 'problem', 'Bus: SdaProblemBus', '[1 1]');
        set_data(chart, 'z', 'double', '[15 1]');
        set_data(chart, 'initialU', 'double', '[15 1]');
    case 'QuadraticCalc'
        set_data(chart, 'problem', 'Bus: SdaProblemBus', '[1 1]');
        set_data(chart, 'memory', 'Bus: SdaMemoryBus', '[1 1]');
    case 'VectorCostCalc'
        set_data(chart, 'problem', 'Bus: SdaProblemBus', '[1 1]');
        set_data(chart, 'memory', 'Bus: SdaMemoryBus', '[1 1]');
    case 'BranchMemory'
        set_data(chart, 'z', 'double', '[15 1]');
        set_data(chart, 'memory', 'Bus: SdaMemoryBus', '[1 1]');
    case 'BestMemory'
        set_data(chart, 'initialU', 'double', '[15 1]');
        set_data(chart, 'best', 'Bus: SdaBestBus', '[1 1]');
    case 'ResultCommit'
        set_data(chart, 'problem', 'Bus: SdaProblemBus', '[1 1]');
        set_data(chart, 'best', 'Bus: SdaBestBus', '[1 1]');
        set_data(chart, 'result', 'Bus: SdaResultBus', '[1 1]');
end
end

function set_data(chart, dataName, dataType, sizeText)
data = chart.find('-isa', 'Stateflow.Data', 'Name', dataName);
if ~isempty(data)
    data.DataType = dataType;
    data.Props.Array.Size = sizeText;
end
end

function build_search_chart(chartPath)
root = sfroot;
chart = root.find('-isa', 'Stateflow.Chart', 'Path', chartPath);
chart.ActionLanguage = 'MATLAB';
chart.ChartUpdate = 'DISCRETE';
chart.SampleTime = '-1';

inputs = {'start','initDone','initValid','quadDone','quadValid', ...
    'vectorDone','vectorValid'};
for k = 1:numel(inputs)
    data = Stateflow.Data(chart);
    data.Name = inputs{k};
    data.Scope = 'Input';
    data.Port = k;
    data.DataType = 'boolean';
    data.Props.Array.Size = '[1 1]';
end

outputs = {'loadProblem','initStart','quadStart','vectorStart', ...
    'memClear','memWrite','bestWrite','commitResult','busy', ...
    'resultValid','fault','level','candidate','fsmState','cycleCount'};
for k = 1:numel(outputs)
    data = Stateflow.Data(chart);
    data.Name = outputs{k};
    data.Scope = 'Output';
    data.Port = k;
    if ismember(outputs{k}, {'level','fsmState'})
        data.DataType = 'uint8';
    elseif strcmp(outputs{k}, 'cycleCount')
        data.DataType = 'uint32';
    else
        data.DataType = 'boolean';
    end
    data.Props.Array.Size = '[1 1]';
end

names = {'IDLE','LOAD','INIT_START','INIT_WAIT','CHECK_LEVEL', ...
    'CENTER_START','CENTER_WAIT','SELECT_CANDIDATE','METRIC_START', ...
    'METRIC_WAIT','VECTOR_CHECK','VECTOR_START','VECTOR_WAIT', ...
    'COMPARE','UPDATE_BEST','DESCEND','NEXT_CANDIDATE', ...
    'BACKTRACK','WRITE_RESULT','DONE','FAULT'};
positions = [ ...
     20  20; 170  20; 320  20; 470  20; ...
     20 150; 170 150; 320 150; 470 150; ...
    620 150; 770 150; 920 150;1070 150;1220 150; ...
     20 280; 170 280; 320 280; 470 280; 620 280; ...
    770 410; 920 410;1070 410];
states = cell(1, numel(names));
for k = 1:numel(names)
    state = Stateflow.State(chart);
    state.Name = names{k};
    state.Position = [positions(k,1) positions(k,2) 115 65];
    code = state_code(names{k}, k-1);
    state.LabelString = sprintf('%s\n%s', names{k}, code);
    states{k} = state;
end

defaultTransition = Stateflow.Transition(chart);
defaultTransition.Destination = states{1};
defaultTransition.DestinationOClock = 0;
defaultTransition.SourceEndPoint = [5 45];
defaultTransition.MidPoint = [10 45];

connect(chart, states{1}, states{2}, '[start]');
connect(chart, states{2}, states{3}, '');
connect(chart, states{3}, states{4}, '');
connect(chart, states{4}, states{5}, '[initDone && initValid]');
connect(chart, states{4}, states{21}, '[initDone && ~initValid]');
connect(chart, states{5}, states{6}, '');
connect(chart, states{6}, states{7}, '');
connect(chart, states{7}, states{8}, '[quadDone && quadValid]');
connect(chart, states{7}, states{21}, '[quadDone && ~quadValid]');
connect(chart, states{8}, states{9}, '');
connect(chart, states{9}, states{10}, '');
connect(chart, states{10}, states{11}, '[quadDone && quadValid]');
connect(chart, states{10}, states{21}, '[quadDone && ~quadValid]');
connect(chart, states{11}, states{12}, '[mod(level,uint8(3))==0]');
connect(chart, states{11}, states{14}, '[mod(level,uint8(3))~=0]');
connect(chart, states{12}, states{13}, '');
connect(chart, states{13}, states{14}, '[vectorDone && vectorValid]');
connect(chart, states{13}, states{21}, '[vectorDone && ~vectorValid]');
connect(chart, states{14}, states{15}, '[level==uint8(15)]');
connect(chart, states{14}, states{16}, '[level<uint8(15)]');
connect(chart, states{15}, states{17}, '');
connect(chart, states{16}, states{5}, '');
connect(chart, states{17}, states{18}, '');
connect(chart, states{18}, states{19}, '[level==uint8(0)]');
connect(chart, states{18}, states{5}, '[level>uint8(0)]');
connect(chart, states{19}, states{20}, '');
connect(chart, states{20}, states{1}, '[~start]');
connect(chart, states{21}, states{1}, '[~start]');
end

function code = state_code(name, id)
defaults = ['during: loadProblem=false; initStart=false; quadStart=false; ' ...
    'vectorStart=false; memClear=false; memWrite=false; bestWrite=false; ' ...
    'commitResult=false; resultValid=false; fault=false; ' ...
    'fsmState=uint8(' num2str(id) '); cycleCount=cycleCount+uint32(1);'];
switch name
    case 'IDLE'
        code = ['entry: level=uint8(1); candidate=false; cycleCount=uint32(0); ' ...
            'busy=false; resultValid=false; fault=false;'];
    case 'LOAD'
        code = ['entry: busy=true; loadProblem=true; memClear=true; ' ...
            'level=uint8(1); cycleCount=uint32(0);'];
    case 'INIT_START'
        code = 'entry: initStart=true;';
    case {'CENTER_START','METRIC_START'}
        code = 'entry: quadStart=true;';
    case 'SELECT_CANDIDATE'
        code = 'entry: candidate=~candidate; memWrite=true;';
    case 'VECTOR_START'
        code = 'entry: vectorStart=true;';
    case 'UPDATE_BEST'
        code = 'entry: bestWrite=true;';
    case 'DESCEND'
        code = 'entry: level=level+uint8(1); memWrite=true;';
    case 'BACKTRACK'
        code = ['entry: if level>uint8(0); level=level-uint8(1); end; ' ...
            'memWrite=true;'];
    case 'WRITE_RESULT'
        code = 'entry: commitResult=true;';
    case 'DONE'
        code = 'entry: busy=false; resultValid=true;';
    case 'FAULT'
        code = 'entry: busy=false; fault=true;';
    otherwise
        code = defaults;
end
end

function connect(chart, source, destination, label)
transition = Stateflow.Transition(chart);
transition.Source = source;
transition.Destination = destination;
transition.LabelString = label;
transition.SourceOClock = 3;
transition.DestinationOClock = 9;
end

function create_bus_objects(modelName)
dictionaryName = get_param(modelName, 'DataDictionary');
if isempty(dictionaryName)
    error('SDA:NoDictionary', ...
        'The model must use a data dictionary for persistent Bus objects.');
end
dictionary = Simulink.data.dictionary.open(dictionaryName);
dictionaryCleanup = onCleanup(@() close(dictionary));
section = getSection(dictionary, 'Design Data');

problem = Simulink.Bus;
problem.Elements = [ ...
    elem('H',[15 15],'double'); elem('U_unc',[15 1],'double'); ...
    elem('NmaxNode',1,'uint32'); elem('UoptPrev',[15 1],'double'); ...
    elem('iabcCompensated',[3 1],'double'); ...
    elem('deltaIabcBase',[3 1],'double'); ...
    elem('deltaIabcSwitch',[3 3],'double'); ...
    elem('lambdaZero',1,'double'); elem('lambdaDeadtime',1,'double'); ...
    elem('deadtimeRatio',1,'double'); elem('currentEpsMargin',1,'double')];
set_dictionary_value(section, 'SdaProblemBus', problem);

memory = Simulink.Bus;
memory.Elements = [elem('Uwork',[15 1],'boolean'); ...
    elem('candidatePosition',[15 1],'uint8'); ...
    elem('metricStack',[16 1],'double'); ...
    elem('cmvMetricStack',[16 1],'double'); ...
    elem('iabcPrediction',[3 6],'double'); elem('z',[15 1],'double')];
set_dictionary_value(section, 'SdaMemoryBus', memory);

best = Simulink.Bus;
best.Elements = [elem('bestU',[15 1],'boolean'); ...
    elem('bestMetric',1,'double'); elem('bestCMVCost',1,'double'); ...
    elem('bestValid',1,'boolean')];
set_dictionary_value(section, 'SdaBestBus', best);

result = Simulink.Bus;
result.Elements = [elem('valid',1,'boolean'); elem('Jopt',1,'double'); ...
    elem('nodes',1,'uint32'); elem('optimal',1,'boolean'); ...
    elem('Uopt',[15 1],'double'); elem('Uapply',[6 1],'double'); ...
    elem('leafNodes',1,'uint32'); elem('bestCMVCost',1,'double'); ...
    elem('cmvMetricRatio',1,'double'); ...
    elem('switchCountApplied',1,'uint32'); ...
    elem('debugLower',[3 1],'double'); elem('debugUpper',[3 1],'double'); ...
    elem('iabcPredictedNext',[3 1],'double'); ...
    elem('Jround',1,'double'); elem('Jshift',1,'double'); ...
    elem('predictionValidNext',1,'boolean')];
set_dictionary_value(section, 'SdaResultBus', result);

debug = Simulink.Bus;
debug.Elements = [elem('fsmState',1,'uint8'); elem('level',1,'uint8'); ...
    elem('candidate',1,'boolean'); elem('head',1,'double'); ...
    elem('center',1,'double'); elem('quadraticMetric',1,'double'); ...
    elem('vectorMetric',1,'double'); elem('cycleCount',1,'uint32'); ...
    elem('fault',1,'boolean')];
set_dictionary_value(section, 'SdaDebugBus', debug);
saveChanges(dictionary);
end

function set_dictionary_value(section, name, value)
try
    entry = getEntry(section, name);
    setValue(entry, value);
catch exception
    if ~contains(exception.identifier, 'EntryNotFound') && ...
            ~contains(exception.message, 'not found') && ...
            ~contains(exception.message, 'does not exist')
        rethrow(exception);
    end
    addEntry(section, name, value);
end
end

function element = elem(name, dimensions, dataType)
element = Simulink.BusElement;
element.Name = name;
element.Dimensions = dimensions;
element.DataType = dataType;
end

function clear_subsystem(path)
lines = find_system(path, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
for k = 1:numel(lines)
    delete_line(lines(k));
end
blocks = find_system(path, 'SearchDepth', 1, 'Type', 'Block');
for k = 2:numel(blocks)
    delete_block(blocks{k});
end
end

function text = problem_latch_script()
text = sprintf(['function problem = ProblemLatch(problemIn, loadEnable)\n' ...
    '%%#codegen\n' ...
    '%% Framework latch. Replace with explicit enabled registers for HDL.\n' ...
    'problem = problemIn;\n' ...
    'if ~loadEnable\n' ...
    '    problem = problemIn;\n' ...
    'end\n']);
end

function text = initializer_script()
text = sprintf(['function [done, valid, z, initialU] = SdaInitializer(problem, start)\n' ...
    '%%#codegen\n' ...
    'z = zeros(15,1); initialU = zeros(15,1);\n' ...
    'done = start; valid = true;\n' ...
    'if start\n' ...
    '    z = problem.H * problem.U_unc;\n' ...
    '    initialU = min(max(round(problem.U_unc),0.0),1.0);\n' ...
    'end\n']);
end

function text = quadratic_script()
text = sprintf(['function [done, valid, metric, head, center] = QuadraticCalc(problem, start, level, candidate, memory)\n' ...
    '%%#codegen\n' ...
    'done=start; valid=true; metric=0.0; head=0.0; center=0.0;\n' ...
    'idx=max(uint8(1),min(uint8(15),level));\n' ...
    'for k=1:15\n' ...
    '    if k < double(idx); head=head+problem.H(double(idx),k)*double(memory.Uwork(k)); end\n' ...
    'end\n' ...
    'if abs(problem.H(double(idx),double(idx))) > 1e-12\n' ...
    '    center=(problem.H(double(idx),:)*problem.U_unc-head)/problem.H(double(idx),double(idx));\n' ...
    'else\n' ...
    '    valid=false;\n' ...
    'end\n' ...
    'metric=(double(candidate)-center)^2;\n']);
end

function text = vector_script()
text = sprintf(['function [done, valid, vectorMetric] = VectorCostCalc(problem, start, level, candidate, memory)\n' ...
    '%%#codegen\n' ...
    'done=start; valid=true; vectorMetric=0.0;\n' ...
    'if start\n' ...
    '    vectorMetric=problem.lambdaZero*double(candidate)+0.0*double(level)+0.0*memory.metricStack(1);\n' ...
    'end\n']);
end

function text = branch_memory_script()
text = sprintf(['function memory = BranchMemory(clear, writeEnable, metric, vectorMetric, z)\n' ...
    '%%#codegen\n' ...
    'persistent state\n' ...
    'if isempty(state)\n' ...
    ' state.Uwork=false(15,1); state.candidatePosition=ones(15,1,''uint8'');\n' ...
    ' state.metricStack=zeros(16,1); state.cmvMetricStack=zeros(16,1);\n' ...
    ' state.iabcPrediction=zeros(3,6); state.z=zeros(15,1);\n' ...
    'end\n' ...
    'if clear\n' ...
    ' state.Uwork(:)=false; state.candidatePosition(:)=uint8(1);\n' ...
    ' state.metricStack(:)=0.0; state.cmvMetricStack(:)=0.0;\n' ...
    ' state.iabcPrediction(:)=0.0; state.z=z;\n' ...
    'elseif writeEnable\n' ...
    ' state.metricStack(1)=metric; state.cmvMetricStack(1)=vectorMetric;\n' ...
    'end\n' ...
    'memory=state;\n']);
end

function text = best_memory_script()
text = sprintf(['function best = BestMemory(writeEnable, initialU, metric)\n' ...
    '%%#codegen\n' ...
    'persistent state\n' ...
    'if isempty(state)\n' ...
    ' state.bestU=false(15,1); state.bestMetric=realmax;\n' ...
    ' state.bestCMVCost=0.0; state.bestValid=false;\n' ...
    'end\n' ...
    'if writeEnable\n' ...
    ' state.bestU=initialU>0.5; state.bestMetric=metric; state.bestValid=true;\n' ...
    'end\n' ...
    'best=state;\n']);
end

function text = result_script()
text = sprintf(['function result = ResultCommit(problem, commit, best)\n' ...
    '%%#codegen\n' ...
    'result.valid=commit && best.bestValid; result.Jopt=best.bestMetric;\n' ...
    'result.nodes=uint32(0); result.optimal=best.bestValid;\n' ...
    'result.Uopt=double(best.bestU); result.Uapply=zeros(6,1);\n' ...
    'result.leafNodes=uint32(0); result.bestCMVCost=best.bestCMVCost;\n' ...
    'result.cmvMetricRatio=0.0; result.switchCountApplied=uint32(0);\n' ...
    'result.debugLower=zeros(3,1); result.debugUpper=zeros(3,1);\n' ...
    'result.iabcPredictedNext=problem.iabcCompensated;\n' ...
    'result.Jround=0.0; result.Jshift=0.0;\n' ...
    'result.predictionValidNext=result.valid;\n']);
end
