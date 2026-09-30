function implement_sda_stage6_trace_cleanup()
%IMPLEMENT_SDA_STAGE6_TRACE_CLEANUP Align leaf semantics and bus names.

rootDir = fileparts(mfilename('fullpath'));
model = 'FCSMPC_double_vector';
modelFile = fullfile(rootDir, [model '.slx']);
base = [model '/FCS_MPC/SDA_Solve/SDA Solve/SDA/' ...
    'double_vector_matlab_FSM_Calc/SDA_Solve/PL_Framework'];

backupDir = fullfile(rootDir, 'backups');
if ~isfolder(backupDir), mkdir(backupDir); end
stamp = datestr(now, 'yyyymmdd_HHMMSS');
copyfile(modelFile, fullfile(backupDir, ...
    ['FCSMPC_double_vector_before_stage6_' stamp '.slx']));

load_system(modelFile);
cleanup = onCleanup(@() close_system(model,0)); %#ok<NASGU>
sf = sfroot;

counterPath = [base '/SearchCounter'];
counter = sf.find('-isa','Stateflow.EMChart','Path',counterPath);
counter.Script = counter_code();
set_data(counter,'currentPrune','boolean','[1 1]');

% Sixth input follows the function signature and receives the registered
% decision used by Stateflow in the same COMPARE transaction.
ports = get_param(counterPath,'PortHandles');
line = get_param(ports.Inport(6),'Line');
if line ~= -1, delete_line(line); end
% Leaf accounting occurs in NEXT_CANDIDATE, after the COMPARE decision.
% At that point the registered prune value is aligned with the completed
% candidate and is stable for both pruned and accepted leaf paths.
src = get_param([base '/MetricPruneDelay'],'PortHandles');
ports = get_param(counterPath,'PortHandles');
add_line(base,src.Outport(1),ports.Inport(6),'autorouting','on');

% QuadraticCalc already computes center, selected candidate, residual, and
% metric in CENTER_START/CENTER_WAIT.  Re-running it after SELECT observes
% the incremented candidatePosition and evaluates the opposite candidate.
% Bypass that redundant request and consume the latched result directly.
chart = sf.find('-isa','Stateflow.Chart','Path',[base '/SearchFSM']);
retarget(chart,'SELECT_CANDIDATE','METRIC_START','VECTOR_CHECK');
delete_state_if_present(chart,'METRIC_START');
delete_state_if_present(chart,'METRIC_WAIT');
delete_dangling_transitions(chart);

% Accept each vector request on its rising edge.  Stateflow entry outputs can
% be observed for two Simulink evaluations around the transition to WAIT;
% without this guard a nonzero CMV increment is accumulated twice.
vector = sf.find('-isa','Stateflow.EMChart','Path',[base '/VectorCostCalc']);
add_start_edge_guard(vector);

% The double golden model uses the same floating-point evaluation order as
% the legacy solver.  A hand-written dot/sum loop differed by one ULP at the
% pruning boundary and changed leafNodes while leaving the optimum intact.
initializer = sf.find('-isa','Stateflow.EMChart','Path',[base '/SdaInitializer']);
align_initializer_double_math(initializer);

problemNames = {'H','U_unc','NmaxNode','UoptPrev','iabcCompensated', ...
    'deltaIabcBase','deltaIabcSwitch','lambdaZero','lambdaDeadtime', ...
    'deadtimeRatio','currentEpsMargin'};
name_bus_inputs([base '/ProblemBusCreator'],problemNames);

debugNames = {'fsmState','level','candidate','head','center', ...
    'quadraticMetric','vectorMetric','cycleCount','fault'};
name_bus_inputs([base '/DebugBusCreator'],debugNames);

set_param(model,'SimulationCommand','update');
save_system(model,modelFile);
fprintf('SDA stage-6 leaf counting and bus naming updated successfully.\n');
end

function code = counter_code()
code = sprintf([ ...
'function [nodes,leafNodes,nodeLimitReached] = SearchCounter(clear,writeEnable,fsmState,level,problem,currentPrune)\n' ...
'%%#codegen\n' ...
'persistent nodeReg leafReg limitReg leafStatePrev\n' ...
'if isempty(nodeReg), nodeReg=uint32(0); leafReg=uint32(0); limitReg=false; leafStatePrev=false; end\n' ...
'if clear, nodeReg=uint32(0); leafReg=uint32(0); limitReg=false; leafStatePrev=false; end\n' ...
'if writeEnable&&fsmState==uint8(7)\n' ...
' nodeReg=nodeReg+uint32(1);\n' ...
' if problem.NmaxNode>uint32(0)&&nodeReg>problem.NmaxNode, limitReg=true; end\n' ...
'end\n' ...
'leafStateNow=fsmState==uint8(16);\n' ...
'if leafStateNow&&~leafStatePrev&&level==uint8(15)&&~currentPrune\n' ...
' leafReg=leafReg+uint32(1);\n' ...
'end\n' ...
'leafStatePrev=leafStateNow;\n' ...
'nodes=nodeReg; leafNodes=leafReg; nodeLimitReached=limitReg;\n' ...
'end\n']);
end

function name_bus_inputs(block,names)
handles = get_param(block,'LineHandles');
for k = 1:numel(names)
    if handles.Inport(k) ~= -1
        set_param(handles.Inport(k),'Name',names{k});
    end
end
end

function set_data(chart,name,type,sizeText)
data = chart.find('-isa','Stateflow.Data','Name',name);
if isempty(data)
    data = Stateflow.Data(chart);
    data.Name = name;
    data.Scope = 'Input';
    inputs = chart.find('-isa','Stateflow.Data','Scope','Input');
    used = arrayfun(@(x) x.Port,inputs);
    data.Port = max([0 used])+1;
end
data.DataType = type;
data.Props.Array.Size = sizeText;
end

function retarget(chart,source,oldDestination,newDestination)
transitions = chart.find('-isa','Stateflow.Transition');
destination = chart.find('-isa','Stateflow.State','Name',newDestination);
assert(~isempty(destination),'Destination state not found: %s',newDestination);
for k = 1:numel(transitions)
    if ~isempty(transitions(k).Source) && ~isempty(transitions(k).Destination) && ...
            strcmp(transitions(k).Source.Name,source) && ...
            strcmp(transitions(k).Destination.Name,oldDestination)
        transitions(k).Destination = destination(1);
        return;
    end
end
% Idempotent re-run: accept an already-retargeted transition.
for k = 1:numel(transitions)
    if ~isempty(transitions(k).Source) && ~isempty(transitions(k).Destination) && ...
            strcmp(transitions(k).Source.Name,source) && ...
            strcmp(transitions(k).Destination.Name,newDestination)
        return;
    end
end
error('Transition not found: %s -> %s',source,oldDestination);
end

function delete_state_if_present(chart,name)
state = chart.find('-isa','Stateflow.State','Name',name);
if ~isempty(state), delete(state); end
end

function delete_dangling_transitions(chart)
transitions = chart.find('-isa','Stateflow.Transition');
for k = 1:numel(transitions)
    % A legitimate default transition has no source but has a destination.
    % Deleted states can leave transitions with no destination; remove them.
    if isempty(transitions(k).Destination) || ...
            (isempty(transitions(k).Source) && ...
             ~isempty(strtrim(transitions(k).LabelString)))
        delete(transitions(k));
    end
end
end

function add_start_edge_guard(chart)
script = chart.Script;
if ~contains(script,'startPrev')
    script = strrep(script, ...
        'persistent totalReg cmvTotalReg incrementReg iabcReg stepReg deadReg lowerReg upperReg missReg validReg', ...
        ['persistent totalReg cmvTotalReg incrementReg iabcReg stepReg ' ...
         'deadReg lowerReg upperReg missReg validReg startPrev']);
    script = strrep(script, ...
        '    missReg=false; validReg=false;', ...
        '    missReg=false; validReg=false; startPrev=false;');
    script = strrep(script, 'if start', 'if start&&~startPrev');
    script = strrep(script, ...
        'valid=validReg; totalMetricNew=totalReg;', ...
        sprintf('startPrev=start;\nvalid=validReg; totalMetricNew=totalReg;'));
end
if ~contains(script,'doneReg')
    script = strrep(script,'validReg startPrev','validReg startPrev doneReg');
    script = strrep(script,'startPrev=false;','startPrev=false; doneReg=false;');
    script = strrep(script,'done=false;','done=doneReg;');
    script = strrep(script,'    done=true;','    doneReg=true;');
    script = strrep(script,'startPrev=start;', ...
        sprintf(['if ~start&&~startPrev, doneReg=false; end\n' ...
        'startPrev=start;']));
end
chart.Script = script;
end

function align_initializer_double_math(chart)
script = chart.Script;
pattern = ['function value=quadratic_cost\(H,U,Uunc\)' ...
    '[\s\S]*?end\s*\n\s*\nfunction total='];
replacement = sprintf([ ...
    'function value=quadratic_cost(H,U,Uunc)\n' ...
    'residual=H*(U-Uunc);\n' ...
    'value=norm(residual,2)^2;\n' ...
    'end\n\nfunction total=']);
updated = regexprep(script,pattern,replacement,'once');
assert(~strcmp(updated,script) || contains(script,'value=norm(residual,2)^2;'), ...
    'Could not locate SdaInitializer quadratic_cost helper.');
chart.Script = updated;
end
