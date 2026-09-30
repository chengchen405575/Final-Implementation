function implement_sda_stage5_fsm_timing()
%IMPLEMENT_SDA_STAGE5_FSM_TIMING Repair request/response and decision timing.

rootDir = fileparts(mfilename('fullpath'));
model = 'FCSMPC_double_vector';
modelFile = fullfile(rootDir, [model '.slx']);
chartPath = [model '/FCS_MPC/SDA_Solve/SDA Solve/SDA/' ...
    'double_vector_matlab_FSM_Calc/SDA_Solve/PL_Framework/SearchFSM'];

backupDir = fullfile(rootDir, 'backups');
if ~isfolder(backupDir), mkdir(backupDir); end
stamp = datestr(now, 'yyyymmdd_HHMMSS');
copyfile(modelFile, fullfile(backupDir, ...
    ['FCSMPC_double_vector_before_stage5_timing_' stamp '.slx']));

load_system(modelFile);
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
sf = sfroot;
chart = sf.find('-isa','Stateflow.Chart','Path',chartPath);

set_state(chart,'METRIC_START', ...
    'METRIC_START\nentry: reset_pulses(); fsmState=uint8(8); quadStart=true;');
set_state(chart,'METRIC_WAIT', ...
    ['METRIC_WAIT\nduring: reset_pulses(); fsmState=uint8(9); ' ...
     'cycleCount=cycleCount+uint32(1);']);
set_transition(chart,'METRIC_WAIT','VECTOR_CHECK','[quadDone && quadValid]');

set_state(chart,'CHECK_LEVEL', ...
    'CHECK_LEVEL\nentry: reset_pulses(); fsmState=uint8(4); cycleCount=cycleCount+uint32(1);');
set_state(chart,'VECTOR_CHECK', ...
    'VECTOR_CHECK\nentry: reset_pulses(); fsmState=uint8(10); cycleCount=cycleCount+uint32(1);');
set_state(chart,'COMPARE', ...
    'COMPARE\nentry: reset_pulses(); fsmState=uint8(13); cycleCount=cycleCount+uint32(1);');
set_state(chart,'NEXT_CANDIDATE', ...
    'NEXT_CANDIDATE\nentry: reset_pulses(); fsmState=uint8(16); cycleCount=cycleCount+uint32(1);');

% The chart cannot call a textual helper from state labels, so expand the
% compact marker into explicit HDL-friendly scalar assignments.
states = chart.find('-isa','Stateflow.State');
reset = ['loadProblem=false; initStart=false; quadStart=false; ' ...
    'vectorStart=false; memClear=false; memWrite=false; bestWrite=false; ' ...
    'commitResult=false; resultValid=false; fault=false;'];
for k = 1:numel(states)
    states(k).LabelString = strrep(states(k).LabelString, 'reset_pulses();', reset);
end

set_param(model,'SimulationCommand','update');
save_system(model,modelFile);
fprintf('SDA stage-5 FSM timing repaired and model updated successfully.\n');
end

function set_state(chart,name,label)
state = chart.find('-isa','Stateflow.State','Name',name);
assert(~isempty(state),'State not found: %s',name);
state(1).LabelString = strrep(label, '\n', sprintf('\n'));
end

function set_transition(chart,source,destination,label)
transitions = chart.find('-isa','Stateflow.Transition');
for k = 1:numel(transitions)
    if ~isempty(transitions(k).Source) && ~isempty(transitions(k).Destination) && ...
            strcmp(transitions(k).Source.Name,source) && ...
            strcmp(transitions(k).Destination.Name,destination)
        transitions(k).LabelString = label;
        return;
    end
end
error('Transition not found: %s -> %s',source,destination);
end
