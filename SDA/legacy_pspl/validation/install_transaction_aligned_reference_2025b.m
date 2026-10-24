function report = install_transaction_aligned_reference_2025b()
%INSTALL_TRANSACTION_ALIGNED_REFERENCE_2025B Add a validation-only reference.
% The duplicate reference executes once when PL accepts a request and holds
% its result until the next accepted request.  The production reference and
% the production Uapply path are not changed.

rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
model = 'FCSMPC_double_vector_2025b';
modelFile = fullfile(rootDir,[model '.slx']);
parent = [model '/FCS_MPC/SDA_Solve/SDA Solve'];
pl = [parent '/SDA_PL_Framework'];
reference = [parent '/ SDA_Solve_Synchronized'];
validation = [parent '/Transaction_Aligned_Reference'];

assert(strcmp(version('-release'),'2025b'),'SDA:WrongRelease', ...
    'Run this update only in MATLAB R2025b.');
initialize_base_workspace(rootDir);
addpath(rootDir);
addpath(fullfile(rootDir,'Subsystems_2025b'));
addpath(genpath(fullfile(rootDir,'functions')));
load_system(modelFile);
cleanup = onCleanup(@() close_if_loaded(model)); %#ok<NASGU>

backupDir = fullfile(rootDir,'backups');
if ~isfolder(backupDir), mkdir(backupDir); end
stamp = datestr(now,'yyyymmdd_HHMMSS');
backup = fullfile(backupDir, ...
    ['FCSMPC_double_vector_2025b_before_aligned_reference_' stamp '.slx']);
copyfile(modelFile,backup);

assert(getSimulinkBlockHandle(reference)>0,'SDA:MissingReference', ...
    'Production SDA_Solve_Synchronized was not found.');
assert(getSimulinkBlockHandle(pl)>0,'SDA:MissingPL','PL framework not found.');
assert(getSimulinkBlockHandle([parent '/PL_Start_Gate'])>0, ...
    'SDA:MissingStart','PL validation start gate was not found.');

delete_if_present(validation);
add_block('simulink/Ports & Subsystems/Triggered Subsystem',validation, ...
    'Position',[905 1030 1160 1260]);
delete_if_present([validation '/In1']);
delete_if_present([validation '/Out1']);
trigger = [validation '/Trigger'];
set_param(trigger,'TriggerType','rising');

% Put an independent copy of the known-good solver inside the triggered
% validation wrapper.  It receives exactly the same input values that are
% present when the PL request is accepted.
core = [validation '/ReferenceCore'];
add_block(reference,core,'Position',[250 80 500 390]);
for k = 1:11
    y = 60+30*(k-1);
    add_block('simulink/Ports & Subsystems/In1', ...
        [validation '/problemIn' num2str(k)],'Port',num2str(k), ...
        'Position',[25 y 55 y+14]);
end
for k = 1:16
    y = 45+24*(k-1);
    add_block('simulink/Ports & Subsystems/Out1', ...
        [validation '/resultOut' num2str(k)],'Port',num2str(k), ...
        'Position',[660 y 690 y+14]);
end
corePorts = get_param(core,'PortHandles');
for k = 1:11
    p = get_param([validation '/problemIn' num2str(k)],'PortHandles');
    add_line(validation,p.Outport,corePorts.Inport(k),'autorouting','on');
end
for k = 1:16
    p = get_param([validation '/resultOut' num2str(k)],'PortHandles');
    add_line(validation,corePorts.Outport(k),p.Inport,'autorouting','on');
end

% Branch the same eleven problem signals used by the production reference.
refPorts = get_param(reference,'PortHandles');
valPorts = get_param(validation,'PortHandles');
for k = 1:11
    line = get_param(refPorts.Inport(k),'Line');
    assert(line~=-1,'SDA:UnconnectedReferenceInput', ...
        'Reference input %d is unconnected.',k);
    source = get_param(line,'SrcPortHandle');
    add_line(parent,source,valPorts.Inport(k),'autorouting','on');
end
startPort = get_param([parent '/PL_Start_Gate'],'PortHandles');
add_line(parent,startPort.Outport,valPorts.Trigger,'autorouting','on');

% PL must latch the problem in the same accepted-start tick, rather than a
% later FSM LOAD tick, otherwise the two solvers see adjacent QP samples.
problemLatch = [pl '/ProblemLatch'];
latchPorts = get_param(problemLatch,'PortHandles');
oldLine = get_param(latchPorts.Inport(2),'Line');
if oldLine~=-1, delete_line(oldLine); end
startFrom = get_param([pl '/From_startPulse'],'PortHandles');
add_line(pl,startFrom.Outport,latchPorts.Inport(2),'autorouting','on');

% Route the comparison bus from the held validation reference, leaving the
% production reference outputs and Uapply path untouched.
refBus = [parent '/Reference_Result_Bus'];
refBusPorts = get_param(refBus,'PortHandles');
converterNames = containers.Map([2 8 11], ...
    {'Ref_nodes_uint32','Ref_leafNodes_uint32','Ref_switchCountApplied_uint32'});
for k = 1:16
    oldLine = get_param(refBusPorts.Inport(k),'Line');
    if oldLine~=-1, delete_line(oldLine); end
    if any(k==[2 8 11])
        converter = [parent '/' converterNames(k)];
        cp = get_param(converter,'PortHandles');
        oldLine = get_param(cp.Inport,'Line');
        if oldLine~=-1, delete_line(oldLine); end
        add_line(parent,valPorts.Outport(k),cp.Inport,'autorouting','on');
        add_line(parent,cp.Outport,refBusPorts.Inport(k),'autorouting','on');
    else
        add_line(parent,valPorts.Outport(k),refBusPorts.Inport(k), ...
            'autorouting','on');
    end
end

annotations = find_system(parent,'FindAll','on','Type','annotation');
for k = 1:numel(annotations)
    text = get_param(annotations(k),'PlainText');
    if contains(text,'Reference is sampled when start rises')
        set_param(annotations(k),'Text',sprintf([ ...
            'Validation only:\nReference and PL use the accepted-start input snapshot.\n' ...
            'Reference result is held until PL resultValid.']));
    end
end

set_param(model,'SimulationCommand','update');
save_system(model,modelFile);
report = struct('modelFile',modelFile,'backup',backup, ...
    'validationReference',validation,'generatedAt',datetime('now'));
fprintf('\nTransaction-aligned reference installed.\n');
fprintf('  model  : %s\n',modelFile);
fprintf('  backup : %s\n',backup);
end

function delete_if_present(path)
if getSimulinkBlockHandle(path)>0, delete_block(path); end
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
