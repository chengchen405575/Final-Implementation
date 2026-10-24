function report=route_reference_outputs_to_bus1_2025b()
%ROUTE_REFERENCE_OUTPUTS_TO_BUS1_2025B Route all reference outputs by tags.
% Existing production branches from SDA_Solve_Synchronized are preserved.
rootDir=fileparts(fileparts(fileparts(mfilename('fullpath'))));
model='FCSMPC_double_vector_2025b'; modelFile=fullfile(rootDir,[model '.slx']);
parent=[model '/FCS_MPC/SDA_Solve/SDA Solve'];
reference=[parent '/ SDA_Solve_Synchronized'];
bus=[parent '/Reference_Result_Bus1'];
assert(strcmp(version('-release'),'2025b'),'SDA:WrongRelease','Use R2025b.');
initialize(rootDir);
backupDir=fullfile(rootDir,'backups'); if ~isfolder(backupDir),mkdir(backupDir);end
stamp=datestr(now,'yyyymmdd_HHMMSS');
backup=fullfile(backupDir,['FCSMPC_double_vector_2025b_before_reference_goto_' stamp '.slx']);
copyfile(modelFile,backup);
load_system(modelFile); cleanup=onCleanup(@() close_if_loaded(model)); %#ok<NASGU>
assert(getSimulinkBlockHandle(reference)>0,'SDA:MissingReference','Reference block not found.');
assert(getSimulinkBlockHandle(bus)>0,'SDA:MissingBus','Reference_Result_Bus1 not found.');
set_param(bus,'Inputs','17');

fields={'Jopt','nodes','optimal','Uopt','Uapply','predictionValidNext', ...
    'iabcPredictedNext','leafNodes','bestCMVCost','cmvMetricRatio', ...
    'switchCountApplied','debug_lower','debug_upper','Jround','Jshift', ...
    'Jbabai','solutionToggle'};
converterNames=containers.Map([2 8 11], ...
    {'Ref_nodes_uint1','Ref_leafNodes_uint1','Ref_switchCountApplied_uint1'});
refPorts=get_param(reference,'PortHandles');
busPorts=get_param(bus,'PortHandles');

% Remove incomplete lines entering Bus1 and the three Bus1 converters.
for k=1:17, delete_input_line(busPorts.Inport(k)); end
for k=[2 8 11]
    cp=get_param([parent '/' converterNames(k)],'PortHandles');
    delete_input_line(cp.Inport); delete_output_line(cp.Outport);
end

refPos=get_param(reference,'Position');
busPos=get_param(bus,'Position');
for k=1:17
    field=fields{k}; safe=matlab.lang.makeValidName(field);
    gotoName=['Goto_ref_' safe]; fromName=['From_ref_' safe];
    gotoPath=[parent '/' gotoName]; fromPath=[parent '/' fromName];
    delete_if_present(gotoPath); delete_if_present(fromPath);
    tag=['sda_reference_' safe];
    yg=refPos(2)+26*(k-1);
    yf=busPos(2)+35*(k-1);
    add_block('simulink/Signal Routing/Goto',gotoPath, ...
        'GotoTag',tag,'TagVisibility','local', ...
        'Position',[refPos(3)+90 yg refPos(3)+230 yg+18]);
    add_block('simulink/Signal Routing/From',fromPath, ...
        'GotoTag',tag,'Position',[busPos(1)-320 yf busPos(1)-175 yf+18]);
    gp=get_param(gotoPath,'PortHandles'); fp=get_param(fromPath,'PortHandles');

    % Delete a dangling line only when the reference output has no real
    % destination. Preserve Uopt/Uapply/prediction/solutionToggle branches.
    line=get_param(refPorts.Outport(k),'Line');
    if line~=-1
        dst=get_param(line,'DstPortHandle'); dst=dst(dst~=-1);
        valid=false;
        for q=1:numel(dst)
            try, get_param(dst(q),'Parent'); valid=true; catch, end
        end
        if ~valid, try, delete_line(line); catch, end, end
    end
    add_line(parent,refPorts.Outport(k),gp.Inport,'autorouting','on');
    try, set_param(get_param(gp.Inport,'Line'),'Name',field); catch, end

    if any(k==[2 8 11])
        converter=[parent '/' converterNames(k)]; cp=get_param(converter,'PortHandles');
        add_line(parent,fp.Outport,cp.Inport,'autorouting','on');
        add_line(parent,cp.Outport,busPorts.Inport(k),'autorouting','on');
    else
        add_line(parent,fp.Outport,busPorts.Inport(k),'autorouting','on');
    end
end

set_param(model,'SimulationCommand','update'); save_system(model,modelFile);
report=struct('modelFile',modelFile,'backup',backup,'fields',{fields}, ...
    'generatedAt',datetime('now'));
fprintf('All 17 reference outputs routed through Goto/From to Reference_Result_Bus1.\n');
fprintf('backup: %s\n',backup);
end
function delete_input_line(port)
try, line=get_param(port,'Line'); if line~=-1,delete_line(line);end, catch, end
end
function delete_output_line(port)
try, line=get_param(port,'Line'); if line~=-1,delete_line(line);end, catch, end
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
