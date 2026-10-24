function report = reorganize_sda_pl_routing_2025b()
%REORGANIZE_SDA_PL_ROUTING_2025B Apply the preferred PL diagram routing.
% ResultBusSelector outputs are direct.  SearchFSM/MATLAB Function outputs
% use local Goto/From routing throughout the top level of PL_Framework.

rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
model = 'FCSMPC_double_vector_2025b';
modelFile = fullfile(rootDir,[model '.slx']);
pl = [model '/FCS_MPC/SDA_Solve/SDA Solve/SDA_PL_Framework'];
assert(strcmp(version('-release'),'2025b'),'SDA:WrongRelease', ...
    'Run this routing update only in MATLAB R2025b.');

addpath(rootDir);
addpath(fullfile(rootDir,'Subsystems_2025b'));
addpath(genpath(fullfile(rootDir,'functions')));
initialize_base_workspace(rootDir);

backupDir = fullfile(rootDir,'backups');
if ~isfolder(backupDir), mkdir(backupDir); end
stamp = datestr(now,'yyyymmdd_HHMMSS');
backup = fullfile(backupDir, ...
    ['FCSMPC_double_vector_2025b_before_routing_' stamp '.slx']);
copyfile(modelFile,backup);

load_system(modelFile);
cleanup = onCleanup(@() close_if_loaded(model)); %#ok<NASGU>

make_result_outputs_direct(pl);
route_all_core_outputs(pl);
set_param(model,'SimulationCommand','update');
save_system(model,modelFile);

report = struct('modelFile',modelFile,'backup',backup, ...
    'generatedAt',datetime('now'));
fprintf('SDA PL routing update complete.\n');
fprintf('  model  : %s\n',modelFile);
fprintf('  backup : %s\n',backup);
end

function make_result_outputs_direct(pl)
fields = {'Jopt','nodes','optimal','Uopt','Uapply', ...
    'predictionValidNext','iabcPredictedNext','leafNodes', ...
    'bestCMVCost','cmvMetricRatio','switchCountApplied', ...
    'debugLower','debugUpper','Jround','Jshift','solutionToggle'};
outports = {'Jopt','nodes','optimal','Uopt','Uapply', ...
    'predictionValidNext','iabcPredictedNext','leafNodes', ...
    'bestCMVCost','cmvMetricRatio','switchCountApplied', ...
    'debug_lower','debug_upper','Jround','Jshift','solutionToggle'};
selector = [pl '/ResultBusSelector'];
selectorPorts = get_param(selector,'PortHandles');

for k = 1:numel(fields)
    gotoPath = [pl '/Goto_result_' fields{k}];
    fromPath = [pl '/From_result_' fields{k}];
    outPath = [pl '/' outports{k}];
    outHandle = get_param(outPath,'PortHandles');
    old = get_param(outHandle.Inport,'Line');
    if old~=-1, delete_line(old); end
    if getSimulinkBlockHandle(gotoPath)>0, delete_block(gotoPath); end
    if getSimulinkBlockHandle(fromPath)>0, delete_block(fromPath); end
    pos = get_param(outPath,'Position');
    pos(1) = 1395; pos(3) = 1425;
    set_param(outPath,'Position',pos);
    line = add_line(pl,selectorPorts.Outport(k),outHandle.Inport, ...
        'autorouting','on');
    % Bus Selector output names are inherited automatically.
    try, set_param(line,'Name',outports{k}); catch, end
end
end

function route_all_core_outputs(pl)
core = {'SearchFSM','ProblemLatch','SdaInitializer','QuadraticCalc', ...
    'VectorCostCalc','BranchMemory','BestMemory','MetricCompare', ...
    'SearchCounter'};
for b = 1:numel(core)
    path = [pl '/' core{b}];
    if getSimulinkBlockHandle(path)<=0, continue; end
    ph = get_param(path,'PortHandles');
    for p = 1:numel(ph.Outport)
        route_one_output(pl,core{b},p);
    end
end
end

function route_one_output(system,sourceName,sourcePort)
sourcePath = [system '/' sourceName];
ph = get_param(sourcePath,'PortHandles');
line = get_param(ph.Outport(sourcePort),'Line');
if line==-1, return; end
dst = get_param(line,'DstPortHandle');
dst = dst(dst~=-1);
if isempty(dst), return; end

% If the source already drives a local Goto, reuse that tag and replace
% every remaining direct branch with a matching From block.
gotoIndex = [];
for k = 1:numel(dst)
    if strcmp(get_param(get_param(dst(k),'Parent'),'BlockType'),'Goto')
        gotoIndex = k; break;
    end
end
if ~isempty(gotoIndex)
    gotoBlock = get_param(dst(gotoIndex),'Parent');
    tag = get_param(gotoBlock,'GotoTag');
    direct = dst;
    direct(gotoIndex) = [];
    for k = 1:numel(direct)
        replace_branch_with_from(system,ph.Outport(sourcePort), ...
            direct(k),tag,sourceName,sourcePort,k);
    end
    return;
end

tag = matlab.lang.makeValidName(sprintf('sda_%s_p%d',sourceName,sourcePort));
srcPos = get_param(sourcePath,'Position');
gotoName = unique_name(system,['Goto_' tag]);
add_block('simulink/Signal Routing/Goto',[system '/' gotoName], ...
    'GotoTag',tag,'TagVisibility','local', ...
    'Position',[srcPos(3)+20 srcPos(2) srcPos(3)+135 srcPos(2)+18]);

delete_line(line);
add_line(system,[sourceName '/' num2str(sourcePort)],[gotoName '/1'], ...
    'autorouting','on');
for k = 1:numel(dst)
    add_from_to_destination(system,dst(k),tag,sourceName,sourcePort,k);
end
end

function replace_branch_with_from(system,src,dst,tag,sourceName,sourcePort,index)
try
    delete_line(system,src,dst);
catch
    % A single-destination root line has no branch object. Rebuild it.
    line = get_param(src,'Line');
    if line~=-1, delete_line(line); end
end
add_from_to_destination(system,dst,tag,sourceName,sourcePort,index);
end

function add_from_to_destination(system,dst,tag,sourceName,sourcePort,index)
dstBlock = get_param(dst,'Parent');
dstName = get_param(dstBlock,'Name');
dstPort = get_param(dst,'PortNumber');
dstPos = get_param(dstBlock,'Position');
base = sprintf('From_%s_p%d_%s_%d', ...
    matlab.lang.makeValidName(sourceName),sourcePort, ...
    matlab.lang.makeValidName(dstName),index);
fromName = unique_name(system,base);
add_block('simulink/Signal Routing/From',[system '/' fromName], ...
    'GotoTag',tag,'Position',[max(5,dstPos(1)-140) dstPos(2) ...
    max(100,dstPos(1)-25) dstPos(2)+18]);
add_line(system,[fromName '/1'],[dstName '/' num2str(dstPort)], ...
    'autorouting','on');
end

function name = unique_name(system,base)
name = base; n = 1;
while getSimulinkBlockHandle([system '/' name])>0
    name = [base '_' num2str(n)]; n = n+1;
end
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
