function report = validate_sda_stage5()
%VALIDATE_SDA_STAGE5 Dynamic transaction test of PL framework vs legacy SDA.
% The production model is never saved by this script.  A short closed-loop
% run captures one real SDA problem and its legacy answer, then a temporary
% discrete harness executes the multi-cycle PL framework on frozen inputs.

rootDir = fileparts(mfilename('fullpath'));
model = 'FCSMPC_double_vector';
modelFile = fullfile(rootDir, [model '.slx']);
target = [model '/FCS_MPC/SDA_Solve/SDA Solve/SDA/' ...
    'double_vector_matlab_FSM_Calc/SDA_Solve'];
plSource = [target '/PL_Framework'];
legacy = [target '/Legacy_SDA_Solve'];
captureStop = '8e-5';
maxTicks = 20000;

load_system(modelFile);
modelCleanup = onCleanup(@() close_if_loaded(model)); %#ok<NASGU>

inputNames = {'H','U_unc','NmaxNode','UoptPrev','iabcCompensated', ...
    'deltaIabcBase','deltaIabcSwitch','lambdaZero','lambdaDeadtime', ...
    'deadtimeRatio','currentEpsMargin'};
legacyNames = {'Jopt','nodes','optimal','Uopt','Uapply','leafNodes', ...
    'bestCMVCost','cmvMetricRatio','switchCountApplied','debugLower', ...
    'debugUpper','iabcPredictedNext','Jround','Jshift', ...
    'predictionValidNext'};

% Log existing lines only in memory; the source model is closed unsaved.
for k = 1:numel(inputNames)
    line = input_line(target, k);
    mark_line(line, ['cap_in_' inputNames{k}]);
end
for k = 1:numel(legacyNames)
    line = output_line(legacy, k);
    mark_line(line, ['cap_ref_' legacyNames{k}]);
end
set_param(model, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');
capture = sim(model, 'StopTime', captureStop, 'ReturnWorkspaceOutputs', 'on');
logs = capture.logsout;

problem = cell(1, numel(inputNames));
for k = 1:numel(inputNames)
    problem{k} = last_logged_value(logs, ['cap_in_' inputNames{k}]);
end
referenceLogged = struct;
for k = 1:numel(legacyNames)
    referenceLogged.(legacyNames{k}) = ...
        last_logged_value(logs, ['cap_ref_' legacyNames{k}]);
end
reference = frozen_legacy_reference(rootDir, problem);

% Create an isolated tick-level harness from the exact saved framework.
harness = ['sda_stage5_harness_' char(java.util.UUID.randomUUID)];
harness = strrep(harness, '-', '_');
new_system(harness);
harnessCleanup = onCleanup(@() close_if_loaded(harness)); %#ok<NASGU>
set_param(harness, 'Solver', 'FixedStepDiscrete', 'FixedStep', '1', ...
    'StopTime', num2str(maxTicks), 'DataDictionary', ...
    'FOC_params.sldd', 'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', ...
    'ReturnWorkspaceOutputs', 'on');

pl = [harness '/PL_Framework'];
add_block(plSource, pl, 'Position', [320 80 560 390]);
set_param([pl '/FrameworkStart'], 'Value', 'true');

% Expose validation-only signals on the copied subsystem.  These ports exist
% only in the temporary harness and never alter the production model.
add_block('simulink/Sinks/Out1', [pl '/ValidationResult'], ...
    'Port', '2', 'Position', [1510 560 1580 580]);
add_line(pl, 'ResultCommit/1', 'ValidationResult/1', 'autorouting', 'on');

for k = 1:numel(problem)
    c = [harness '/Problem_' inputNames{k}];
    add_block('simulink/Sources/Constant', c, ...
        'Value', literal(problem{k}), ...
        'Position', [30 30+42*k 210 50+42*k]);
    add_line(harness, ['Problem_' inputNames{k} '/1'], ...
        ['PL_Framework/' num2str(k)], 'autorouting', 'on');
end

% Export result fields and control diagnostics through harness outports.
selector = [harness '/ResultSelector'];
fields = {'valid','Jopt','nodes','optimal','Uopt','Uapply','leafNodes', ...
    'bestCMVCost','cmvMetricRatio','switchCountApplied','debugLower', ...
    'debugUpper','iabcPredictedNext','Jround','Jshift', ...
    'predictionValidNext'};
add_block('simulink/Signal Routing/Bus Selector', selector, ...
    'OutputSignals', strjoin(fields, ','), ...
    'Position', [700 420 710 760]);
add_line(harness, 'PL_Framework/2', 'ResultSelector/1', 'autorouting', 'on');

port = 1;
for k = 1:numel(fields)
    add_out(harness, fields{k}, port, selector, k, 860, 25+38*k);
    port = port + 1;
end
control = { ...
    'commitPulse',[pl '/SearchFSM'],8; ...
    'busy',[pl '/SearchFSM'],9; ...
    'resultValid',[pl '/SearchFSM'],10; ...
    'fault',[pl '/SearchFSM'],11; ...
    'level',[pl '/SearchFSM'],12; ...
    'fsmState',[pl '/SearchFSM'],14; ...
    'cycleCount',[pl '/SearchFSM'],15; ...
    'counterNodes',[pl '/SearchCounter'],1; ...
    'counterLeaves',[pl '/SearchCounter'],2; ...
    'nodeLimitReached',[pl '/SearchCounter'],3; ...
    'candidate',[pl '/SearchFSM'],13; ...
    'currentPrune',[pl '/MetricPruneDelay'],1; ...
    'currentMetric',[pl '/MetricCompare'],1; ...
    'currentCmvMetric',[pl '/MetricCompare'],2};
for k = 1:size(control,1)
    internal = ['Validation_' control{k,1}];
    add_block('simulink/Sinks/Out1', [pl '/' internal], ...
        'Port', num2str(k+2), 'Position', [1510 600+35*k 1580 620+35*k]);
    localSource = strrep(control{k,2}, [pl '/'], '');
    add_line(pl, [localSource '/' num2str(control{k,3})], ...
        [internal '/1'], 'autorouting', 'on');
    add_out(harness, control{k,1}, port, pl, k+2, ...
        1110, 25+38*k);
    port = port + 1;
end

set_param(harness, 'SimulationCommand', 'update');
run = sim(harness, 'ReturnWorkspaceOutputs', 'on');
yout = run.yout;

actual = struct;
trace = struct;
for k = 1:numel(fields)
    actual.(fields{k}) = last_dataset_value(yout, k);
end
for k = 1:size(control,1)
    index = numel(fields)+k;
    actual.(control{k,1}) = last_dataset_value(yout, index);
    trace.(control{k,1}) = dataset_data(yout, index);
end

report = compare_results(reference, actual, maxTicks, trace);
report.captureStop = str2double(captureStop);
report.modelFile = modelFile;
report.generatedAt = datetime('now');

outDir = fullfile(rootDir, 'validation_results');
if ~isfolder(outDir), mkdir(outDir); end
stamp = datestr(now, 'yyyymmdd_HHMMSS');
matFile = fullfile(outDir, ['sda_stage5_' stamp '.mat']);
txtFile = fullfile(outDir, ['sda_stage5_' stamp '.txt']);
save(matFile, 'report', 'reference', 'referenceLogged', 'actual', 'trace', 'problem');
write_report(txtFile, report);
report.matFile = matFile;
report.textFile = txtFile;

fprintf('\nSDA stage-5 dynamic validation\n');
fprintf('  completed : %d\n', report.completed);
fprintf('  fault     : %d\n', report.fault);
fprintf('  pass      : %d\n', report.pass);
fprintf('  cycles    : %u\n', report.cycleCount);
fprintf('  J error   : %.17g\n', report.JError);
fprintf('  U max err : %.17g\n', report.UmaxError);
fprintf('  node diff : %.17g\n', report.nodesError);
fprintf('  report    : %s\n', txtFile);
end

function report = compare_results(ref, act, maxTicks, trace)
report.completed = logical(act.valid) && logical(act.resultValid);
report.fault = logical(act.fault);
report.cycleCount = uint32(act.cycleCount);
report.hitTickLimit = ~report.completed && report.cycleCount >= uint32(maxTicks-2);
report.JError = abs(double(act.Jopt)-double(ref.Jopt));
report.UmaxError = max(abs(double(act.Uopt(:))-double(ref.Uopt(:))));
report.UapplyMaxError = max(abs(double(act.Uapply(:))-double(ref.Uapply(:))));
report.nodesError = double(act.nodes)-double(ref.nodes);
report.leafNodesError = double(act.leafNodes)-double(ref.leafNodes);
report.cmvError = abs(double(act.bestCMVCost)-double(ref.bestCMVCost));
report.optimalMatch = logical(act.optimal)==logical(ref.optimal);
report.nodeProfile = zeros(15,1,'uint32');
fsm = trace.fsmState(:); level = trace.level(:);
for k = 1:15
    report.nodeProfile(k) = uint32(sum(fsm==uint8(7) & level==uint8(k)));
end
if isfield(ref,'nodeProfile')
    report.referenceNodeProfile = uint32(ref.nodeProfile(:));
    report.nodeProfileError = int32(report.nodeProfile)-int32(report.referenceNodeProfile);
end
report.compareCountProfile = zeros(15,1,'uint32');
report.pruneProfile = zeros(15,1,'uint32');
report.metricSumProfile = zeros(15,1);
metric = trace.currentMetric(:); prune = logical(trace.currentPrune(:));
for k = 1:15
    mask = fsm==uint8(13) & level==uint8(k);
    report.compareCountProfile(k) = uint32(sum(mask));
    report.pruneProfile(k) = uint32(sum(mask & prune));
    report.metricSumProfile(k) = sum(metric(mask));
end
if isfield(ref,'metricSumProfile')
    report.referenceMetricSumProfile = ref.metricSumProfile(:);
    report.referencePruneProfile = uint32(ref.pruneProfile(:));
end
compareMask = fsm==uint8(13);
report.plTraceLevel = uint8(level(compareMask));
report.plTraceCandidate = logical(trace.candidate(compareMask));
report.plTraceMetric = metric(compareMask);
report.plTraceCmv = trace.currentCmvMetric(compareMask);
if isfield(ref,'metricTrace')
    count = min(numel(report.plTraceMetric),double(ref.nodes));
    refMetric = ref.metricTrace(1:count);
    refLevel = uint8(ref.levelTrace(1:count));
    refCandidate = logical(ref.candidateTrace(1:count));
    scaleTrace = max(1.0,abs(refMetric));
    mismatch = find(report.plTraceLevel(1:count)~=refLevel | ...
        report.plTraceCandidate(1:count)~=refCandidate | ...
        abs(report.plTraceMetric(1:count)-refMetric)>1e-11.*scaleTrace,1);
    if isempty(mismatch), mismatch=0; end
    report.firstTraceMismatch = uint32(mismatch);
    if mismatch>0
        report.firstPlLevel = report.plTraceLevel(mismatch);
        report.firstRefLevel = refLevel(mismatch);
        report.firstPlCandidate = report.plTraceCandidate(mismatch);
        report.firstRefCandidate = refCandidate(mismatch);
        report.firstPlMetric = report.plTraceMetric(mismatch);
        report.firstRefMetric = refMetric(mismatch);
        report.firstPlCmv = report.plTraceCmv(mismatch);
        report.firstRefCmv = ref.cmvTrace(mismatch);
    end
end
scale = max(1.0, abs(double(ref.Jopt)));
report.metricTolerance = 1e-10*scale;
report.solutionPass = report.completed && ~report.fault && ...
    report.JError <= report.metricTolerance && report.UmaxError == 0 && ...
    report.UapplyMaxError == 0;
report.statisticsPass = report.nodesError==0 && report.leafNodesError==0;
report.tracePass = ~isfield(report,'firstTraceMismatch') || ...
    report.firstTraceMismatch==uint32(0);
report.pass = report.solutionPass && report.statisticsPass && report.tracePass;
end

function ref = frozen_legacy_reference(rootDir, problem)
% Evaluate the exact legacy algorithm on the same frozen problem used by
% the harness.  This removes closed-loop sample alignment from the test.
referenceDir = fullfile(rootDir, 'functions', ...
    'Deadtime_CMV_FTE_Double_Vector');
sourceFile = fullfile(referenceDir,'SDA_Solve.m');
generatedDir = tempname;
mkdir(generatedDir);
generatedCleanup = onCleanup(@() rmdir(generatedDir,'s')); %#ok<NASGU>
source = fileread(sourceFile);
source = regexprep(source, ...
    'predictionValidNext\]\s*=\s*\.\.\.\s*SDA_Solve\(', ...
    sprintf(['predictionValidNext, nodeProfile, metricSumProfile, ' ...
    'pruneProfile, metricTrace, candidateTrace, levelTrace, cmvTrace] = ...\n' ...
    '    SDA_Solve_profile_generated(']));
source = strrep(source, 'nodes = 0;', ...
    sprintf(['nodes = 0;\nnodeProfile = zeros(15,1);\n' ...
    'metricSumProfile = zeros(15,1);\npruneProfile = zeros(15,1);\n' ...
    'metricTrace = zeros(10001,1);\ncandidateTrace = false(10001,1);\n' ...
    'levelTrace = zeros(10001,1);\ncmvTrace = zeros(10001,1);']));
source = strrep(source, 'nodes = nodes+1;', ...
    sprintf('nodes = nodes+1;\n    nodeProfile(i) = nodeProfile(i)+1;'));
source = strrep(source, '    if metricNew > bestMetric', ...
    sprintf(['    metricTrace(nodes) = metricNew;\n' ...
    '    candidateTrace(nodes) = logical(candidate);\n' ...
    '    levelTrace(nodes) = i;\n' ...
    '    cmvTrace(nodes) = cmvMetricNew;\n' ...
    '    metricSumProfile(i) = metricSumProfile(i)+metricNew;\n' ...
    '    if metricNew > bestMetric\n' ...
    '        pruneProfile(i) = pruneProfile(i)+1;']));
generatedFile = fullfile(generatedDir,'SDA_Solve_profile_generated.m');
fid = fopen(generatedFile,'w');
assert(fid~=-1,'Cannot create generated reference profiler.');
fwrite(fid,source);
fclose(fid);
oldPath = path;
cleanup = onCleanup(@() path(oldPath)); %#ok<NASGU>
addpath(generatedDir,'-begin');
clear SDA_Solve_profile_generated
[ref.Jopt,ref.nodes,ref.optimal,ref.Uopt,ref.Uapply, ...
 ref.leafNodes,ref.bestCMVCost,ref.cmvMetricRatio, ...
 ref.switchCountApplied,ref.debugLower,ref.debugUpper, ...
 ref.iabcPredictedNext,ref.Jround,ref.Jshift, ...
 ref.predictionValidNext,ref.nodeProfile,ref.metricSumProfile, ...
 ref.pruneProfile,ref.metricTrace,ref.candidateTrace,ref.levelTrace, ...
 ref.cmvTrace] = ...
    SDA_Solve_profile_generated(problem{:});
clear SDA_Solve_profile_generated
end

function write_report(file, r)
fid = fopen(file, 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'SDA stage-5 dynamic validation\n');
fprintf(fid, 'completed=%d\nfault=%d\npass=%d\n',r.completed,r.fault,r.pass);
fprintf(fid, 'solutionPass=%d\nstatisticsPass=%d\ntracePass=%d\n', ...
    r.solutionPass,r.statisticsPass,r.tracePass);
fprintf(fid, 'cycleCount=%u\nhitTickLimit=%d\n',r.cycleCount,r.hitTickLimit);
fprintf(fid, 'JError=%.17g\nmetricTolerance=%.17g\n',r.JError,r.metricTolerance);
fprintf(fid, 'UmaxError=%.17g\nUapplyMaxError=%.17g\n',r.UmaxError,r.UapplyMaxError);
fprintf(fid, 'nodesError=%.17g\nleafNodesError=%.17g\n',r.nodesError,r.leafNodesError);
fprintf(fid, 'cmvError=%.17g\noptimalMatch=%d\n',r.cmvError,r.optimalMatch);
end

function add_out(model,name,port,source,sourcePort,x,y)
path = [model '/' name];
add_block('simulink/Sinks/Out1', path, 'Port', num2str(port), ...
    'Position', [x y x+70 y+20]);
if contains(source, '/')
    src = [strrep(source,[model '/'],'') '/' num2str(sourcePort)];
else
    src = [source '/' num2str(sourcePort)];
end
add_line(model, src, [name '/1'], 'autorouting', 'on');
end

function value = last_dataset_value(dataset, index)
element = dataset.getElement(index);
value = element.Values.Data;
value = squeeze_last(value);
end

function value = dataset_data(dataset, index)
element = dataset.getElement(index);
value = squeeze(element.Values.Data);
end

function value = last_logged_value(logs, name)
element = logs.getElement(name);
value = element.Values.Data;
value = squeeze_last(value);
end

function value = squeeze_last(data)
sz = size(data);
if isvector(data)
    value = data(end);
elseif numel(sz) >= 2
    index = repmat({':'}, 1, ndims(data));
    index{end} = sz(end);
    value = data(index{:});
else
    value = data;
end
value = squeeze(value);
end

function line = input_line(block, port)
h = get_param(block, 'LineHandles'); line = h.Inport(port);
end

function line = output_line(block, port)
h = get_param(block, 'LineHandles'); line = h.Outport(port);
end

function mark_line(line, name)
port = get_param(line, 'SrcPortHandle');
set_param(port, 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', ...
    'DataLoggingName', name);
end

function text = literal(value)
if isa(value,'uint32')
    text = ['uint32(' mat2str(double(value),17) ')'];
elseif islogical(value)
    text = mat2str(value);
else
    text = mat2str(value,17);
end
end

function close_if_loaded(model)
if bdIsLoaded(model), close_system(model, 0); end
end
