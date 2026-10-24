function report = compare_sda_pl_reference_2025b(stopTime)
%COMPARE_SDA_PL_REFERENCE_2025B Compare transaction-aligned PL/reference data.
if nargin < 1, stopTime = 0.5; end
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
model = 'FCSMPC_double_vector_2025b';
assert(strcmp(version('-release'),'2025b'),'SDA:WrongRelease', ...
    'Run comparison only in MATLAB R2025b.');
initialize_base_workspace(rootDir);
addpath(rootDir);
addpath(fullfile(rootDir,'Subsystems_2025b'));
addpath(genpath(fullfile(rootDir,'functions')));
load_system(model);
cleanup = onCleanup(@() close_if_loaded(model)); %#ok<NASGU>

run = sim(model,'StopTime',num2str(stopTime,17), ...
    'SignalLogging','on','SignalLoggingName','logsout', ...
    'ReturnWorkspaceOutputs','on');
logs = run.logsout;
reference = logs.getElement('sda_reference_result_log').Values;
pl = logs.getElement('sda_pl_result_log').Values;
startLog = run.get('sda_pl_start_log');
handshake = run.get('sda_pl_handshake_log');

startTime = event_times(startLog.time,startLog.signals.values);
hs = handshake.signals.values;
validTime = event_times(handshake.time,hs(:,2));
faultTime = event_times(handshake.time,hs(:,3));
busyRiseTime = event_times(handshake.time,hs(:,1));
% A request is accepted only when the engine enters busy.  Raw start pulses
% that occur while busy must not advance the reference/result pairing.
acceptedStartTime = zeros(size(busyRiseTime));
for k = 1:numel(busyRiseTime)
    index = find(startTime <= busyRiseTime(k) + 10*eps(max(1,abs(busyRiseTime(k)))),1,'last');
    if isempty(index)
        acceptedStartTime(k) = busyRiseTime(k);
    else
        acceptedStartTime(k) = startTime(index);
    end
end
pairCount = min(numel(acceptedStartTime),numel(validTime));

fields = {'Jopt','nodes','optimal','Uopt','Uapply', ...
    'predictionValidNext','iabcPredictedNext','leafNodes', ...
    'bestCMVCost','cmvMetricRatio','switchCountApplied', ...
    'debug_lower','debug_upper','Jround','Jshift','solutionToggle'};
exactFields = {'nodes','optimal','Uopt','Uapply','predictionValidNext', ...
    'leafNodes','switchCountApplied','solutionToggle'};
floatFields = setdiff(fields,exactFields,'stable');

cases = repmat(struct,0,1);
for c = 1:pairCount
    cases(c).startTime = acceptedStartTime(c); %#ok<AGROW>
    cases(c).validTime = validTime(c);
    cases(c).latency = validTime(c)-acceptedStartTime(c);
    % The transaction-aligned validation reference is triggered at the
    % accepted start and holds its result, so sample it when PL completes.
    cases(c).reference = sample_bus(reference,validTime(c));
    cases(c).pl = sample_bus(pl,validTime(c));
    cases(c).errors = struct;
    cases(c).exactPass = true;
    cases(c).floatPass = true;
    for k = 1:numel(exactFields)
        name = exactFields{k};
        a = cases(c).reference.(name);
        b = cases(c).pl.(name);
        equal = isequal(a,b);
        cases(c).errors.(name) = double(~equal);
        cases(c).exactPass = cases(c).exactPass && equal;
    end
    for k = 1:numel(floatFields)
        name = floatFields{k};
        a = double(cases(c).reference.(name));
        b = double(cases(c).pl.(name));
        absError = max(abs(a(:)-b(:)));
        scale = max(1,max(abs(a(:))));
        relError = absError/scale;
        pass = absError <= 1e-12 + 1e-10*scale;
        cases(c).errors.([name '_abs']) = absError;
        cases(c).errors.([name '_rel']) = relError;
        cases(c).floatPass = cases(c).floatPass && pass;
    end
    cases(c).pass = cases(c).exactPass && cases(c).floatPass;
end

report = struct;
report.stopTime = stopTime;
report.startCount = numel(startTime);
report.validCount = numel(validTime);
report.pairCount = pairCount;
report.acceptedStartCount = numel(acceptedStartTime);
report.pendingCount = max(0,numel(acceptedStartTime)-numel(validTime));
report.faultCount = numel(faultTime);
report.busyAtEnd = logical(hs(end,1));
report.cases = cases;
if pairCount > 0
    report.passCount = sum([cases.pass]);
    report.allPass = all([cases.pass]);
    report.maxLatency = max([cases.latency]);
else
    report.passCount = 0;
    report.allPass = false;
    report.maxLatency = NaN;
end

outDir = fullfile(rootDir,'validation_results');
if ~isfolder(outDir), mkdir(outDir); end
stamp = datestr(now,'yyyymmdd_HHMMSS');
matFile = fullfile(outDir,['sda_pl_comparison_' stamp '.mat']);
txtFile = fullfile(outDir,['sda_pl_comparison_' stamp '.txt']);
save(matFile,'report','startTime','acceptedStartTime','busyRiseTime','validTime','faultTime');
write_report(txtFile,report,fields);
report.matFile = matFile;
report.textFile = txtFile;

fprintf('\nSDA PL/reference comparison\n');
fprintf('  stop time : %.9g s\n',stopTime);
fprintf('  starts    : %d\n',report.startCount);
fprintf('  accepted  : %d\n',report.acceptedStartCount);
fprintf('  completed : %d\n',report.validCount);
fprintf('  paired    : %d\n',report.pairCount);
fprintf('  passed    : %d\n',report.passCount);
fprintf('  faults    : %d\n',report.faultCount);
fprintf('  busy end  : %d\n',report.busyAtEnd);
fprintf('  report    : %s\n',txtFile);
end

function times = event_times(time,values)
values = logical(values(:));
rising = values & [true; ~values(1:end-1)];
times = time(rising);
end

function output = sample_bus(bus,time)
names = fieldnames(bus);
output = struct;
for k = 1:numel(names)
    series = bus.(names{k});
    index = find(series.Time <= time + 10*eps(max(1,abs(time))),1,'last');
    if isempty(index), index = 1; end
    sample = getsampleusingtime(series,series.Time(index));
    output.(names{k}) = squeeze(sample.Data);
end
end

function write_report(file,report,fields)
fid = fopen(file,'w');
assert(fid~=-1,'SDA:ReportOpen','Cannot create comparison report.');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'SDA PL/reference comparison\n');
fprintf(fid,'stopTime=%.17g\nstartCount=%d\nvalidCount=%d\n', ...
    report.stopTime,report.startCount,report.validCount);
fprintf(fid,'acceptedStartCount=%d\n',report.acceptedStartCount);
fprintf(fid,'pairCount=%d\npendingCount=%d\nfaultCount=%d\n', ...
    report.pairCount,report.pendingCount,report.faultCount);
fprintf(fid,'passCount=%d\nallPass=%d\nbusyAtEnd=%d\n', ...
    report.passCount,report.allPass,report.busyAtEnd);
for c = 1:report.pairCount
    fprintf(fid,'\nCASE %d start=%.17g valid=%.17g latency=%.17g pass=%d\n', ...
        c,report.cases(c).startTime,report.cases(c).validTime, ...
        report.cases(c).latency,report.cases(c).pass);
    for k = 1:numel(fields)
        name = fields{k};
        if isfield(report.cases(c).errors,name)
            fprintf(fid,'  %s exactMismatch=%g\n',name, ...
                report.cases(c).errors.(name));
        else
            fprintf(fid,'  %s abs=%.17g rel=%.17g\n',name, ...
                report.cases(c).errors.([name '_abs']), ...
                report.cases(c).errors.([name '_rel']));
        end
    end
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
