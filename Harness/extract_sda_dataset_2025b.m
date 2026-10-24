function dataset = extract_sda_dataset_2025b(mldatxFile, outputFile)
%EXTRACT_SDA_DATASET_2025B Convert an SDI run into transaction-aligned SDA cases.
%   The transaction boundary is every change of solutionToggle. Inputs and
%   reference results are sampled at that same logged time.

assert(strcmp(version('-release'),'2025b'), 'SDA:WrongRelease', ...
    'Run this extractor with MATLAB R2025b.');

harnessDir = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(mldatxFile)
    mldatxFile = fullfile(harnessDir,'10Nm.mldatx');
end
if nargin < 2 || isempty(outputFile)
    [~,stem] = fileparts(mldatxFile);
    outputFile = fullfile(harnessDir,['sda_dataset_' stem '_2025b.mat']);
end
assert(isfile(mldatxFile),'SDA:MissingLog','Cannot find %s.',mldatxFile);

Simulink.sdi.clear;
cleanup = onCleanup(@() Simulink.sdi.clear); %#ok<NASGU>
Simulink.sdi.load(mldatxFile);
runIds = Simulink.sdi.getAllRunIDs;
assert(~isempty(runIds),'SDA:NoRuns','No SDI run was found in %s.',mldatxFile);
run = Simulink.sdi.getRun(runIds(end));

signals = index_signals(run);
toggleSeries = require_signal(signals,'SDA_reference_output.solutionToggle');
toggle = logical(toggleSeries.Data(:));
transactionIndex = find([false; toggle(2:end) ~= toggle(1:end-1)]);
assert(~isempty(transactionIndex),'SDA:NoTransactions', ...
    'No solutionToggle transitions were found.');

dataset = struct;
dataset.schemaVersion = uint32(1);
dataset.sourceFile = char(mldatxFile);
dataset.sourceRunName = char(run.Name);
dataset.createdWith = version;
dataset.transactionTime = toggleSeries.Time(transactionIndex).';
dataset.count = uint32(numel(transactionIndex));

% Input names signal8..signal11 reflect unnamed Bus Creator inputs in the
% source model. Their semantic names are fixed here by the SDA port order.
dataset.input.H = sample_nd(require_signal(signals,'SDA_input.H'),transactionIndex);
dataset.input.U_unc = sample_nd(require_signal(signals,'SDA_input.U_unc'),transactionIndex);
dataset.input.NmaxNode = uint32(sample_scalar( ...
    require_signal(signals,'SDA_input.FCS_NmaxNode'),transactionIndex));
dataset.input.UoptPrev = sample_nd(require_signal(signals,'SDA_input.Uopt_prev'),transactionIndex);
dataset.input.iabcCompensated = sample_components(signals, ...
    'SDA_input.iabcReconstruction',3,transactionIndex);
dataset.input.deltaIabcBase = sample_components(signals, ...
    'SDA_input.deltaIabcBase',3,transactionIndex);
dataset.input.deltaIabcSwitch = sample_nd( ...
    require_signal(signals,'SDA_input.deltaIabcSwitch'),transactionIndex);
dataset.input.lambdaZero = sample_scalar(require_signal(signals,'SDA_input.signal8'),transactionIndex);
dataset.input.lambdaDeadtime = sample_scalar(require_signal(signals,'SDA_input.signal9'),transactionIndex);
dataset.input.deadtimeRatio = sample_scalar(require_signal(signals,'SDA_input.signal10'),transactionIndex);
dataset.input.currentEpsMargin = sample_scalar(require_signal(signals,'SDA_input.signal11'),transactionIndex);

dataset.reference.Jopt = sample_scalar(require_signal(signals,'SDA_reference_output.Jopt'),transactionIndex);
dataset.reference.nodes = uint32(sample_scalar(require_signal(signals,'SDA_reference_output.nodes'),transactionIndex));
dataset.reference.optimal = logical(sample_scalar(require_signal(signals,'SDA_reference_output.optimal'),transactionIndex));
dataset.reference.Uopt = sample_nd(require_signal(signals,'SDA_reference_output.Uopt'),transactionIndex);
dataset.reference.Uapply = sample_nd(require_signal(signals,'SDA_reference_output.Uapply'),transactionIndex);
dataset.reference.predictionValidNext = logical(sample_scalar( ...
    require_signal(signals,'SDA_reference_output.predictionValidNext'),transactionIndex));
dataset.reference.iabcPredictedNext = sample_components(signals, ...
    'SDA_reference_output.iabcPredictedNext',3,transactionIndex);
dataset.reference.leafNodes = uint32(sample_scalar( ...
    require_signal(signals,'SDA_reference_output.leafNodes'),transactionIndex));
dataset.reference.bestCMVCost = sample_scalar(require_signal(signals, ...
    'SDA_reference_output.bestCMVCost'),transactionIndex);
dataset.reference.cmvMetricRatio = sample_scalar(require_signal(signals, ...
    'SDA_reference_output.cmvMetricRatio'),transactionIndex);
dataset.reference.switchCountApplied = uint32(sample_scalar(require_signal(signals, ...
    'SDA_reference_output.switchCountApplied'),transactionIndex));
dataset.reference.debug_lower = sample_components(signals, ...
    'SDA_reference_output.debug_lower',3,transactionIndex);
dataset.reference.debug_upper = sample_components(signals, ...
    'SDA_reference_output.debug_upper',3,transactionIndex);
dataset.reference.Jround = sample_scalar(require_signal(signals,'SDA_reference_output.Jround'),transactionIndex);
dataset.reference.Jshift = sample_scalar(require_signal(signals,'SDA_reference_output.Jshift'),transactionIndex);
dataset.reference.solutionToggle = toggle(transactionIndex).';

validate_dataset(dataset);
save(outputFile,'dataset','-v7.3');
fprintf('Extracted %d SDA transactions.\n',double(dataset.count));
fprintf('Saved dataset: %s\n',outputFile);
end

function signals = index_signals(run)
signals = containers.Map('KeyType','char','ValueType','any');
for k = 1:run.signalCount
    signal = run.getSignalByIndex(k);
    signals(char(signal.Name)) = signal.Values;
end
end

function series = require_signal(signals,name)
assert(isKey(signals,name),'SDA:MissingSignal','Required SDI signal is missing: %s',name);
series = signals(name);
end

function value = sample_scalar(series,index)
data = series.Data;
value = reshape(data(index),1,[]);
end

function value = sample_nd(series,index)
data = series.Data;
if ndims(data)==3 && size(data,2)==1
    value = reshape(data(:,1,index),size(data,1),numel(index));
elseif ismatrix(data)
    value = data(:,index);
else
    subs = repmat({':'},1,ndims(data));
    subs{end} = index;
    value = data(subs{:});
end
end

function value = sample_components(signals,prefix,count,index)
value = zeros(count,numel(index));
for component = 1:count
    name = sprintf('%s(%d,1)',prefix,component);
    value(component,:) = sample_scalar(require_signal(signals,name),index);
end
end

function validate_dataset(dataset)
n = double(dataset.count);
assert(numel(dataset.transactionTime)==n,'SDA:BadCount','Transaction time count mismatch.');
groups = {'input','reference'};
for g = 1:numel(groups)
    values = dataset.(groups{g});
    names = fieldnames(values);
    for k = 1:numel(names)
        value = values.(names{k});
        assert(size(value,ndims(value))==n || (isvector(value) && numel(value)==n), ...
            'SDA:BadShape','%s.%s does not contain %d cases.',groups{g},names{k},n);
        if isnumeric(value) && ~isinteger(value)
            assert(all(isfinite(value(:))),'SDA:NonFinite','%s.%s contains NaN/Inf.',groups{g},names{k});
        end
    end
end
end
