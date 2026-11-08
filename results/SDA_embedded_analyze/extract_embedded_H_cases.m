function outputFile = extract_embedded_H_cases(mldatxFile,outputFile)
%EXTRACT_EMBEDDED_H_CASES Extract all SDA transactions from all SDI runs.

rootDir = fileparts(fileparts(mfilename('fullpath')));
analysisDir = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(mldatxFile)
    mldatxFile = fullfile(rootDir,'embedded_H.mldatx');
end
dataDir = fullfile(analysisDir,'data');
if ~isfolder(dataDir), mkdir(dataDir); end
if nargin < 2 || isempty(outputFile)
    outputFile = fullfile(dataDir,'embedded_H_sda_cases_2025b.mat');
end
assert(isfile(mldatxFile),'SDA:MissingArchive','Missing %s.',mldatxFile);

Simulink.sdi.clear;
cleanup = onCleanup(@() Simulink.sdi.clear);
Simulink.sdi.load(mldatxFile);
ids = Simulink.sdi.getAllRunIDs;
assert(~isempty(ids),'SDA:NoRuns','No SDI runs found.');

archive = struct;
archive.schemaVersion = uint32(2);
archive.sourceFile = char(mldatxFile);
archive.createdWith = version;
runItems = cell(1,numel(ids));
for r = 1:numel(ids)
    run = Simulink.sdi.getRun(ids(r));
    signals = index_signals(run);
    toggleSeries = require_signal(signals,'SDA_reference_output.solutionToggle');
    toggle = logical(toggleSeries.Data(:));
    index = find([false; toggle(2:end) ~= toggle(1:end-1)]);
    assert(~isempty(index),'SDA:NoTransactions','No transactions in %s.',run.Name);

    item = struct;
    item.name = char(run.Name);
    item.formulation = formulation_from_name(item.name);
    item.time = toggleSeries.Time(index).';
    item.count = uint32(numel(index));
    item.input.H = sample_nd(require_signal(signals,'SDA_input.H'),index);
    item.input.U_unc = sample_nd(require_signal(signals,'SDA_input.U_unc'),index);
    item.input.NmaxNode = uint32(sample_scalar(require_signal(signals,'SDA_input.FCS_NmaxNode'),index));
    item.input.UoptPrev = sample_nd(require_signal(signals,'SDA_input.Uopt_prev'),index);
    item.input.iabcCompensated = sample_components(signals,'SDA_input.iabcReconstruction',3,index);
    item.input.deltaIabcBase = sample_components(signals,'SDA_input.deltaIabcBase',3,index);
    item.input.deltaIabcSwitch = sample_nd(require_signal(signals,'SDA_input.deltaIabcSwitch'),index);
    item.input.lambdaZeroLogged = sample_scalar(require_signal(signals,'SDA_input.signal8'),index);
    item.input.lambdaDeadtime = sample_scalar(require_signal(signals,'SDA_input.signal9'),index);
    item.input.deadtimeRatio = sample_scalar(require_signal(signals,'SDA_input.signal10'),index);
    item.input.currentEpsMargin = sample_scalar(require_signal(signals,'SDA_input.signal11'),index);

    item.reference.Jopt = sample_scalar(require_signal(signals,'SDA_reference_output.Jopt'),index);
    item.reference.nodes = uint32(sample_scalar(require_signal(signals,'SDA_reference_output.nodes'),index));
    item.reference.optimal = logical(sample_scalar(require_signal(signals,'SDA_reference_output.optimal'),index));
    item.reference.Uopt = sample_nd(require_signal(signals,'SDA_reference_output.Uopt'),index);
    item.reference.Uapply = sample_nd(require_signal(signals,'SDA_reference_output.Uapply'),index);
    item.reference.leafNodes = uint32(sample_scalar(require_signal(signals,'SDA_reference_output.leafNodes'),index));
    item.reference.bestCMVCost = sample_scalar(require_signal(signals,'SDA_reference_output.bestCMVCost'),index);
    item.reference.Jround = sample_scalar(require_signal(signals,'SDA_reference_output.Jround'),index);
    item.reference.Jshift = sample_scalar(require_signal(signals,'SDA_reference_output.Jshift'),index);
    item.reference.Jbabai = sample_optional_scalar(signals,'SDA_reference_output.Jbabai',index);
    runItems{r} = item;
    fprintf('%-28s %-8s %7d transactions\n',item.name,item.formulation,double(item.count));
end
archive.runs = [runItems{:}];
save(outputFile,'archive','-v7.3');
fprintf('Saved %s\n',outputFile);
end

function mode = formulation_from_name(name)
if startsWith(string(name),'embedded_H_')
    mode = 'embedded';
elseif contains(string(name),'no_zero_lambda')
    mode = 'discrete_no_zero';
elseif startsWith(string(name),'original_')
    mode = 'discrete';
else
    mode = 'unknown';
end
end

function signals = index_signals(run)
signals = containers.Map('KeyType','char','ValueType','any');
for k=1:run.signalCount
    signal=run.getSignalByIndex(k);
    signals(char(signal.Name))=signal.Values;
end
end
function series=require_signal(signals,name)
assert(isKey(signals,name),'SDA:MissingSignal','Missing signal %s.',name);
series=signals(name);
end
function value=sample_optional_scalar(signals,name,index)
if isKey(signals,name), value=sample_scalar(signals(name),index); else, value=nan(1,numel(index)); end
end
function value=sample_scalar(series,index)
data=series.Data; value=reshape(data(index),1,[]);
end
function value=sample_nd(series,index)
data=series.Data;
if ndims(data)==3 && size(data,2)==1
    value=reshape(data(:,1,index),size(data,1),numel(index));
elseif ndims(data)==3 && size(data,3)==numel(series.Time)
    value=data(:,:,index);
elseif ismatrix(data)
    value=data(:,index);
else
    subs=repmat({':'},1,ndims(data)); subs{end}=index; value=data(subs{:});
end
end
function value=sample_components(signals,prefix,count,index)
value=zeros(count,numel(index));
for c=1:count
    value(c,:)=sample_scalar(require_signal(signals,sprintf('%s(%d,1)',prefix,c)),index);
end
end
