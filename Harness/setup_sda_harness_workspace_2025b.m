function setup_sda_harness_workspace_2025b(datasetFile)
%SETUP_SDA_HARNESS_WORKSPACE_2025B Load playback data and bus definitions.
assert(strcmp(version('-release'),'2025b'),'SDA:WrongRelease', ...
    'Run the harness with MATLAB R2025b.');
harnessDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(harnessDir);
if nargin < 1 || isempty(datasetFile)
    datasetFile = fullfile(harnessDir,'sda_dataset_10Nm_2025b.mat');
end
assert(isfile(datasetFile),'SDA:MissingDataset', ...
    'Dataset missing. Run extract_sda_dataset_2025b first: %s',datasetFile);
loaded = load(datasetFile,'dataset');
assignin('base','sdaHarnessData',loaded.dataset);
if evalin('base',"~exist('sdaHarnessCaseLimit','var')")
    assignin('base','sdaHarnessCaseLimit',uint32(min(100,double(loaded.dataset.count))));
end
assignin('base','sdaHarnessAbsTol',1e-12);
assignin('base','sdaHarnessRelTol',1e-10);
addpath(rootDir,genpath(fullfile(rootDir,'SDA')));
dictionaryFile = fullfile(rootDir,'FOC_params.sldd');
if isfile(dictionaryFile)
    defineSdaBuses(dictionaryFile);
end
end
