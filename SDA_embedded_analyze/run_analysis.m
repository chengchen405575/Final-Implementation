function report = run_analysis(casesPerRun,fullTracePerRun)
%RUN_ANALYSIS Extract the archive if needed, then replay profiled SDA cases.
analysisDir=fileparts(mfilename('fullpath'));
addpath(analysisDir);
datasetFile=fullfile(analysisDir,'data','embedded_H_sda_cases_2025b.mat');
if ~isfile(datasetFile), extract_embedded_H_cases([],datasetFile); end
if nargin<1, casesPerRun=500; end
if nargin<2, fullTracePerRun=10; end
report=analyze_search_intermediates(datasetFile,casesPerRun,fullTracePerRun,[]);
end
