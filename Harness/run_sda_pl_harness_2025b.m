function report = run_sda_pl_harness_2025b(caseLimit)
%RUN_SDA_PL_HARNESS_2025B Replay SDA transactions and print a summary.
assert(strcmp(version('-release'),'2025b'),'SDA:WrongRelease', ...
    'Run this harness only with MATLAB R2025b.');
if nargin < 1, caseLimit = 100; end
harnessDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(harnessDir);
addpath(rootDir,harnessDir,genpath(fullfile(rootDir,'SDA')));
setup_sda_harness_workspace_2025b;
data = evalin('base','sdaHarnessData');
caseLimit = min(double(caseLimit),double(data.count));
assignin('base','sdaHarnessCaseLimit',uint32(caseLimit));

model = 'SDA_PL_Harness_2025b';
load_system(fullfile(harnessDir,[model '.slx']));
cleanup = onCleanup(@() close_if_loaded(model)); %#ok<NASGU>
maximumSimulationTime = max(0.01,caseLimit*1e-3);
out = sim(model,'StopTime',num2str(maximumSimulationTime,17), ...
    'ReturnWorkspaceOutputs','on');
log = out.get('sda_harness_summary_log');
value = squeeze(log.Data(end,:));
report = struct('lastPass',logical(value(1)), ...
    'passCount',uint32(value(2)),'failCount',uint32(value(3)), ...
    'comparedCount',uint32(value(4)),'maxAbsError',value(5), ...
    'maxRelError',value(6),'mismatchMask',uint32(value(7)), ...
    'requestedCount',uint32(caseLimit),'simulationEndTime',out.tout(end));
report.completed = report.comparedCount==report.requestedCount;
report.allPass = report.completed && report.failCount==0;

fprintf('\nSDA PL offline harness\n');
fprintf('  requested : %d\n',report.requestedCount);
fprintf('  compared  : %d\n',report.comparedCount);
fprintf('  passed    : %d\n',report.passCount);
fprintf('  failed    : %d\n',report.failCount);
fprintf('  max abs   : %.17g\n',report.maxAbsError);
fprintf('  max rel   : %.17g\n',report.maxRelError);
fprintf('  all pass  : %d\n',report.allPass);
if ~report.completed
    warning('SDA:HarnessTimeout', ...
        'Harness stopped before all requested transactions completed.');
end
end

function close_if_loaded(model)
if bdIsLoaded(model), close_system(model,0); end
end
