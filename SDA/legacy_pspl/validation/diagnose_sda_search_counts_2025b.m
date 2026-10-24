function report = diagnose_sda_search_counts_2025b(stopTime,transactionIndex)
%DIAGNOSE_SDA_SEARCH_COUNTS_2025B Find the first PL/reference search split.
if nargin<1, stopTime=0.48; end
if nargin<2, transactionIndex=2; end
rootDir=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
model='FCSMPC_double_vector_2025b';
parent=[model '/FCS_MPC/SDA_Solve/SDA Solve'];
pl=[parent '/SDA_PL_Framework'];
validation=[parent '/Transaction_Aligned_Reference'];
assert(strcmp(version('-release'),'2025b'),'SDA:WrongRelease','Use R2025b.');
initialize_base_workspace(rootDir);
addpath(rootDir); addpath(fullfile(rootDir,'Subsystems_2025b'));
addpath(genpath(fullfile(rootDir,'functions')));
load_system(model);
cleanup=onCleanup(@() close_if_loaded(model)); %#ok<NASGU>

inputNames={'H','U_unc','NmaxNode','UoptPrev','iabcCompensated', ...
    'deltaIabcBase','deltaIabcSwitch','lambdaZero','lambdaDeadtime', ...
    'deadtimeRatio','currentEpsMargin'};
vp=get_param(validation,'PortHandles');
for k=1:11
    line=get_param(vp.Inport(k),'Line');
    src=get_param(line,'SrcPortHandle');
    mark_port(src,['diag_problem_' inputNames{k}]);
end
fsm=get_param([pl '/SearchFSM'],'PortHandles');
cmp=get_param([pl '/MetricCompare'],'PortHandles');
prune=get_param([pl '/MetricPruneDelay'],'PortHandles');
memoryPorts=get_param([pl '/BranchMemory'],'PortHandles');
vectorPorts=get_param([pl '/VectorCostCalc'],'PortHandles');
mark_port(fsm.Outport(14),'diag_fsm');
mark_port(fsm.Outport(12),'diag_level');
mark_port(fsm.Outport(13),'diag_candidate');
mark_port(cmp.Outport(1),'diag_metric');
mark_port(cmp.Outport(2),'diag_cmv');
mark_port(prune.Outport(1),'diag_prune');
mark_port(memoryPorts.Outport(1),'diag_memory');
mark_port(vectorPorts.Outport(5),'diag_cmv_increment');
mark_port(vectorPorts.Outport(8),'diag_delta_dead');
mark_port(vectorPorts.Outport(9),'diag_lower');
mark_port(vectorPorts.Outport(10),'diag_upper');
mark_port(vectorPorts.Outport(11),'diag_deadtime_miss');

run=sim(model,'StopTime',num2str(stopTime,17),'SignalLogging','on', ...
    'SignalLoggingName','logsout','ReturnWorkspaceOutputs','on');
logs=run.logsout;
startLog=run.get('sda_pl_start_log');
handshake=run.get('sda_pl_handshake_log');
startTime=event_times(startLog.time,startLog.signals.values);
hs=handshake.signals.values;
busyRise=event_times(handshake.time,hs(:,1));
validTime=event_times(handshake.time,hs(:,2));
accepted=zeros(size(busyRise));
for k=1:numel(busyRise)
    q=find(startTime<=busyRise(k)+10*eps(max(1,abs(busyRise(k)))),1,'last');
    if isempty(q), accepted(k)=busyRise(k); else, accepted(k)=startTime(q); end
end
assert(transactionIndex<=min(numel(accepted),numel(validTime)), ...
    'SDA:MissingTransaction','Requested transaction did not complete.');
t0=accepted(transactionIndex); t1=validTime(transactionIndex);
problem=cell(1,11);
for k=1:11
    problem{k}=sample_series(logs.getElement(['diag_problem_' inputNames{k}]).Values,t0);
end
reference=profile_reference(rootDir,problem);

fsmSeries=logs.getElement('diag_fsm').Values;
levelSeries=logs.getElement('diag_level').Values;
candidateSeries=logs.getElement('diag_candidate').Values;
metricSeries=logs.getElement('diag_metric').Values;
cmvSeries=logs.getElement('diag_cmv').Values;
pruneSeries=logs.getElement('diag_prune').Values;
time=fsmSeries.Time(:);
window=time>=t0-10*eps(max(1,abs(t0))) & time<=t1+10*eps(max(1,abs(t1)));
fsmData=scalar_data(fsmSeries); levelData=scalar_data(levelSeries);
candidateData=scalar_data(candidateSeries); metricData=scalar_data(metricSeries);
cmvData=scalar_data(cmvSeries); pruneData=scalar_data(pruneSeries);
fsmU8=uint8(fsmData);
stateEntry=[true; fsmU8(2:end)~=fsmU8(1:end-1)];
compare=window & fsmU8==uint8(13) & stateEntry;
compareTimes=time(compare);
plLevel=uint8(levelData(compare)); plCandidate=logical(candidateData(compare));
plMetric=double(metricData(compare)); plCmv=double(cmvData(compare));
plPrune=logical(pruneData(compare));

count=min(numel(plMetric),double(reference.nodes));
refLevel=uint8(reference.levelTrace(1:count));
refCandidate=logical(reference.candidateTrace(1:count));
refMetric=double(reference.metricTrace(1:count));
refCmv=double(reference.cmvTrace(1:count));
metricDelta=abs(plMetric(1:count)-refMetric);
metricScale=max(1,abs(refMetric));
plIabcBeforeTrace=nan(3,count);
memoryLog=logs.getElement('diag_memory').Values;
for q=1:count
    if mod(double(plLevel(q)),3)==0
        memq=sample_bus(memoryLog,compareTimes(q));
        plIabcBeforeTrace(:,q)=double(memq.iabcPrediction(:,double(plLevel(q))/3));
    end
end
iabcDelta=max(abs(plIabcBeforeTrace-reference.iabcBeforeTrace(:,1:count)),[],1);
firstIabcMismatch=find(isfinite(iabcDelta)&iabcDelta>1e-12,1);
if isempty(firstIabcMismatch), firstIabcMismatch=0; end
mismatch=find(plLevel(1:count)~=refLevel | ...
    plCandidate(1:count)~=refCandidate | metricDelta>1e-13.*metricScale,1);
if isempty(mismatch), mismatch=0; end

report=struct;
report.transactionIndex=transactionIndex; report.startTime=t0; report.validTime=t1;
report.referenceNodes=uint32(reference.nodes); report.plNodes=uint32(numel(plMetric));
report.referenceLeaves=uint32(reference.leafNodes);
nextCandidate=window & fsmU8==uint8(16) & stateEntry;
nextLevel=uint8(levelData(nextCandidate));
nextPrune=logical(pruneData(nextCandidate));
report.plLeaves=uint32(sum(nextLevel==uint8(15)&~nextPrune));
report.firstTraceMismatch=uint32(mismatch);
report.firstIabcMismatch=uint32(firstIabcMismatch);
if firstIabcMismatch>0
    report.firstIabcLevel=plLevel(firstIabcMismatch);
    report.firstIabcPl=plIabcBeforeTrace(:,firstIabcMismatch);
    report.firstIabcRef=reference.iabcBeforeTrace(:,firstIabcMismatch);
end
report.referenceNodeProfile=uint32(reference.nodeProfile(:));
report.plNodeProfile=zeros(15,1,'uint32');
for k=1:15, report.plNodeProfile(k)=uint32(sum(plLevel==uint8(k))); end
report.nodeProfileError=int32(report.plNodeProfile)-int32(report.referenceNodeProfile);
if mismatch>0
    report.firstPlLevel=plLevel(mismatch); report.firstRefLevel=refLevel(mismatch);
    report.firstPlCandidate=plCandidate(mismatch); report.firstRefCandidate=refCandidate(mismatch);
    report.firstPlMetric=plMetric(mismatch); report.firstRefMetric=refMetric(mismatch);
    report.firstMetricDelta=metricDelta(mismatch);
    report.firstPlCmv=plCmv(mismatch); report.firstRefCmv=refCmv(mismatch);
    tm=compareTimes(mismatch);
    memory=sample_bus(logs.getElement('diag_memory').Values,tm);
    level=double(plLevel(mismatch)); step=level/3;
    report.firstPlUCurrent=double(memory.Uwork(level-2:level));
    report.firstPlUPrevious=double(memory.Uwork(level-5:level-3));
    report.firstPlIabcBefore=double(memory.iabcPrediction(:,step));
    report.firstPlCmvIncrement=double(sample_series(logs.getElement('diag_cmv_increment').Values,tm));
    report.firstPlDeltaDead=double(sample_series(logs.getElement('diag_delta_dead').Values,tm));
    report.firstPlLower=double(sample_series(logs.getElement('diag_lower').Values,tm));
    report.firstPlUpper=double(sample_series(logs.getElement('diag_upper').Values,tm));
    report.firstPlDeadtimeMiss=logical(sample_series(logs.getElement('diag_deadtime_miss').Values,tm));
    report.firstRefUCurrent=reference.uCurrentTrace(:,mismatch);
    report.firstRefUPrevious=reference.uPreviousTrace(:,mismatch);
    report.firstRefIabcBefore=reference.iabcBeforeTrace(:,mismatch);
    report.firstRefCmvIncrement=reference.cmvIncrementTrace(mismatch);
    report.firstRefDeltaDead=reference.deltaDeadTrace(:,mismatch);
    report.firstRefLower=reference.lowerTrace(:,mismatch);
    report.firstRefUpper=reference.upperTrace(:,mismatch);
    report.firstRefDeadtimeMiss=reference.deadtimeMissTrace(mismatch);
end

outDir=fullfile(rootDir,'validation_results'); if ~isfolder(outDir), mkdir(outDir); end
stamp=datestr(now,'yyyymmdd_HHMMSS');
matFile=fullfile(outDir,['sda_search_diagnosis_' stamp '.mat']);
txtFile=fullfile(outDir,['sda_search_diagnosis_' stamp '.txt']);
save(matFile,'report','reference','problem','plLevel','plCandidate','plMetric','plCmv','plPrune');
write_report(txtFile,report); report.matFile=matFile; report.textFile=txtFile;
disp(report);
end

function reference=profile_reference(rootDir,problem)
sourceFile=fullfile(rootDir,'functions','Deadtime_CMV_FTE_Double_Vector','SDA_Solve.m');
generatedDir=tempname; mkdir(generatedDir);
cleanupDir=onCleanup(@() rmdir(generatedDir,'s')); %#ok<NASGU>
source=fileread(sourceFile);
source=regexprep(source,'predictionValidNext\]\s*=\s*\.\.\.\s*SDA_Solve\(', ...
    sprintf(['predictionValidNext, nodeProfile, metricTrace, candidateTrace, ' ...
    'levelTrace, cmvTrace, uPreviousTrace, uCurrentTrace, iabcBeforeTrace, ' ...
    'deltaDeadTrace, lowerTrace, upperTrace, cmvIncrementTrace, deadtimeMissTrace] = ...\n' ...
    '    SDA_Solve_profile_generated(']));
source=strrep(source,'nodes = 0;',sprintf(['nodes = 0;\nnodeProfile=zeros(15,1);' ...
    '\nmetricTrace=zeros(10001,1);\ncandidateTrace=false(10001,1);' ...
    '\nlevelTrace=zeros(10001,1);\ncmvTrace=zeros(10001,1);' ...
    '\nuPreviousTrace=zeros(3,10001);\nuCurrentTrace=zeros(3,10001);' ...
    '\niabcBeforeTrace=zeros(3,10001);\ndeltaDeadTrace=zeros(3,10001);' ...
    '\nlowerTrace=zeros(3,10001);\nupperTrace=zeros(3,10001);' ...
    '\ncmvIncrementTrace=zeros(10001,1);\ndeadtimeMissTrace=false(10001,1);']));
source=strrep(source,'nodes = nodes+1;', ...
    sprintf('nodes = nodes+1;\n    nodeProfile(i)=nodeProfile(i)+1;'));
source=strrep(source,'    if metricNew > bestMetric',sprintf([ ...
    '    metricTrace(nodes)=metricNew;\n' ...
    '    candidateTrace(nodes)=logical(candidate);\n' ...
    '    levelTrace(nodes)=i;\n    cmvTrace(nodes)=cmvMetricNew;\n' ...
    '    if mod(i,nu)==0\n' ...
    '        uPreviousTrace(:,nodes)=uPrevious; uCurrentTrace(:,nodes)=uCurrent;\n' ...
    '        iabcBeforeTrace(:,nodes)=iabcBeforeSwitch; deltaDeadTrace(:,nodes)=deltaDead;\n' ...
    '        [~,lowerTrace(:,nodes),upperTrace(:,nodes)]=cmv_cost(uPrevious,uCurrent,iabcBeforeSwitch,deltaDead,lambdaZero,lambdaDeadtime,currentEpsMargin);\n' ...
    '        cmvIncrementTrace(nodes)=cmvIncrement;\n' ...
    '        zeroPart=0.0; if all(uCurrent==0.0)||all(uCurrent==1.0), zeroPart=lambdaZero; end\n' ...
    '        deadtimeMissTrace(nodes)=cmvIncrement>zeroPart;\n' ...
    '    end\n' ...
    '    if metricNew > bestMetric']));
file=fullfile(generatedDir,'SDA_Solve_profile_generated.m');
fid=fopen(file,'w'); assert(fid~=-1); fwrite(fid,source); fclose(fid);
oldPath=path; cleanupPath=onCleanup(@() path(oldPath)); %#ok<NASGU>
addpath(generatedDir,'-begin'); clear SDA_Solve_profile_generated
[reference.Jopt,reference.nodes,reference.optimal,reference.Uopt,reference.Uapply, ...
 reference.leafNodes,reference.bestCMVCost,reference.cmvMetricRatio, ...
 reference.switchCountApplied,reference.debugLower,reference.debugUpper, ...
 reference.iabcPredictedNext,reference.Jround,reference.Jshift, ...
 reference.predictionValidNext,reference.nodeProfile,reference.metricTrace, ...
 reference.candidateTrace,reference.levelTrace,reference.cmvTrace, ...
 reference.uPreviousTrace,reference.uCurrentTrace,reference.iabcBeforeTrace, ...
 reference.deltaDeadTrace,reference.lowerTrace,reference.upperTrace, ...
 reference.cmvIncrementTrace,reference.deadtimeMissTrace]= ...
    SDA_Solve_profile_generated(problem{:});
clear SDA_Solve_profile_generated
end

function mark_port(port,name)
set_param(port,'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName',name);
end
function times=event_times(time,values)
values=logical(values(:)); rising=values & [true;~values(1:end-1)]; times=time(rising);
end
function value=sample_series(series,time)
idx=find(series.Time<=time+10*eps(max(1,abs(time))),1,'last');
sample=getsampleusingtime(series,series.Time(idx)); value=squeeze(sample.Data);
end
function output=sample_bus(bus,time)
names=fieldnames(bus); output=struct;
for k=1:numel(names), output.(names{k})=sample_series(bus.(names{k}),time); end
end
function data=scalar_data(series)
data=squeeze(series.Data); data=data(:);
end
function write_report(file,r)
fid=fopen(file,'w'); cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'transaction=%d start=%.17g valid=%.17g\n',r.transactionIndex,r.startTime,r.validTime);
fprintf(fid,'referenceNodes=%u plNodes=%u referenceLeaves=%u plLeaves=%u\n', ...
    r.referenceNodes,r.plNodes,r.referenceLeaves,r.plLeaves);
fprintf(fid,'firstTraceMismatch=%u\n',r.firstTraceMismatch);
fprintf(fid,'firstIabcMismatch=%u\n',r.firstIabcMismatch);
if r.firstIabcMismatch>0
    fprintf(fid,'firstIabc level=%u ref=%s pl=%s\n',r.firstIabcLevel, ...
        mat2str(r.firstIabcRef.',17),mat2str(r.firstIabcPl.',17));
end
fprintf(fid,'level refNodes plNodes difference\n');
for k=1:15, fprintf(fid,'%d %u %u %d\n',k,r.referenceNodeProfile(k),r.plNodeProfile(k),r.nodeProfileError(k)); end
if r.firstTraceMismatch>0
    fprintf(fid,'first refLevel=%u plLevel=%u refCandidate=%d plCandidate=%d\n', ...
        r.firstRefLevel,r.firstPlLevel,r.firstRefCandidate,r.firstPlCandidate);
    fprintf(fid,'first refMetric=%.17g plMetric=%.17g delta=%.17g\n', ...
        r.firstRefMetric,r.firstPlMetric,r.firstMetricDelta);
    fprintf(fid,'first refCmv=%.17g plCmv=%.17g\n',r.firstRefCmv,r.firstPlCmv);
    fprintf(fid,'uPrevious ref=%s pl=%s\n',mat2str(r.firstRefUPrevious.'),mat2str(r.firstPlUPrevious.'));
    fprintf(fid,'uCurrent ref=%s pl=%s\n',mat2str(r.firstRefUCurrent.'),mat2str(r.firstPlUCurrent.'));
    fprintf(fid,'iabcBefore ref=%s pl=%s\n',mat2str(r.firstRefIabcBefore.',17),mat2str(r.firstPlIabcBefore.',17));
    fprintf(fid,'deltaDead ref=%s pl=%s\n',mat2str(r.firstRefDeltaDead.',17),mat2str(r.firstPlDeltaDead.',17));
    fprintf(fid,'lower ref=%s pl=%s\n',mat2str(r.firstRefLower.',17),mat2str(r.firstPlLower.',17));
    fprintf(fid,'upper ref=%s pl=%s\n',mat2str(r.firstRefUpper.',17),mat2str(r.firstPlUpper.',17));
    fprintf(fid,'cmvIncrement ref=%.17g pl=%.17g deadtimeMiss ref=%d pl=%d\n', ...
        r.firstRefCmvIncrement,r.firstPlCmvIncrement,r.firstRefDeadtimeMiss,r.firstPlDeadtimeMiss);
end
end
function initialize_base_workspace(rootDir)
parameterDir=fullfile(rootDir,'Parameter');
evalin('base',sprintf('addpath(''%s'');',strrep(rootDir,'''','''''')));
evalin('base',sprintf('addpath(genpath(''%s''));',strrep(fullfile(rootDir,'functions'),'''','''''')));
evalin('base','clear FOC Plant FCS FOC_Scaling rules AltPWM; FOC.DataType=''double'';');
scripts={'Testbenchdata','MTPA_MTPV_FFTable','PlantTable','ControllerParam', ...
    'AltPWM','FilterParam','Flags','LutParamCalc','ScalingRules'};
for k=1:numel(scripts), file=fullfile(parameterDir,[scripts{k} '.m']); evalin('base',sprintf('run(''%s'');',strrep(file,'''',''''''))); end
evalin('base','AltPWM=false;'); file=fullfile(parameterDir,'FCS_Parameter.m');
evalin('base',sprintf('run(''%s'');',strrep(file,'''','''''')));
end
function close_if_loaded(model)
if bdIsLoaded(model), close_system(model,0); end
end
