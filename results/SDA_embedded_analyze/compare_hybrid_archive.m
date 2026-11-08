function [summary,validation,details] = compare_hybrid_archive(hybridFile,baselineFile)
%COMPARE_HYBRID_ARCHIVE Compare deployed hybrid runs with archived baselines.
if nargin<1||isempty(hybridFile), hybridFile=fullfile(fileparts(fileparts(mfilename('fullpath'))),'hybrid_SDA.mldatx'); end
if nargin<2||isempty(baselineFile), baselineFile=fullfile(fileparts(fileparts(mfilename('fullpath'))),'embedded_H.mldatx'); end
hybrid=read_archive(hybridFile); baseline=read_archive(baselineFile); details=struct([]);
torques=[10 50 90]; rows=cell(numel(torques)*3,9); vr=cell(numel(torques),10); rr=0;
for k=1:numel(torques)
    tq=torques(k);
    h=find_run(hybrid,sprintf('hybrid_%dNm',tq));
    x=find_run(baseline,sprintf('original_%dNM',tq));
    e=find_run(baseline,sprintf('embedded_H_%dNM',tq));
    [ihAll,ixAll,ieAll]=align_three(h.time,x.time,e.time,0);
    [ih,ix,ie]=align_three(h.time,x.time,e.time,0.05);
    methods={'Original explicit','Embedded only','Deployed hybrid'};
    groups={double(x.nodes(ix)),double(e.nodes(ie)),double(h.nodes(ih))};
    opts={x.optimal(ix),e.optimal(ie),h.optimal(ih)};
    for m=1:3
        rr=rr+1; a=groups{m};
        rows(rr,:)={tq,methods{m},numel(a),mean(a),median(a),prctile(a,95),max(a),100*mean(opts{m}),100*mean(a>=10000)};
    end
    hn=groups{3}; xn=groups{1}; en=groups{2};
    tol=1e-7*max(1,abs(h.Jopt(ih)));
    gap=h.initialRadius(ih)-h.Jopt(ih);
    details(k).torque=tq; %#ok<AGROW>
    details(k).time=h.time(ih);
    details(k).nodesOriginal=xn;
    details(k).nodesEmbedded=en;
    details(k).nodesHybrid=hn;
    details(k).initialGap=gap;
    details(k).initialOptimal=gap<=tol;
    details(k).timeFull=h.time(ihAll);
    details(k).nodesOriginalFull=double(x.nodes(ixAll));
    details(k).nodesEmbeddedFull=double(e.nodes(ieAll));
    details(k).nodesHybridFull=double(h.nodes(ihAll));
    vr(k,:)={tq,numel(ih),100*(1-mean(hn)/mean(xn)),100*(1-mean(hn)/mean(en)), ...
        100*mean(gap<=tol),median(gap), ...
        100*mean(all(h.Uopt(:,ih)==x.Uopt(:,ix),1)), ...
        100*mean(all(h.Uopt(:,ih)==e.Uopt(:,ie),1)), ...
        max(abs(h.Jopt(ih)-x.Jopt(ix))), ...
        median(e.Jopt(ie)-h.Jopt(ih))};
end
summary=cell2table(rows,'VariableNames',{'TorqueNm','Method','Transactions','MeanNodes','MedianNodes','P95Nodes','MaxNodes','OptimalPercent','NodeLimitPercent'});
validation=cell2table(vr,'VariableNames',{'TorqueNm','PairedTransactions','MeanReductionVsExplicitPercent','MeanReductionVsEmbeddedPercent','InitialOptimalPercent','MedianInitialGap','UoptMatchExplicitPercent','UoptMatchEmbeddedPercent','MaxAbsJoptDifferenceVsExplicit','MedianEmbeddedMinusHybridJopt'});
disp(summary); disp(validation);
end

function runs=read_archive(file)
assert(isfile(file),'Missing archive %s',file); Simulink.sdi.clear; c=onCleanup(@()Simulink.sdi.clear);
Simulink.sdi.load(file); ids=Simulink.sdi.getAllRunIDs; out=cell(1,numel(ids));
for r=1:numel(ids)
    run=Simulink.sdi.getRun(ids(r)); map=containers.Map('KeyType','char','ValueType','any');
    for q=1:run.signalCount, s=run.getSignalByIndex(q); map(char(s.Name))=s.Values; end
    toggle=req(map,'SDA_reference_output.solutionToggle'); tv=logical(toggle.Data(:));
    idx=find([false;tv(2:end)~=tv(1:end-1)]);
    z=struct('name',char(run.Name),'time',toggle.Time(idx).','nodes',uint32(scalar(req(map,'SDA_reference_output.nodes'),idx)), ...
        'optimal',logical(scalar(req(map,'SDA_reference_output.optimal'),idx)), ...
        'Jopt',scalar(req(map,'SDA_reference_output.Jopt'),idx), ...
        'Uopt',ndsample(req(map,'SDA_reference_output.Uopt'),idx), ...
        'initialRadius',optional(map,'SDA_reference_output.initialRadius',idx));
    out{r}=z;
end
runs=[out{:}]; clear c; Simulink.sdi.clear;
end
function run=find_run(runs,name)
i=find(strcmp({runs.name},name),1); assert(~isempty(i),'Missing run %s',name); run=runs(i);
end
function [ia,ib,ic]=align_three(a,b,c,t0)
ka=round(a*1e9); kb=round(b*1e9); kc=round(c*1e9);
[kab,ia,ib]=intersect(ka,kb,'stable'); [~,iab,ic]=intersect(kab,kc,'stable'); ia=ia(iab); ib=ib(iab);
keep=a(ia)>=t0; ia=ia(keep); ib=ib(keep); ic=ic(keep);
end
function s=req(map,name), assert(isKey(map,name),'Missing signal %s',name); s=map(name); end
function v=scalar(s,idx), d=s.Data; v=reshape(d(idx),1,[]); end
function v=optional(map,name,idx), if isKey(map,name), v=scalar(map(name),idx); else, v=nan(1,numel(idx)); end, end
function v=ndsample(s,idx)
d=s.Data;
if ndims(d)==3&&size(d,2)==1, v=reshape(d(:,1,idx),size(d,1),numel(idx));
elseif ismatrix(d), v=d(:,idx); else, sub=repmat({':'},1,ndims(d)); sub{end}=idx; v=d(sub{:}); end
end
