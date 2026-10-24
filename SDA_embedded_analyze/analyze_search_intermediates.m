function report = analyze_search_intermediates(datasetFile,casesPerRun,fullTracePerRun,outputFile)
%ANALYZE_SEARCH_INTERMEDIATES Replay cases and collect search statistics.
analysisDir=fileparts(mfilename('fullpath'));
if nargin<1||isempty(datasetFile), datasetFile=fullfile(analysisDir,'data','embedded_H_sda_cases_2025b.mat'); end
if nargin<2||isempty(casesPerRun), casesPerRun=500; end
if nargin<3||isempty(fullTracePerRun), fullTracePerRun=10; end
if nargin<4||isempty(outputFile), outputFile=fullfile(analysisDir,'data','embedded_H_search_analysis.mat'); end
loaded=load(datasetFile,'archive'); archive=loaded.archive;
report=struct; report.schemaVersion=uint32(1); report.sourceDataset=datasetFile;
report.runs=repmat(struct,1,numel(archive.runs));
for r=1:numel(archive.runs)
    run=archive.runs(r); count=double(run.count);
    selected=unique(round(linspace(1,count,min(casesPerRun,count))));
    tracePositions=unique(round(linspace(1,numel(selected),min(fullTracePerRun,numel(selected)))));
    summary=initialize_summary(numel(selected),15);
    traces=cell(1,numel(tracePositions)); traceCursor=0;
    for q=1:numel(selected)
        c=selected(q); input=select_case(run.input,c);
        if strcmp(run.formulation,'embedded')
            [result,trace]=SDA_Solve_Embedded_Profiled(input);
        elseif strcmp(run.formulation,'discrete_no_zero')
            [result,trace]=SDA_Solve_Profiled_Core(input,0.0,"discrete_no_zero");
        elseif strcmp(run.formulation,'discrete')
            [result,trace]=SDA_Solve_Discrete_Profiled(input);
        else
            error('SDA:UnknownFormulation','Unknown run %s.',run.name);
        end
        reference=select_case(run.reference,c);
        validate_replay(result,reference,run.name,c);
        summary.caseIndex(q)=uint32(c); summary.time(q)=run.time(c);
        summary.Jopt(q)=result.Jopt; summary.nodes(q)=result.nodes;
        summary.leafNodes(q)=result.leafNodes; summary.initialRadius(q)=result.initialRadius;
        summary.initialization(q)=string(result.initialization);
        summary.visitedByDepth(:,q)=trace.visitedByDepth;
        summary.prunedByDepth(:,q)=trace.prunedByDepth;
        summary.survivedByDepth(:,q)=trace.survivedByDepth;
        summary.zeroPrunedByDepth(:,q)=trace.zeroPrunedByDepth;
        for d=1:15
            mask=double(trace.level)==d;
            if any(mask)
                summary.meanMarginByDepth(d,q)=mean(trace.margin(mask));
                summary.medianMarginByDepth(d,q)=median(trace.margin(mask));
            end
        end
        if any(q==tracePositions)
            traceCursor=traceCursor+1;
            traces{traceCursor}=struct('caseIndex',uint32(c),'time',run.time(c), ...
                'result',result,'trace',trace);
        end
    end
    report.runs(r).name=run.name; report.runs(r).formulation=run.formulation;
    report.runs(r).summary=summary; report.runs(r).selectedTraces=traces;
    fprintf('%-28s replayed=%4d mean nodes=%8.2f\n',run.name,numel(selected),mean(double(summary.nodes)));
end
save(outputFile,'report','-v7.3'); fprintf('Saved %s\n',outputFile);
end

function summary=initialize_summary(count,n)
summary=struct('caseIndex',zeros(1,count,'uint32'),'time',zeros(1,count), ...
    'Jopt',zeros(1,count),'nodes',zeros(1,count,'uint32'), ...
    'leafNodes',zeros(1,count,'uint32'),'initialRadius',zeros(1,count), ...
    'initialization',strings(1,count),'visitedByDepth',zeros(n,count,'uint32'), ...
    'prunedByDepth',zeros(n,count,'uint32'),'survivedByDepth',zeros(n,count,'uint32'), ...
    'zeroPrunedByDepth',zeros(n,count,'uint32'), ...
    'meanMarginByDepth',nan(n,count),'medianMarginByDepth',nan(n,count));
end

function item=select_case(group,index)
item=struct; names=fieldnames(group);
for k=1:numel(names)
    value=group.(names{k});
    if ischar(value)||isstring(value), item.(names{k})=value; continue; end
    if isvector(value)
        item.(names{k})=value(index);
    elseif ndims(value)==3
        item.(names{k})=value(:,:,index);
    else
        item.(names{k})=value(:,index);
    end
end
end

function validate_replay(result,reference,runName,index)
scale=max(1,abs(double(reference.Jopt)));
assert(abs(result.Jopt-double(reference.Jopt))<=1e-8*scale, ...
    'SDA:ReplayJopt','Jopt mismatch in %s case %d.',runName,index);
assert(isequal(result.nodes,uint32(reference.nodes)), ...
    'SDA:ReplayNodes','Node mismatch in %s case %d.',runName,index);
assert(isequal(double(result.Uopt(:)),double(reference.Uopt(:))), ...
    'SDA:ReplayUopt','Uopt mismatch in %s case %d.',runName,index);
end
