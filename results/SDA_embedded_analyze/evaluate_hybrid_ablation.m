function study = evaluate_hybrid_ablation(datasetFile,casesPerTorque,outputDirectory)
%EVALUATE_HYBRID_ABLATION Compare native, embedded, hybrid, and oracle search.
analysisDir=fileparts(mfilename('fullpath'));
if nargin<1||isempty(datasetFile), datasetFile=fullfile(analysisDir,'data','embedded_H_sda_cases_2025b.mat'); end
if nargin<2||isempty(casesPerTorque), casesPerTorque=500; end
if nargin<3||isempty(outputDirectory), outputDirectory=fullfile(analysisDir,'hybrid_results'); end
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
L=load(datasetFile,'archive'); archive=L.archive; torques=["10NM","50NM","90NM"];
study=struct; rows=[];
for t=1:3
    erun=find_run(archive,"embedded_H_"+torques(t)); xrun=find_run(archive,"original_"+torques(t));
    valid=find(erun.time>=0.05); positions=unique(round(linspace(1,numel(valid),min(casesPerTorque,numel(valid))))); indices=valid(positions);
    data=initialize_data(numel(indices));
    for q=1:numel(indices)
        index=indices(q); ei=select_case(erun.input,index); xi=select_case(xrun.input,index);
        eref=select_case(erun.reference,index); xref=select_case(xrun.reference,index);
        [xr,xt]=SDA_Solve_Discrete_Profiled(xi);
        [er,et]=SDA_Solve_Embedded_Profiled(ei);
        [hr,ht]=SDA_Solve_Hybrid_Profiled(xi,ei);
        [xor,xot]=SDA_Solve_Oracle_Profiled(xi,"discrete",xref.Uopt);
        [eor,eot]=SDA_Solve_Oracle_Profiled(ei,"embedded",eref.Uopt);
        assert_solution(xr,xref,"native explicit",torques(t),index);
        assert_solution(er,eref,"native embedded",torques(t),index);
        assert_solution(hr,xref,"hybrid",torques(t),index);
        assert_solution(xor,xref,"oracle explicit",torques(t),index);
        assert_solution(eor,eref,"oracle embedded",torques(t),index);
        resultList={xr,er,hr,xor,eor}; traceList={xt,et,ht,xot,eot}; refList={xref,eref,xref,xref,eref};
        for a=1:5
            r=resultList{a}; tr=traceList{a}; ref=refList{a}; tol=1e-7*max(1,abs(double(ref.Jopt)));
            data.nodes(a,q)=double(r.nodes); data.initialGap(a,q)=max(0,r.initialRadius-double(ref.Jopt));
            if data.initialGap(a,q)<=tol
                first=0;
            else
                hit=find(abs(tr.radiusAfter-double(ref.Jopt))<=tol,1,'first');
                if isempty(hit), first=double(r.nodes); else, first=hit; end
            end
            data.discoveryNodes(a,q)=first; data.proofNodes(a,q)=double(r.nodes)-first;
            data.initialOptimal(a,q)=data.initialGap(a,q)<=tol;
        end
    end
    study(t).torque=char(torques(t)); study(t).caseIndex=indices; study(t).data=data;
    for a=1:5
        rows=[rows; {char(torques(t)),data.names{a},mean(data.nodes(a,:)), ...
            median(data.nodes(a,:)),prctile(data.nodes(a,:),95), ...
            median(data.initialGap(a,:)),100*mean(data.initialOptimal(a,:)), ...
            mean(data.discoveryNodes(a,:)),mean(data.proofNodes(a,:))}]; %#ok<AGROW>
    end
end
summary=cell2table(rows,'VariableNames',{'Torque','Method','MeanNodes','MedianNodes','Q95Nodes', ...
    'MedianInitialGap','InitialOptimalPercent','MeanDiscoveryNodes','MeanProofNodes'});
study(1).summary=summary; save(fullfile(outputDirectory,'hybrid_ablation_results.mat'),'study','summary','-v7.3');
writetable(summary,fullfile(outputDirectory,'hybrid_ablation_summary.csv')); disp(summary);
make_figures(study,outputDirectory);
end

function data=initialize_data(count)
data.names={'Native explicit','Native embedded','Hybrid','Oracle explicit','Oracle embedded'};
data.nodes=zeros(5,count); data.initialGap=zeros(5,count); data.initialOptimal=false(5,count);
data.discoveryNodes=zeros(5,count); data.proofNodes=zeros(5,count);
end
function make_figures(study,out)
for t=1:3
    d=study(t).data; names=d.names;
    figure('Color','w','Position',[80 80 1250 790]); tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
    groups=categorical(repmat(names,1,size(d.nodes,2)),names,names);
    nexttile; boxchart(groups,d.nodes(:)); ylabel('Visited nodes'); title('Total search effort'); grid on;
    nexttile; boxchart(groups,log10(1+d.initialGap(:))); ylabel('$\log_{10}(1+R_0-J^\star)$','Interpreter','latex'); title('Initial upper-bound gap'); grid on;
    nexttile; bar(categorical(names),mean(d.discoveryNodes,2)); ylabel('Mean nodes'); title('Incumbent discovery stage'); grid on;
    nexttile; bar(categorical(names),mean(d.proofNodes,2)); ylabel('Mean nodes'); title('Optimality proof stage'); grid on;
    sgtitle(study(t).torque+" hybrid-SDA ablation");
    exportgraphics(gcf,fullfile(out,"hybrid_ablation_"+study(t).torque+".png"),'Resolution',300); close(gcf);
end
end
function assert_solution(result,reference,label,torque,index)
tol=1e-7*max(1,abs(double(reference.Jopt)));
assert(abs(result.Jopt-double(reference.Jopt))<=tol && isequal(double(result.Uopt(:)),double(reference.Uopt(:))), ...
    'SDA:HybridMismatch','%s mismatch at %s case %d.',label,torque,index);
end
function run=find_run(container,name)
names=string({container.runs.name}); index=find(names==name,1); assert(~isempty(index)); run=container.runs(index);
end
function item=select_case(group,index)
item=struct; names=fieldnames(group);
for k=1:numel(names)
    value=group.(names{k});
    if isvector(value), item.(names{k})=value(index);
    elseif ndims(value)==3, item.(names{k})=value(:,:,index);
    else, item.(names{k})=value(:,index); end
end
end
