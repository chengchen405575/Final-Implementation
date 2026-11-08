function [summary,detail] = analyze_initializer_candidates(datasetFile,casesPerTorque,outputDirectory)
%ANALYZE_INITIALIZER_CANDIDATES Compare Round, Shift, and Babai incumbents.
% Costs are complete feasible costs in the objective searched by each method.
analysisDir=fileparts(mfilename('fullpath'));
if nargin<1||isempty(datasetFile), datasetFile=fullfile(analysisDir,'data','embedded_H_sda_cases_2025b.mat'); end
if nargin<2||isempty(casesPerTorque), casesPerTorque=500; end
if nargin<3||isempty(outputDirectory), outputDirectory=fullfile(analysisDir,'hybrid_results'); end
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
L=load(datasetFile,'archive'); archive=L.archive;
torques=[10 50 90]; methodNames={'Explicit','Embedded','Hybrid'};
candidateNames={'Round','Shift','Babai'}; rows={}; detail=struct([]); cursor=0;
for t=1:numel(torques)
    tag=sprintf('%dNM',torques(t));
    erun=find_run(archive,"embedded_H_"+tag); xrun=find_run(archive,"original_"+tag);
    valid=find(erun.time>=0.05); pos=unique(round(linspace(1,numel(valid),min(casesPerTorque,numel(valid))))); idx=valid(pos);
    gaps=zeros(3,3,numel(idx)); costs=zeros(3,3,numel(idx));
    for q=1:numel(idx)
        ei=select_case(erun.input,idx(q)); xi=select_case(xrun.input,idx(q));
        eref=select_case(erun.reference,idx(q)); xref=select_case(xrun.reference,idx(q));
        xr=SDA_Solve_Discrete_Profiled(xi); er=SDA_Solve_Embedded_Profiled(ei); hr=SDA_Solve_Hybrid_Profiled(xi,ei);
        costs(:,:,q)=[xr.candidateTotalCosts;er.candidateTotalCosts;hr.candidateTotalCosts];
        optimum=[double(xref.Jopt);double(eref.Jopt);double(xref.Jopt)];
        gaps(:,:,q)=max(0,costs(:,:,q)-optimum);
    end
    cursor=cursor+1; detail(cursor).torque=torques(t); detail(cursor).indices=idx; %#ok<AGROW>
    detail(cursor).costs=costs; detail(cursor).gaps=gaps;
    for m=1:3
        best=min(costs(m,:,:),[],2); scale=max(1,abs(best)); tol=1e-7*scale;
        for c=1:3
            g=reshape(gaps(m,c,:),1,[]); cc=reshape(costs(m,c,:),1,[]);
            isBest=abs(cc-reshape(best,1,[]))<=reshape(tol,1,[]);
            isOptimal=g<=1e-7*max(1,abs(cc));
            rows(end+1,:)={torques(t),methodNames{m},candidateNames{c}, ... %#ok<AGROW>
                median(g),mean(g),100*mean(isBest),100*mean(isOptimal)};
        end
    end
end
summary=cell2table(rows,'VariableNames',{'TorqueNm','SearchMethod','Candidate','MedianGap','MeanGap','BestOrTiedPercent','InitiallyOptimalPercent'});
writetable(summary,fullfile(outputDirectory,'initializer_candidate_summary.csv'));
save(fullfile(outputDirectory,'initializer_candidate_results.mat'),'summary','detail','-v7.3');
make_figures(detail,methodNames,candidateNames,outputDirectory); disp(summary);
end

function make_figures(detail,methodNames,candidateNames,out)
colors=[0.00 0.32 0.65;0.85 0.33 0.10;0.90 0.62 0.05];
f=figure('Color','w','Position',[80 60 1350 900]); tiledlayout(3,3,'Padding','compact','TileSpacing','compact');
for t=1:3
    for m=1:3
        nexttile; hold on; g=squeeze(detail(t).gaps(m,:,:));
        for c=1:3
            x=repmat(c,1,size(g,2)); boxchart(x,log10(1+g(c,:)),'BoxFaceColor',colors(c,:),'MarkerStyle','.');
        end
        xticks(1:3); xticklabels(candidateNames); grid on; box on; ylim([0 5.2]);
        if m==1, title(sprintf('%d Nm',detail(t).torque)); end
        if t==1, ylabel(sprintf('%s\n$\\log_{10}(1+G_0)$',methodNames{m}),'Interpreter','latex');
        else, ylabel(methodNames{m}); end
    end
end
sgtitle('Initial-incumbent gap of Round, Shift, and Babai candidates');
exportgraphics(f,fullfile(out,'initializer_candidate_gap_comparison.pdf'),'ContentType','vector');
exportgraphics(f,fullfile(out,'initializer_candidate_gap_comparison.png'),'Resolution',300); close(f);

f=figure('Color','w','Position',[100 100 1250 430]); tiledlayout(1,3,'Padding','compact','TileSpacing','compact');
for m=1:3
    nexttile; values=zeros(3,3);
    for t=1:3
        c=squeeze(detail(t).costs(m,:,:)); best=min(c,[],1); tol=1e-7*max(1,abs(best));
        for j=1:3, values(t,j)=100*mean(abs(c(j,:)-best)<=tol); end
    end
    bar(categorical({'10 Nm','50 Nm','90 Nm'}),values,'grouped'); ylim([0 105]); grid on;
    ylabel('Best or tied among three candidates (%)'); title(methodNames{m});
    if m==3, legend(candidateNames,'Location','southoutside','Orientation','horizontal'); end
end
sgtitle('Frequency with which each initializer supplies the lowest feasible cost');
exportgraphics(f,fullfile(out,'initializer_candidate_best_frequency.pdf'),'ContentType','vector');
exportgraphics(f,fullfile(out,'initializer_candidate_best_frequency.png'),'Resolution',300); close(f);
end

function run=find_run(container,name)
names=string({container.runs.name}); k=find(names==name,1); assert(~isempty(k),'Missing run %s',name); run=container.runs(k);
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
