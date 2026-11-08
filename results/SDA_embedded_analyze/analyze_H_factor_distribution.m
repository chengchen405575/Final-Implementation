function result = analyze_H_factor_distribution(datasetFile,ablationFile,outputDirectory)
%ANALYZE_H_FACTOR_DISTRIBUTION Relate Cholesky structure to oracle node count.
analysisDir=fileparts(mfilename('fullpath'));
if nargin<1||isempty(datasetFile), datasetFile=fullfile(analysisDir,'data','embedded_H_sda_cases_2025b.mat'); end
if nargin<2||isempty(ablationFile), ablationFile=fullfile(analysisDir,'hybrid_results','hybrid_ablation_results.mat'); end
if nargin<3||isempty(outputDirectory), outputDirectory=fullfile(analysisDir,'hybrid_results'); end
A=load(datasetFile,'archive'); archive=A.archive;
B=load(ablationFile,'study'); study=B.study;
n=15; rowE=[]; rowX=[]; diagE=[]; diagX=[]; kappaE=[]; kappaX=[]; deltaNodes=[]; torque=[];
for t=1:numel(study)
    erun=find_run(archive,"embedded_H_"+string(study(t).torque));
    xrun=find_run(archive,"original_"+string(study(t).torque));
    indices=study(t).caseIndex;
    for q=1:numel(indices)
        He=double(erun.input.H(:,:,indices(q))); Hx=double(xrun.input.H(:,:,indices(q)));
        [re,de,ke]=features(He); [rx,dx,kx]=features(Hx);
        rowE(:,end+1)=re; rowX(:,end+1)=rx; diagE(:,end+1)=de; diagX(:,end+1)=dx; %#ok<AGROW>
        kappaE(:,end+1)=ke; kappaX(:,end+1)=kx; %#ok<AGROW>
        deltaNodes(end+1)=study(t).data.nodes(5,q)-study(t).data.nodes(4,q); %#ok<AGROW>
        torque(end+1)=t; %#ok<AGROW>
    end
end
rhoDiag=zeros(n,1); pDiag=zeros(n,1); rhoKappa=zeros(n,1); pKappa=zeros(n,1); rhoRow=zeros(n,1); pRow=zeros(n,1);
for i=1:n
    [rhoDiag(i),pDiag(i)]=corr((diagE(i,:)-diagX(i,:)).',deltaNodes.','Type','Spearman','Rows','complete');
    [rhoKappa(i),pKappa(i)]=corr((kappaE(i,:)-kappaX(i,:)).',deltaNodes.','Type','Spearman','Rows','complete');
    [rhoRow(i),pRow(i)]=corr((rowE(i,:)-rowX(i,:)).',deltaNodes.','Type','Spearman','Rows','complete');
end
result=struct('rowEmbedded',rowE,'rowExplicit',rowX,'diagEmbedded',diagE,'diagExplicit',diagX, ...
    'kappaEmbedded',kappaE,'kappaExplicit',kappaX,'oracleNodeDifference',deltaNodes, ...
    'torqueIndex',torque,'rhoDiag',rhoDiag,'pDiag',pDiag,'rhoKappa',rhoKappa,'pKappa',pKappa, ...
    'rhoRow',rhoRow,'pRow',pRow);
save(fullfile(outputDirectory,'H_factor_distribution_results.mat'),'result','-v7.3');
summary=table((1:n).',rhoRow,pRow,rhoDiag,pDiag,rhoKappa,pKappa, ...
    'VariableNames',{'Depth','RhoRowNorm','PRowNorm','RhoDiagonal','PDiagonal','RhoCoupling','PCoupling'});
writetable(summary,fullfile(outputDirectory,'H_factor_correlation_by_depth.csv'));
disp(summary);
make_figure(result,outputDirectory);
end

function [rowNorm,diagonal,kappa]=features(H)
n=size(H,1); rowNorm=zeros(n,1); diagonal=abs(diag(H)); kappa=zeros(n,1);
for i=1:n
    rowNorm(i)=norm(H(i,1:i),2);
    if i>1, kappa(i)=norm(H(i,1:i-1),2)/max(diagonal(i),eps); end
end
end
function make_figure(r,out)
d=(1:15).'; figure('Color','w','Position',[70 70 1260 820]); tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
nexttile; plot_band(d,r.rowExplicit,r.rowEmbedded,'Row norm $\|H(i,1{:}i)\|_2$');
nexttile; plot_band(d,r.diagExplicit,r.diagEmbedded,'Diagonal magnitude $|H_{ii}|$');
nexttile; plot_band(d,r.kappaExplicit,r.kappaEmbedded,'Coupling ratio $\kappa_i$');
nexttile; plot(d,r.rhoRow,'-^',d,r.rhoDiag,'-o',d,r.rhoKappa,'-s','LineWidth',1.4); hold on; yline(0,'k--');
grid on; ylim([-1 1]); xlabel('SDA depth'); ylabel('Spearman $\rho$','Interpreter','latex');
title('Correlation with oracle node difference'); legend('Row norm','Diagonal','Coupling','Location','best');
exportgraphics(gcf,fullfile(out,'H_factor_distribution_and_correlation.png'),'Resolution',300); close(gcf);
end
function plot_band(d,X,E,label)
qx=prctile(X,[25 50 75],2); qe=prctile(E,[25 50 75],2); hold on;
fill([d;flipud(d)],[qx(:,1);flipud(qx(:,3))],[0 0.447 0.741],'FaceAlpha',0.14,'EdgeColor','none');
fill([d;flipud(d)],[qe(:,1);flipud(qe(:,3))],[0.85 0.325 0.098],'FaceAlpha',0.14,'EdgeColor','none');
plot(d,qx(:,2),'-o','LineWidth',1.3); plot(d,qe(:,2),'-s','LineWidth',1.3); grid on;
xlabel('SDA depth'); ylabel(label,'Interpreter','latex'); legend('Explicit IQR','Embedded IQR','Explicit median','Embedded median','Location','best');
end
function run=find_run(container,name)
names=string({container.runs.name}); index=find(names==name,1); assert(~isempty(index)); run=container.runs(index);
end
