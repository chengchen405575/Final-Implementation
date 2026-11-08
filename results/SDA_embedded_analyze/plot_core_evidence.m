function plot_core_evidence(resultFile,outputDirectory)
%PLOT_CORE_EVIDENCE Produce the two figures used by the concise report.
analysisDir=fileparts(mfilename('fullpath'));
if nargin<1||isempty(resultFile), resultFile=fullfile(analysisDir,'hybrid_results','hybrid_ablation_results.mat'); end
if nargin<2||isempty(outputDirectory), outputDirectory=fullfile(analysisDir,'hybrid_results'); end
L=load(resultFile,'study'); study=L.study;
methodNames=study(1).data.names;

% Figure 1: only the evidence needed for the embedded upper-bound claim.
figure('Color','w','Position',[100 100 1120 440]); tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile; hold on; colors=lines(3);
for t=1:3
    d=study(t).data;
    x=[ones(1,size(d.initialGap,2)) 2*ones(1,size(d.initialGap,2))]+0.12*(t-2);
    y=[log10(1+d.initialGap(1,:)) log10(1+d.initialGap(2,:))];
    boxchart(x,y,'BoxFaceColor',colors(t,:),'MarkerStyle','none','BoxWidth',0.10);
end
set(gca,'XTick',[1 2],'XTickLabel',{'Native explicit','Embedded Babai'}); grid on;
ylabel('$\log_{10}(1+R_0-J^\star)$','Interpreter','latex'); title('Initial upper-bound gap');
legend({'10 Nm','50 Nm','90 Nm'},'Location','best');
nexttile; values=zeros(3,2);
for t=1:3, values(t,:)=100*mean(study(t).data.initialOptimal([1 2],:),2).'; end
bar(categorical({'10 Nm','50 Nm','90 Nm'}),values); grid on; ylim([0 60]);
ylabel('Initially optimal cases (%)'); legend('Native explicit','Embedded Babai','Location','best');
title('Probability of an optimal initial incumbent');
exportgraphics(gcf,fullfile(outputDirectory,'core_upper_bound_evidence.png'),'Resolution',300); close(gcf);

% Figure 2: total node distributions for the implementable and oracle cases.
figure('Color','w','Position',[80 90 1320 430]); tiledlayout(1,3,'Padding','compact','TileSpacing','compact');
for t=1:3
    nexttile; d=study(t).data;
    groups=categorical(repmat(methodNames,1,size(d.nodes,2)),methodNames,methodNames);
    boxchart(groups,d.nodes(:),'MarkerStyle','none'); grid on; ylabel('Visited nodes');
    title(study(t).torque); xtickangle(28);
end
exportgraphics(gcf,fullfile(outputDirectory,'core_node_count_ablation.png'),'Resolution',300); close(gcf);
end
