function plot_deployed_hybrid_results(details,outputDirectory)
%PLOT_DEPLOYED_HYBRID_RESULTS Publication figures for closed-loop results.
if nargin<2||isempty(outputDirectory), outputDirectory=fullfile(fileparts(mfilename('fullpath')),'hybrid_results'); end
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
colors=[0.00 0.32 0.65; 0.85 0.33 0.10; 0.18 0.62 0.30];
names={'Explicit','Embedded','Hybrid'};

% Overview figures use non-overlapping 1-ms bins.  Each displayed point is
% the arithmetic mean of approximately 40 synchronized SDA transactions.
% This avoids the opaque colour block produced by plotting all 40-kHz
% samples, while retaining the complete 0--1 s simulation interval.
for k=1:numel(details)
    t=details(k).timeFull(:);
    nodeMatrix=[details(k).nodesOriginalFull(:), ...
        details(k).nodesEmbeddedFull(:),details(k).nodesHybridFull(:)];
    binWidth=1e-3;
    bin=floor((t-t(1))/binWidth)+1;
    nBins=max(bin);
    timeBinned=accumarray(bin,t,[nBins 1],@mean,NaN);
    nodesBinned=nan(nBins,3);
    for m=1:3
        nodesBinned(:,m)=accumarray(bin,nodeMatrix(:,m),[nBins 1],@mean,NaN);
    end
    f=figure('Color','w','Position',[100 100 1120 520]); hold on;
    for m=1:3
        plot(timeBinned,nodesBinned(:,m),'LineWidth',1.15, ...
            'Color',colors(m,:),'DisplayName',names{m});
    end
    xlabel('Simulation time (s)'); ylabel('Mean visited SDA nodes per 1-ms bin');
    title(sprintf('Closed-loop node-count overview at %d Nm',details(k).torque));
    legend('Location','northoutside','Orientation','horizontal');
    grid on; box on; xlim([t(1) t(end)]); ylim([0 3300]);
    base=sprintf('closed_loop_nodes_time_%dNm',details(k).torque);
    exportgraphics(f,fullfile(outputDirectory,[base '.pdf']),'ContentType','vector');
    exportgraphics(f,fullfile(outputDirectory,[base '.png']),'Resolution',300); close(f);
end

f=figure('Color','w','Position',[80 80 1500 470]);
tiledlayout(1,3,'Padding','compact','TileSpacing','compact');
for k=1:numel(details)
    nexttile; hold on;
    groups={details(k).nodesOriginal,details(k).nodesEmbedded,details(k).nodesHybrid};
    for m=1:3
        x=repmat(m,1,numel(groups{m}));
        boxchart(x,groups{m},'BoxFaceColor',colors(m,:),'MarkerStyle','.');
        plot(m,mean(groups{m}),'kd','MarkerFaceColor','w','MarkerSize',6,'LineWidth',1.1);
    end
    xticks(1:3); xticklabels(names); ylabel('Visited SDA nodes');
    title(sprintf('%d Nm',details(k).torque)); grid on; box on; xlim([0.5 3.5]); ylim([0 3300]);
end
sgtitle('Closed-loop search effort over all steady-state transactions');
exportgraphics(f,fullfile(outputDirectory,'closed_loop_node_distributions.pdf'),'ContentType','vector');
exportgraphics(f,fullfile(outputDirectory,'closed_loop_node_distributions.png'),'Resolution',300); close(f);

f=figure('Color','w','Position',[100 100 800 550]); hold on;
for k=1:numel(details)
    g=max(0,details(k).initialGap(:)); x=sort(log10(1+g)); y=(1:numel(x))'/numel(x);
    plot(x,y,'LineWidth',1.8,'Color',colors(k,:),'DisplayName',sprintf('%d Nm (optimal at init.: %.1f%%)',details(k).torque,100*mean(details(k).initialOptimal)));
end
xlabel('$\log_{10}(1+R_0-J^\star)$','Interpreter','latex'); ylabel('Empirical cumulative probability');
title('Quality of the embedded-Babai initial incumbent'); legend('Location','southeast');
grid on; box on; ylim([0 1]);
exportgraphics(f,fullfile(outputDirectory,'closed_loop_initial_gap_cdf.pdf'),'ContentType','vector');
exportgraphics(f,fullfile(outputDirectory,'closed_loop_initial_gap_cdf.png'),'Resolution',300); close(f);
end
