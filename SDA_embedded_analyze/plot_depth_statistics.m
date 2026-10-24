function plot_depth_statistics(reportFile,outputDirectory)
%PLOT_DEPTH_STATISTICS Plot depth-wise search statistics from replay output.
analysisDir=fileparts(mfilename('fullpath'));
if nargin<1||isempty(reportFile), reportFile=fullfile(analysisDir,'data','embedded_H_search_analysis.mat'); end
if nargin<2||isempty(outputDirectory), outputDirectory=fullfile(analysisDir,'figures'); end
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
loaded=load(reportFile,'report'); report=loaded.report;
torques={'10NM','50NM','90NM'}; depth=(1:15).';
for t=1:numel(torques)
    embedded=find_run(report,"embedded_H_"+torques{t});
    discrete=find_run(report,"original_"+torques{t});
    e=embedded.summary; d=discrete.summary;
    eVisit=mean(double(e.visitedByDepth),2); dVisit=mean(double(d.visitedByDepth),2);
    ePrune=sum(double(e.prunedByDepth),2)./max(1,sum(double(e.visitedByDepth),2));
    dPrune=sum(double(d.prunedByDepth),2)./max(1,sum(double(d.visitedByDepth),2));
    figure('Color','w','Position',[100 100 1150 430]); tiledlayout(1,2,'Padding','compact');
    nexttile; plot(depth,dVisit,'-o',depth,eVisit,'-s','LineWidth',1.4); grid on;
    xlabel('SDA depth'); ylabel('Mean visited nodes');
    legend('Explicit branch penalty','Embedded H','Location','best'); title(torques{t});
    nexttile; plot(depth,dPrune,'-o',depth,ePrune,'-s','LineWidth',1.4); grid on;
    xlabel('SDA depth'); ylabel('Pruned / visited'); ylim([0 1]);
    legend('Explicit branch penalty','Embedded H','Location','best'); title('Depth-wise pruning rate');
    exportgraphics(gcf,fullfile(outputDirectory,"depth_statistics_"+torques{t}+".png"),'Resolution',300);
    close(gcf);
end
end

function run=find_run(report,name)
names=string({report.runs.name}); index=find(names==name,1);
assert(~isempty(index),'SDA:MissingRun','Missing run %s.',name); run=report.runs(index);
end
