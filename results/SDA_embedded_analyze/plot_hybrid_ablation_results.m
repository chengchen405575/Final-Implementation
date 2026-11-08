function plot_hybrid_ablation_results(resultFile,outputDirectory)
analysisDir=fileparts(mfilename('fullpath'));
if nargin<1||isempty(resultFile), resultFile=fullfile(analysisDir,'hybrid_results','hybrid_ablation_results.mat'); end
if nargin<2||isempty(outputDirectory), outputDirectory=fullfile(analysisDir,'hybrid_results'); end
L=load(resultFile,'study'); study=L.study;
for t=1:3
    d=study(t).data; names=d.names;
    groups=categorical(repmat(names,1,size(d.nodes,2)),names,names);
    figure('Color','w','Position',[80 80 1250 790]); tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
    nexttile; boxchart(groups,d.nodes(:)); ylabel('Visited nodes'); title('Total search effort'); grid on;
    nexttile; boxchart(groups,log10(1+d.initialGap(:))); ylabel('$\log_{10}(1+R_0-J^\star)$','Interpreter','latex'); title('Initial upper-bound gap'); grid on;
    nexttile; bar(categorical(names,names,names),mean(d.discoveryNodes,2)); ylabel('Mean nodes'); title('Incumbent discovery stage'); grid on;
    nexttile; bar(categorical(names,names,names),mean(d.proofNodes,2)); ylabel('Mean nodes'); title('Optimality proof stage'); grid on;
    sgtitle(study(t).torque+" hybrid-SDA ablation");
    exportgraphics(gcf,fullfile(outputDirectory,"hybrid_ablation_"+study(t).torque+".png"),'Resolution',300); close(gcf);
end
end
