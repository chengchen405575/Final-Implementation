function selections = generate_diagnostic_figures(datasetFile,reportFile,outputDirectory)
%GENERATE_DIAGNOSTIC_FIGURES Create paper-candidate SDA search figures.
analysisDir=fileparts(mfilename('fullpath'));
if nargin<1||isempty(datasetFile), datasetFile=fullfile(analysisDir,'data','embedded_H_sda_cases_2025b.mat'); end
if nargin<2||isempty(reportFile), reportFile=fullfile(analysisDir,'data','embedded_H_search_analysis.mat'); end
if nargin<3||isempty(outputDirectory), outputDirectory=fullfile(analysisDir,'figures'); end
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
A=load(datasetFile,'archive'); archive=A.archive;
R=load(reportFile,'report'); report=R.report;
torques=["10NM","50NM","90NM"];

% Full-data paired distributions and exact leaf-cost offset.
figure('Color','w','Position',[100 100 1200 460]); tiledlayout(1,2,'Padding','compact');
nexttile; hold on; colors=lines(3);
for t=1:3
    e=find_run(archive,"embedded_H_"+torques(t)); d=find_run(archive,"original_"+torques(t));
    mask=e.time>=0.05; delta=double(e.reference.nodes(mask))-double(d.reference.nodes(mask));
    [f,x]=ksdensity(delta); plot(x,f,'LineWidth',1.6,'Color',colors(t,:));
end
xline(0,'k--'); grid on; xlabel('$N_{\rm emb}-N_{\rm exp}$','Interpreter','latex');
ylabel('Probability density'); legend(torques,'Location','best'); title('Paired node-count difference');
nexttile; hold on;
for t=1:3
    e=find_run(archive,"embedded_H_"+torques(t)); d=find_run(archive,"original_"+torques(t));
    mask=e.time>=0.05; delta=double(e.reference.Jopt(mask))-double(d.reference.Jopt(mask));
    plot(e.time(mask),delta-6250,'Color',colors(t,:));
end
yline(0,'k--'); grid on; xlabel('Time (s)');
ylabel('$J_{\rm emb}-J_{\rm exp}-6250$','Interpreter','latex');
legend(torques,'Location','best'); title('Numerical deviation from the predicted offset');
exportgraphics(gcf,fullfile(outputDirectory,'global_node_difference_and_cost_offset.png'),'Resolution',300); close(gcf);

% Mean depth statistics from independent replay.
plot_depth_statistics(reportFile,outputDirectory);

% Reproducible representative cases at the 5th, 50th, and 95th percentiles.
selections=struct; percentiles=[5 50 95];
for t=1:3
    e=find_run(archive,"embedded_H_"+torques(t)); d=find_run(archive,"original_"+torques(t));
    valid=find(e.time>=0.05); delta=double(e.reference.nodes(valid))-double(d.reference.nodes(valid));
    targets=prctile(delta,percentiles); indices=zeros(1,3);
    for p=1:3, [~,near]=min(abs(delta-targets(p))); indices(p)=valid(near); end
    selections(t).torque=char(torques(t)); selections(t).percentiles=percentiles;
    selections(t).caseIndex=indices; selections(t).time=e.time(indices);
    selections(t).nodeDifference=double(e.reference.nodes(indices))-double(d.reference.nodes(indices));
    for p=1:3
        ei=select_case(e.input,indices(p)); di=select_case(d.input,indices(p));
        [er,et]=SDA_Solve_Embedded_Profiled(ei); [dr,dt]=SDA_Solve_Discrete_Profiled(di);
        make_case_figure(et,dt,er,dr,torques(t),percentiles(p),indices(p),e.time(indices(p)),outputDirectory);
    end
end
save(fullfile(outputDirectory,'representative_case_selections.mat'),'selections');
end

function make_case_figure(et,dt,er,dr,torque,percentile,index,time,outputDirectory)
depth=(1:15).';
ePrune=double(et.prunedByDepth)./max(1,double(et.visitedByDepth));
dPrune=double(dt.prunedByDepth)./max(1,double(dt.visitedByDepth));
figure('Color','w','Position',[80 60 1250 820]); tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
nexttile; plot(depth,double(dt.visitedByDepth),'-o',depth,double(et.visitedByDepth),'-s','LineWidth',1.4); grid on;
xlabel('SDA depth'); ylabel('Visited nodes'); legend('Explicit','Embedded','Location','best'); title('Nodes by depth');
nexttile; plot(depth,dPrune,'-o',depth,ePrune,'-s','LineWidth',1.4); grid on; ylim([0 1]);
xlabel('SDA depth'); ylabel('Pruned / visited'); title('Pruning rate');
nexttile; hold on;
[dm,dq1,dq3]=margin_quantiles(dt); [em,eq1,eq3]=margin_quantiles(et);
fill([depth;flipud(depth)],[dq1;flipud(dq3)],[0 0.447 0.741],'FaceAlpha',0.15,'EdgeColor','none');
fill([depth;flipud(depth)],[eq1;flipud(eq3)],[0.85 0.325 0.098],'FaceAlpha',0.15,'EdgeColor','none');
plot(depth,dm,'-o','LineWidth',1.4); plot(depth,em,'-s','LineWidth',1.4); yline(0,'k--'); grid on;
xlabel('SDA depth'); ylabel('$R-P_i$','Interpreter','latex'); title('Pruning margin: median and IQR');
legend('Explicit IQR','Embedded IQR','Explicit median','Embedded median','Location','best');
nexttile; stairs(1:numel(dt.radiusBefore),dt.radiusBefore,'LineWidth',1.3); hold on;
stairs(1:numel(et.radiusBefore),et.radiusBefore,'LineWidth',1.3); grid on;
xlabel('Visited-node index'); ylabel('Current sphere radius'); title('Dynamic incumbent radius');
legend(sprintf('Explicit (%d nodes)',dr.nodes),sprintf('Embedded (%d nodes)',er.nodes),'Location','best');
sgtitle(sprintf('%s, node-difference Q%d, case %d, t=%.6f s',torque,percentile,index,time));
name=sprintf('representative_%s_Q%02d_case_%d.png',torque,percentile,index);
exportgraphics(gcf,fullfile(outputDirectory,name),'Resolution',300); close(gcf);
end

function [med,q1,q3]=margin_quantiles(trace)
med=nan(15,1); q1=med; q3=med;
for d=1:15
    x=trace.margin(double(trace.level)==d);
    if ~isempty(x), values=prctile(x,[25 50 75]); q1(d)=values(1); med(d)=values(2); q3(d)=values(3); end
end
end
function run=find_run(container,name)
names=string({container.runs.name}); index=find(names==name,1);
assert(~isempty(index),'SDA:MissingRun','Missing %s.',name); run=container.runs(index);
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
