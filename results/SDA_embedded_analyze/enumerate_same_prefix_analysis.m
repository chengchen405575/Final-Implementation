function result = enumerate_same_prefix_analysis(datasetFile,selectionFile,outputDirectory)
%ENUMERATE_SAME_PREFIX_ANALYSIS Compare both metrics on identical prefixes.
analysisDir=fileparts(mfilename('fullpath'));
if nargin<1||isempty(datasetFile), datasetFile=fullfile(analysisDir,'data','embedded_H_sda_cases_2025b.mat'); end
if nargin<2||isempty(selectionFile), selectionFile=fullfile(analysisDir,'figures','representative_case_selections.mat'); end
if nargin<3||isempty(outputDirectory), outputDirectory=fullfile(analysisDir,'figures','same_prefix'); end
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
A=load(datasetFile,'archive'); archive=A.archive; S=load(selectionFile,'selections'); selections=S.selections;
percentiles=[5 50 95]; result=struct; cursor=0;
for t=1:numel(selections)
    torque=string(selections(t).torque);
    e=find_run(archive,"embedded_H_"+torque); d=find_run(archive,"original_"+torque);
    for p=1:3
        index=selections(t).caseIndex(p); ei=select_case(e.input,index); di=select_case(d.input,index);
        [er,~]=SDA_Solve_Embedded_Profiled(ei); [dr,~]=SDA_Solve_Discrete_Profiled(di);
        stats=evaluate_all_depths(ei,di,er.initialRadius,dr.initialRadius);
        cursor=cursor+1; result(cursor).torque=char(torque); result(cursor).percentile=percentiles(p);
        result(cursor).caseIndex=index; result(cursor).time=e.time(index);
        result(cursor).embeddedRadius=er.initialRadius; result(cursor).explicitRadius=dr.initialRadius;
        result(cursor).stats=stats;
        make_figure(stats,er.initialRadius,dr.initialRadius,torque,percentiles(p),index,outputDirectory);
        fprintf('%s Q%02d case=%d leaf offset median=%.9g range=[%.9g %.9g]\n', ...
            torque,percentiles(p),index,stats.Cmedian(15),stats.Cmin(15),stats.Cmax(15));
    end
end
save(fullfile(outputDirectory,'same_prefix_results.mat'),'result','-v7.3');
end

function stats=evaluate_all_depths(ei,di,Remb,Rexp)
n=numel(ei.U_unc); C=6250; names={'Cmedian','Cq25','Cq75','Cmin','Cmax','Smedian', ...
    'embeddedInside','explicitInside','embeddedOnly','explicitOnly'};
for k=1:numel(names), stats.(names{k})=zeros(n,1); end
stats.depth=(1:n).'; stats.count=zeros(n,1);
for depth=1:n
    count=2^depth; Pembed=zeros(count,1); Pexplicit=zeros(count,1);
    for code=0:count-1
        prefix=zeros(depth,1);
        for bit=1:depth, prefix(bit)=bitget(uint32(code),bit); end
        Pembed(code+1)=prefix_metric(ei,prefix,0.0);
        Pexplicit(code+1)=prefix_metric(di,prefix,double(di.lambdaZeroLogged));
    end
    D=Pembed-Pexplicit; q=prctile(D,[25 50 75]);
    stats.Cq25(depth)=q(1); stats.Cmedian(depth)=q(2); stats.Cq75(depth)=q(3);
    stats.Cmin(depth)=min(D); stats.Cmax(depth)=max(D); stats.Smedian(depth)=C-q(2);
    inE=Pembed<=Remb; inD=Pexplicit<=Rexp;
    stats.embeddedInside(depth)=mean(inE); stats.explicitInside(depth)=mean(inD);
    stats.embeddedOnly(depth)=mean(inE & ~inD); stats.explicitOnly(depth)=mean(inD & ~inE);
    stats.count(depth)=count;
end
end

function metric=prefix_metric(input,U,lambdaZero)
H=double(input.H); Uunc=double(input.U_unc(:)); depth=numel(U); z=H*Uunc;
metric=0;
for i=1:depth
    residual=z(i)-H(i,1:i)*U(1:i); metric=metric+residual^2;
end
completed=floor(depth/3); uprev=double(input.UoptPrev(4:6));
iabc=double(input.iabcCompensated(:)); base=double(input.deltaIabcBase(:)); sw=double(input.deltaIabcSwitch);
for step=1:completed
    rows=(step-1)*3+(1:3); current=U(rows);
    [deltaStep,deltaDead]=deadtime_step(uprev,current,iabc,base,sw,double(input.deadtimeRatio),double(input.currentEpsMargin));
    metric=metric+cmv_penalty(uprev,current,iabc,deltaDead,lambdaZero,double(input.lambdaDeadtime),double(input.currentEpsMargin));
    iabc=iabc+deltaStep; uprev=current;
end
end

function make_figure(s,Remb,Rexp,torque,percentile,index,out)
d=s.depth;
figure('Color','w','Position',[80 70 1200 790]); tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
nexttile; hold on;
fill([d;flipud(d)],[s.Cq25;flipud(s.Cq75)],[0.85 0.325 0.098],'FaceAlpha',0.2,'EdgeColor','none');
plot(d,s.Cmedian,'-s','LineWidth',1.5); yline(6250,'k--','Leaf offset 6250'); grid on;
xlabel('Prefix depth'); ylabel('$C_i^{\rm eff}=P_i^{\rm emb}-P_i^{\rm exp}$','Interpreter','latex');
title('Effective offset on identical prefixes'); legend('IQR','Median','Location','best');
nexttile; plot(d,s.Smedian,'-o','LineWidth',1.5); yline(0,'k--'); grid on;
xlabel('Prefix depth'); ylabel('$S_i=6250-C_i^{\rm eff}$','Interpreter','latex'); title('Median representation-induced slack');
nexttile; plot(d,s.explicitInside,'-o',d,s.embeddedInside,'-s','LineWidth',1.5); grid on; ylim([0 1]);
xlabel('Prefix depth'); ylabel('Fraction inside initial sphere'); legend('Explicit','Embedded','Location','best');
title(sprintf('Initial radii: explicit %.1f, embedded %.1f',Rexp,Remb));
nexttile; plot(d,s.embeddedOnly,'-s',d,s.explicitOnly,'-o','LineWidth',1.5); grid on; ylim([0 1]);
xlabel('Prefix depth'); ylabel('Fraction of all prefixes');
legend('Inside embedded only','Inside explicit only','Location','best'); title('Disagreement of prefix decisions');
sgtitle(sprintf('%s Q%02d identical-prefix landscape, case %d',torque,percentile,index));
exportgraphics(gcf,fullfile(out,sprintf('same_prefix_%s_Q%02d_case_%d.png',torque,percentile,index)),'Resolution',300); close(gcf);
end

function [deltaStep,deltaDead]=deadtime_step(previous,current,iabc,base,sw,ratio,epsMargin)
uDead=current; changed=previous~=current; signEps=0.1*epsMargin;
for phase=1:3
    if ~changed(phase), continue; end
    if iabc(phase)>=signEps, uDead(phase)=0; elseif iabc(phase)<=-signEps, uDead(phase)=1;
    else, uDead(phase)=0.5*(1-iabc(phase)/signEps); end
end
uEffective=ratio*uDead+(1-ratio)*current; deltaDead=ratio*(base+sw*uDead); deltaStep=base+sw*uEffective;
end
function penalty=cmv_penalty(previous,current,iabc,deltaDead,lambdaZero,lambdaDead,epsMargin)
penalty=0; ending=iabc+deltaDead; lower=min(iabc,ending)-epsMargin; upper=max(iabc,ending)+epsMargin;
uncertain=lower<=0 & upper>=0; if all(current==0)||all(current==1), penalty=penalty+lambdaZero; end
changed=previous~=current; if sum(changed)~=2, return; end
can0=false(3,1); can1=false(3,1);
for phase=1:3
    if ~changed(phase), can0(phase)=previous(phase)==0; can1(phase)=previous(phase)==1;
    elseif uncertain(phase), can0(phase)=true; can1(phase)=true;
    elseif ending(phase)>0, can0(phase)=true; else, can1(phase)=true; end
end
if all(can0)||all(can1), penalty=penalty+lambdaDead; end
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
