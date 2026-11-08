function [result,trace] = SDA_Solve_Profiled_Core(input,lambdaZero,formulation,initialCandidate)
%SDA_SOLVE_PROFILED_CORE Offline replica of SDA_Solve_Synchronized.
% Records every evaluated node without changing the production controller.

H=double(input.H); U_unc=double(input.U_unc(:));
NmaxNod=double(input.NmaxNode); UoptPrev=double(input.UoptPrev(:));
iabcCompensated=double(input.iabcCompensated(:));
deltaIabcBase=double(input.deltaIabcBase(:));
deltaIabcSwitch=double(input.deltaIabcSwitch);
lambdaDeadtime=double(input.lambdaDeadtime);
deadtimeRatio=double(input.deadtimeRatio);
currentEpsMargin=double(input.currentEpsMargin);
Np=5; nu=3; n=numel(U_unc); Ubar=H*U_unc; uprev=UoptPrev(4:6);

Uround=min(max(round(U_unc),0),1);
Ushift=zeros(n,1); Ushift(1:n-nu)=UoptPrev(nu+1:n); Ushift(n-nu+1:n)=UoptPrev(n-nu+1:n);
Ubabai=zeros(n,1);
for level=1:n
    head=0; if level>1, head=H(level,1:level-1)*Ubabai(1:level-1); end
    center=(Ubar(level)-head)/H(level,level);
    Ubabai(level)=double(center>0.5);
end
Jround=norm(H*(Uround-U_unc),2)^2;
Jshift=norm(H*(Ushift-U_unc),2)^2;
Jbabai=norm(H*(Ubabai-U_unc),2)^2;
cmvRound=sequence_cmv_cost_predicted(Uround,uprev,iabcCompensated,Np,nu, ...
    deltaIabcBase,deltaIabcSwitch,lambdaZero,lambdaDeadtime,deadtimeRatio,currentEpsMargin);
cmvShift=sequence_cmv_cost_predicted(Ushift,uprev,iabcCompensated,Np,nu, ...
    deltaIabcBase,deltaIabcSwitch,lambdaZero,lambdaDeadtime,deadtimeRatio,currentEpsMargin);
cmvBabai=sequence_cmv_cost_predicted(Ubabai,uprev,iabcCompensated,Np,nu, ...
    deltaIabcBase,deltaIabcSwitch,lambdaZero,lambdaDeadtime,deadtimeRatio,currentEpsMargin);
candidateTotalCosts=[Jround+cmvRound,Jshift+cmvShift,Jbabai+cmvBabai];
if nargin>=4 && ~isempty(initialCandidate)
    Uini=double(initialCandidate(:));
    assert(numel(Uini)==n && all(Uini==0|Uini==1),'SDA:BadInitialCandidate');
    Jini=norm(H*(Uini-U_unc),2)^2; initialization="external";
else
    Uini=Uround; Jini=Jround; initialization="round";
    if Jshift<Jini, Uini=Ushift; Jini=Jshift; initialization="shift"; end
    if Jbabai<Jini, Uini=Ubabai; Jini=Jbabai; initialization="babai"; end
end

bestCMVCost=sequence_cmv_cost_predicted(Uini,uprev,iabcCompensated,Np,nu, ...
    deltaIabcBase,deltaIabcSwitch,lambdaZero,lambdaDeadtime,deadtimeRatio,currentEpsMargin);
bestMetric=Jini+bestCMVCost; initialRadius=bestMetric; Uopt=Uini;

maxTrace=2^(n+1)-2;
levelTrace=zeros(maxTrace,1,'uint8'); candidateTrace=false(maxTrace,1);
prefixTrace=zeros(maxTrace,1,'uint16'); metricBefore=zeros(maxTrace,1);
residualSquared=zeros(maxTrace,1); zeroIncrement=zeros(maxTrace,1);
deadtimeIncrement=zeros(maxTrace,1); metricAfter=zeros(maxTrace,1);
radiusBefore=zeros(maxTrace,1); margin=zeros(maxTrace,1);
pruned=false(maxTrace,1); leaf=false(maxTrace,1); radiusUpdated=false(maxTrace,1);
radiusAfter=zeros(maxTrace,1);
visitedByDepth=zeros(n,1,'uint32'); prunedByDepth=zeros(n,1,'uint32');
survivedByDepth=zeros(n,1,'uint32'); zeroPrunedByDepth=zeros(n,1,'uint32');

U=zeros(n,1); kidx=ones(n,1); d2s=zeros(n+1,1); cmvD2s=zeros(n+1,1);
iabcPrediction=zeros(3,Np+1); iabcPrediction(:,1)=iabcCompensated;
nodes=0; recorded=0; leafNodes=0; optimal=true; i=1;
while i>=1
    if kidx(i)>2, kidx(i)=1; i=i-1; continue; end
    head=0; if i>1, head=H(i,1:i-1)*U(1:i-1); end
    center=(Ubar(i)-head)/H(i,i); firstCandidate=double(center>0.5);
    if kidx(i)==1, candidate=firstCandidate; else, candidate=1-firstCandidate; end
    kidx(i)=kidx(i)+1; U(i)=candidate; nodes=nodes+1;
    if NmaxNod>0 && nodes>NmaxNod, optimal=false; break; end

    residual=Ubar(i)-head-H(i,i)*candidate;
    baseMetric=d2s(i); residualSq=residual*residual;
    metricNew=baseMetric+residualSq; cmvMetricNew=cmvD2s(i);
    zeroPart=0; deadtimePart=0;
    if mod(i,nu)==0
        predictionStep=i/nu; blockStart=i-nu+1; uCurrent=U(blockStart:i);
        if predictionStep==1, uPrevious=uprev; else, uPrevious=U(blockStart-nu:blockStart-1); end
        iabcBeforeSwitch=iabcPrediction(:,predictionStep);
        [deltaStep,deltaDead]=deadtime_corrected_current_step(uPrevious,uCurrent, ...
            iabcBeforeSwitch,deltaIabcBase,deltaIabcSwitch,deadtimeRatio,currentEpsMargin);
        cmvIncrement=cmv_cost(uPrevious,uCurrent,iabcBeforeSwitch,deltaDead, ...
            lambdaZero,lambdaDeadtime,currentEpsMargin);
        if all(uCurrent==0)||all(uCurrent==1), zeroPart=lambdaZero; end
        deadtimePart=cmvIncrement-zeroPart;
        metricNew=metricNew+cmvIncrement; cmvMetricNew=cmvMetricNew+cmvIncrement;
        iabcPrediction(:,predictionStep+1)=iabcBeforeSwitch+deltaStep;
    end

    recorded=recorded+1; radiusOld=bestMetric; isPruned=metricNew>radiusOld;
    levelTrace(recorded)=uint8(i); candidateTrace(recorded)=logical(candidate);
    prefixTrace(recorded)=encode_prefix(U,i); metricBefore(recorded)=baseMetric;
    residualSquared(recorded)=residualSq; zeroIncrement(recorded)=zeroPart;
    deadtimeIncrement(recorded)=deadtimePart; metricAfter(recorded)=metricNew;
    radiusBefore(recorded)=radiusOld; margin(recorded)=radiusOld-metricNew;
    pruned(recorded)=isPruned; leaf(recorded)=i==n;
    visitedByDepth(i)=visitedByDepth(i)+1;
    if isPruned
        prunedByDepth(i)=prunedByDepth(i)+1;
        if zeroPart>0 && baseMetric+residualSq<=radiusOld
            zeroPrunedByDepth(i)=zeroPrunedByDepth(i)+1;
        end
        radiusAfter(recorded)=bestMetric;
        continue
    else
        survivedByDepth(i)=survivedByDepth(i)+1;
    end
    if i==n
        leafNodes=leafNodes+1;
        if metricNew<bestMetric
            bestMetric=metricNew; Uopt=U; bestCMVCost=cmvMetricNew;
            radiusUpdated(recorded)=true;
        end
    else
        d2s(i+1)=metricNew; cmvD2s(i+1)=cmvMetricNew; i=i+1; kidx(i)=1;
    end
    radiusAfter(recorded)=bestMetric;
end

levelTrace=levelTrace(1:recorded); candidateTrace=candidateTrace(1:recorded);
prefixTrace=prefixTrace(1:recorded); metricBefore=metricBefore(1:recorded);
residualSquared=residualSquared(1:recorded); zeroIncrement=zeroIncrement(1:recorded);
deadtimeIncrement=deadtimeIncrement(1:recorded); metricAfter=metricAfter(1:recorded);
radiusBefore=radiusBefore(1:recorded); margin=margin(1:recorded);
pruned=pruned(1:recorded); leaf=leaf(1:recorded);
radiusUpdated=radiusUpdated(1:recorded); radiusAfter=radiusAfter(1:recorded);
trace=struct('level',levelTrace,'candidate',candidateTrace,'prefixCode',prefixTrace, ...
    'metricBefore',metricBefore,'residualSquared',residualSquared, ...
    'zeroIncrement',zeroIncrement,'deadtimeIncrement',deadtimeIncrement, ...
    'metricAfter',metricAfter,'radiusBefore',radiusBefore,'margin',margin, ...
    'pruned',pruned,'leaf',leaf,'radiusUpdated',radiusUpdated,'radiusAfter',radiusAfter, ...
    'visitedByDepth',visitedByDepth,'prunedByDepth',prunedByDepth, ...
    'survivedByDepth',survivedByDepth,'zeroPrunedByDepth',zeroPrunedByDepth);
result=struct('formulation',char(formulation),'Jopt',bestMetric,'nodes',uint32(nodes), ...
    'optimal',optimal,'Uopt',Uopt,'leafNodes',uint32(leafNodes), ...
    'bestCMVCost',bestCMVCost,'Jround',Jround,'Jshift',Jshift,'Jbabai',Jbabai, ...
    'initialization',char(initialization),'initialRadius',initialRadius, ...
    'initialQuadraticRadius',Jini,'initialCMVCost',initialRadius-Jini, ...
    'candidateNames',{{'Round','Shift','Babai'}}, ...
    'candidateTotalCosts',candidateTotalCosts, ...
    'candidateSequences',[Uround,Ushift,Ubabai]);
end

function code=encode_prefix(U,depth)
code=uint16(0);
for k=1:depth
    if U(k)>0.5, code=bitor(code,bitshift(uint16(1),k-1)); end
end
end

function [deltaStep,deltaDead]=deadtime_corrected_current_step(uPrevious,uCurrent,iabcBeforeSwitch,deltaBase,deltaSwitch,deadtimeRatio,currentEps)
uDead=uCurrent; changed=uPrevious~=uCurrent; signEps=0.1*currentEps;
for phase=1:3
    if ~changed(phase), continue; end
    v=iabcBeforeSwitch(phase);
    if v>=signEps, uDead(phase)=0; elseif v<=-signEps, uDead(phase)=1; else, uDead(phase)=0.5*(1-v/signEps); end
end
uEffective=deadtimeRatio*uDead+(1-deadtimeRatio)*uCurrent;
deltaDead=deadtimeRatio*(deltaBase+deltaSwitch*uDead);
deltaStep=deltaBase+deltaSwitch*uEffective;
end

function [penalty,lowerBound,upperBound]=cmv_cost(uPrevious,uCurrent,iabcBeforeSwitch,deltaDead,lambdaZero,lambdaDeadtime,currentEpsMargin)
penalty=0; iabcDeadEnd=iabcBeforeSwitch+deltaDead;
lowerBound=min(iabcBeforeSwitch,iabcDeadEnd)-currentEpsMargin;
upperBound=max(iabcBeforeSwitch,iabcDeadEnd)+currentEpsMargin;
uncertain=(lowerBound<=0)&(upperBound>=0);
if all(uCurrent==0)||all(uCurrent==1), penalty=penalty+lambdaZero; end
changed=uPrevious~=uCurrent; if sum(changed)~=2, return; end
canBeZero=false(3,1); canBeOne=false(3,1);
for phase=1:3
    if ~changed(phase)
        canBeZero(phase)=uPrevious(phase)==0; canBeOne(phase)=uPrevious(phase)==1;
    elseif uncertain(phase)
        canBeZero(phase)=true; canBeOne(phase)=true;
    elseif iabcDeadEnd(phase)>0
        canBeZero(phase)=true;
    else
        canBeOne(phase)=true;
    end
end
if all(canBeZero)||all(canBeOne), penalty=penalty+lambdaDeadtime; end
end

function total=sequence_cmv_cost_predicted(U,uprev,iabcInitial,Np,nu,deltaBase,deltaSwitch,lambdaZero,lambdaDeadtime,deadtimeRatio,currentEpsMargin)
total=0; uPrevious=uprev; iabcBeforeSwitch=iabcInitial;
for step=1:Np
    rows=(step-1)*nu+(1:nu); uCurrent=U(rows);
    [deltaStep,deltaDead]=deadtime_corrected_current_step(uPrevious,uCurrent,iabcBeforeSwitch,deltaBase,deltaSwitch,deadtimeRatio,currentEpsMargin);
    total=total+cmv_cost(uPrevious,uCurrent,iabcBeforeSwitch,deltaDead,lambdaZero,lambdaDeadtime,currentEpsMargin);
    iabcBeforeSwitch=iabcBeforeSwitch+deltaStep; uPrevious=uCurrent;
end
end
