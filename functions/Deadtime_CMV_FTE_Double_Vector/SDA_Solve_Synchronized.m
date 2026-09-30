function [Jopt, nodes, optimal, Uopt, Uapply, ...
    predictionValidNext,iabcPredictedNext, ...
    leafNodes, bestCMVCost, cmvMetricRatio, switchCountApplied, ...
    debug_lower, debug_upper, Jround, Jshift, solutionToggle] = ...
    SDA_Solve_Synchronized(H, U_unc, NmaxNod, UoptPrev, ...
    iabcCompensated, deltaIabcBase, deltaIabcSwitch, ...
    lambdaZero, lambdaDeadtime, deadtime_ratio, currentEpsMargin)
%SDA_SOLVE Independent double-vector SDA MATLAB Function module.
% The first five ports follow FCSMPC_IdealSDA. Feed Uopt back to UoptPrev
% through a Unit Delay initialized with zeros(15,1). UapplyFirst and
% UapplySecond are applied successively for half an external period each.

Np = 5;
nu = 3;
n = numel(U_unc);

Ubar = H*U_unc;
uprev = UoptPrev(4:6);
deadtimeRatio = deadtime_ratio;

Uround = min(max(round(U_unc),0.0),1.0);
Ushift = zeros(n,1);
Ushift(1:n-nu) = UoptPrev(nu+1:n);
Ushift(n-nu+1:n) = UoptPrev(n-nu+1:n);
Jround = norm(H*(Uround-U_unc),2)^2;
Jshift = norm(H*(Ushift-U_unc),2)^2;
if Jshift < Jround
    Uini = Ushift;
else
    Uini = Uround;
end

bestCMVCost = sequence_cmv_cost_predicted( ...
    Uini,uprev,iabcCompensated,Np,nu, ...
    deltaIabcBase,deltaIabcSwitch,lambdaZero,lambdaDeadtime, ...
    deadtimeRatio,currentEpsMargin);
bestMetric = norm(H*(Uini-U_unc),2)^2+bestCMVCost;
Uopt = Uini;

U = zeros(n,1);
kidx = ones(n,1);
d2s = zeros(n+1,1);
cmvD2s = zeros(n+1,1);
iabcPrediction = zeros(3,Np+1);
iabcPrediction(:,1) = iabcCompensated;
nodes = 0;
leafNodes = 0;
optimal = true;
i = 1;

while i >= 1
    if kidx(i) > 2
        kidx(i) = 1;
        i = i-1;
        continue;
    end
    head = 0.0;
    if i > 1
        head = H(i,1:i-1)*U(1:i-1);
    end
    center = (Ubar(i)-head)/H(i,i);
    if center <= 0.5
        firstCandidate = 0.0;
    else
        firstCandidate = 1.0;
    end
    if kidx(i) == 1
        candidate = firstCandidate;
    else
        candidate = 1.0-firstCandidate;
    end
    kidx(i) = kidx(i)+1;
    U(i) = candidate;
    nodes = nodes+1;
    if NmaxNod > 0 && nodes > NmaxNod
        optimal = false;
        break;
    end

    residual = Ubar(i)-head-H(i,i)*candidate;
    metricNew = d2s(i)+residual*residual;
    cmvMetricNew = cmvD2s(i);
    if mod(i,nu) == 0
        predictionStep = i/nu;
        blockStart = i-nu+1;
        uCurrent = U(blockStart:i);
        if predictionStep == 1
            uPrevious = uprev;
        else
            uPrevious = U(blockStart-nu:blockStart-1);
        end
        iabcBeforeSwitch = iabcPrediction(:,predictionStep);
        [deltaStep,deltaDead] = deadtime_corrected_current_step( ...
            uPrevious,uCurrent,iabcBeforeSwitch, ...
            deltaIabcBase,deltaIabcSwitch,deadtimeRatio,currentEpsMargin);
        cmvIncrement = cmv_cost( ...
            uPrevious,uCurrent,iabcBeforeSwitch,deltaDead, ...
            lambdaZero,lambdaDeadtime,currentEpsMargin);
        metricNew = metricNew+cmvIncrement;
        cmvMetricNew = cmvMetricNew+cmvIncrement;
        iabcPrediction(:,predictionStep+1) = iabcBeforeSwitch+deltaStep;
    end
    if metricNew > bestMetric
        continue;
    end
    if i == n
        leafNodes = leafNodes+1;
        if metricNew < bestMetric
            bestMetric = metricNew;
            Uopt = U;
            bestCMVCost = cmvMetricNew;
        end
    else
        d2s(i+1) = metricNew;
        cmvD2s(i+1) = cmvMetricNew;
        i = i+1;
        kidx(i) = 1;
    end
end

if ~isfinite(bestMetric)
    Uopt = repmat(uprev,Np,1);
end

Uapply = Uopt(1:6);
UapplyFirst = Uopt(1:3);
UapplySecond = Uopt(4:6);

[deltaFirst,~] = deadtime_corrected_current_step( ...
    uprev,UapplyFirst,iabcCompensated, ...
    deltaIabcBase,deltaIabcSwitch, ...
    deadtimeRatio,currentEpsMargin);
iabcAfterFirst = iabcCompensated + deltaFirst;

[deltaSecond,~] = deadtime_corrected_current_step( ...
    UapplyFirst,UapplySecond,iabcAfterFirst, ...
    deltaIabcBase,deltaIabcSwitch,deadtimeRatio,currentEpsMargin);
iabcPredictedNext = iabcAfterFirst+deltaSecond;

switchCountApplied = sum(abs(UapplyFirst-uprev))+ ...
    sum(abs(UapplySecond-UapplyFirst));
Jopt = bestMetric;
if Jopt > eps
    cmvMetricRatio = bestCMVCost/Jopt;
else
    cmvMetricRatio = 0.0;
end
[~,debugDeltaDead] = deadtime_corrected_current_step( ...
    uprev,UapplyFirst,iabcCompensated,deltaIabcBase,deltaIabcSwitch, ...
    deadtimeRatio,currentEpsMargin);
[~,debug_lower,debug_upper] = cmv_cost( ...
    uprev,UapplyFirst,iabcCompensated,debugDeltaDead, ...
    lambdaZero,lambdaDeadtime,currentEpsMargin);
predictionValidNext = true;

% Toggle once after every completed SDA solution. The downstream switch
% detects this edge and explicitly restarts from the first vector.
persistent solutionToggleState
if isempty(solutionToggleState)
    solutionToggleState = false;
end
solutionToggleState = ~solutionToggleState;
solutionToggle = solutionToggleState;
end

function [deltaStep,deltaDead] = deadtime_corrected_current_step( ...
    uPrevious,uCurrent,iabcBeforeSwitch, ...
    deltaBase,deltaSwitch,deadtimeRatio,currentEps)
uDead = uCurrent;
changed = (uPrevious ~= uCurrent);
signEps = 0.1*currentEps;
for phase = 1:3
    if ~changed(phase)
        continue;
    end
    iPhase = iabcBeforeSwitch(phase);
    if iPhase >= signEps
        uDead(phase) = 0.0;
    elseif iPhase <= -signEps
        uDead(phase) = 1.0;
    else
        uDead(phase) = 0.5*(1.0-iPhase/signEps);
    end
end
uEffective = deadtimeRatio*uDead+(1.0-deadtimeRatio)*uCurrent;
deltaDead = deadtimeRatio*(deltaBase+deltaSwitch*uDead);
deltaStep = deltaBase+deltaSwitch*uEffective;
end

function [penalty,lowerBound,upperBound] = cmv_cost( ...
    uPrevious,uCurrent,iabcBeforeSwitch,deltaDead, ...
    lambdaZero,lambdaDeadtime,currentEpsMargin)
penalty = 0.0;
iabcDeadEnd = iabcBeforeSwitch+deltaDead;
lowerBound = min(iabcBeforeSwitch,iabcDeadEnd)-currentEpsMargin;
upperBound = max(iabcBeforeSwitch,iabcDeadEnd)+currentEpsMargin;
uncertain = (lowerBound <= 0.0) & (upperBound >= 0.0);
if all(uCurrent == 0.0) || all(uCurrent == 1.0)
    penalty = penalty+lambdaZero;
end
changed = (uPrevious ~= uCurrent);
if sum(changed) ~= 2
    return;
end
canBeZero = false(3,1);
canBeOne = false(3,1);
for phase = 1:3
    if ~changed(phase)
        canBeZero(phase) = (uPrevious(phase) == 0.0);
        canBeOne(phase) = (uPrevious(phase) == 1.0);
    elseif uncertain(phase)
        canBeZero(phase) = true;
        canBeOne(phase) = true;
    elseif iabcDeadEnd(phase) > 0.0
        canBeZero(phase) = true;
    else
        canBeOne(phase) = true;
    end
end
if all(canBeZero) || all(canBeOne)
    penalty = penalty+lambdaDeadtime;
end
end

function totalPenalty = sequence_cmv_cost_predicted( ...
    U,uprev,iabcInitial,Np,nu,deltaBase,deltaSwitch, ...
    lambdaZero,lambdaDeadtime,deadtimeRatio,currentEpsMargin)
totalPenalty = 0.0;
uPrevious = uprev;
iabcBeforeSwitch = iabcInitial;
for step = 1:Np
    rows = (step-1)*nu+(1:nu);
    uCurrent = U(rows);
    [deltaStep,deltaDead] = deadtime_corrected_current_step( ...
        uPrevious,uCurrent,iabcBeforeSwitch,deltaBase,deltaSwitch, ...
        deadtimeRatio,currentEpsMargin);
    totalPenalty = totalPenalty+cmv_cost( ...
        uPrevious,uCurrent,iabcBeforeSwitch,deltaDead, ...
        lambdaZero,lambdaDeadtime,currentEpsMargin);
    iabcBeforeSwitch = iabcBeforeSwitch+deltaStep;
    uPrevious = uCurrent;
end
end
