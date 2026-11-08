function [Jopt, nodes, optimal, Uopt, Uapply, ...
    predictionValidNext,iabcPredictedNext, ...
    leafNodes, bestCMVCost, cmvMetricRatio, switchCountApplied, ...
    debug_lower, debug_upper, Jround, Jshift, ...
    JbabaiEmbedded, JbabaiExplicit, initialRadius, solutionToggle, ...
    feasibleSolutionFound, babaiPrefixFeasible, prefixRepairUsed, ...
    feasiblePrefixCount, currentPruneCount] = ...
    SDA_Solve_Synchronized_Hybrid_CurrentConstrained( ...
    H_exp, U_unc_exp, H_emb, U_unc_emb, NmaxNod, UoptPrev, ...
    iabcCompensated, deltaIabcBase, deltaIabcSwitch, ...
    lambdaZero, lambdaDeadtime, deadtime_ratio, currentEpsMargin, ...
    currentLimit)
%#codegen
%SDA_SOLVE_SYNCHRONIZED_HYBRID_CURRENTCONSTRAINED
% Current-constrained extension of SDA_Solve_Synchronized_Hybrid.
%
% The ordinary embedded Babai sequence remains the fast path. Its first two
% switching blocks, which are applied before the next controller transaction,
% are checked at both the dead-time endpoint and the vector endpoint. If that
% prefix is infeasible, all feasible two-block prefixes are scored only by the
% six-level embedded partial metric. The best prefix is completed once with a
% conditional embedded Babai suffix and reevaluated with the full explicit
% objective. The explicit search retains every feasible prefix and performs
% current pruning after each of the first two complete switching blocks.
%
% currentLimit is a phase-current peak limit in amperes. A nonpositive or
% nonfinite value disables the current constraint for diagnostic use.

Np = 5;
nu = 3;
n = numel(U_unc_exp);
constrainedBlocks = 2;

UbarExp = H_exp*U_unc_exp;
UbarEmb = H_emb*U_unc_emb;
uprev = UoptPrev(4:6);
deadtimeRatio = deadtime_ratio;
constraintEnabled = isfinite(currentLimit) && currentLimit > 0.0;

Uround = min(max(round(U_unc_exp),0.0),1.0);
Ushift = zeros(n,1);
Ushift(1:n-nu) = UoptPrev(nu+1:n);
Ushift(n-nu+1:n) = UoptPrev(n-nu+1:n);

% Ordinary embedded Babai candidate.
Ubabai = zeros(n,1);
for babaiLevel = 1:n
    babaiHead = 0.0;
    if babaiLevel > 1
        babaiHead = H_emb(babaiLevel,1:babaiLevel-1)* ...
            Ubabai(1:babaiLevel-1);
    end
    babaiCenter = (UbarEmb(babaiLevel)-babaiHead)/ ...
        H_emb(babaiLevel,babaiLevel);
    Ubabai(babaiLevel) = double(babaiCenter > 0.5);
end

Jround = norm(H_exp*(Uround-U_unc_exp),2)^2;
Jshift = norm(H_exp*(Ushift-U_unc_exp),2)^2;
JbabaiEmbedded = norm(H_emb*(Ubabai-U_unc_emb),2)^2;
JbabaiExplicit = norm(H_exp*(Ubabai-U_unc_exp),2)^2;

[babaiCMVCost,babaiPrefixFeasible] = evaluate_sequence( ...
    Ubabai,uprev,iabcCompensated,Np,nu, ...
    deltaIabcBase,deltaIabcSwitch,lambdaZero,lambdaDeadtime, ...
    deadtimeRatio,currentEpsMargin,currentLimit, ...
    constrainedBlocks,constraintEnabled);

prefixRepairUsed = false;
feasiblePrefixCount = 0;
hasFeasibleIncumbent = false;
Uini = zeros(n,1);
Jini = inf;
bestCMVCost = 0.0;

if babaiPrefixFeasible
    % Preserve the existing hybrid fast path exactly when it is feasible.
    Uini = Ubabai;
    Jini = JbabaiExplicit;
    bestCMVCost = babaiCMVCost;
    hasFeasibleIncumbent = true;
else
    prefixRepairUsed = true;
    bestPrefixMetric = inf;
    bestPrefix = zeros(2*nu,1);

    % Enumerate only the physical two-vector prefix. No Babai suffix is
    % formed inside this loop. The selected prefix is the feasible point
    % nearest to the unconstrained center in the first six embedded rows.
    for state1 = 0:7
        u1 = state_from_index(state1);
        [delta1,deltaDead1] = deadtime_corrected_current_step( ...
            uprev,u1,iabcCompensated,deltaIabcBase,deltaIabcSwitch, ...
            deadtimeRatio,currentEpsMargin);
        iDead1 = iabcCompensated+deltaDead1;
        iAfter1 = iabcCompensated+delta1;
        if ~current_is_feasible(iDead1,iAfter1,currentLimit, ...
                constraintEnabled)
            continue;
        end

        for state2 = 0:7
            u2 = state_from_index(state2);
            [delta2,deltaDead2] = deadtime_corrected_current_step( ...
                u1,u2,iAfter1,deltaIabcBase,deltaIabcSwitch, ...
                deadtimeRatio,currentEpsMargin);
            iDead2 = iAfter1+deltaDead2;
            iAfter2 = iAfter1+delta2;
            if ~current_is_feasible(iDead2,iAfter2,currentLimit, ...
                    constraintEnabled)
                continue;
            end

            feasiblePrefixCount = feasiblePrefixCount+1;
            prefix = [u1;u2];
            prefixResidual = UbarEmb(1:2*nu)- ...
                H_emb(1:2*nu,1:2*nu)*prefix;
            prefixMetric = prefixResidual.'*prefixResidual;
            if prefixMetric < bestPrefixMetric
                bestPrefixMetric = prefixMetric;
                bestPrefix = prefix;
            end
        end
    end

    if feasiblePrefixCount > 0
        % Compute exactly one conditional embedded Babai suffix.
        Uini(1:2*nu) = bestPrefix;
        for babaiLevel = 2*nu+1:n
            babaiHead = H_emb(babaiLevel,1:babaiLevel-1)* ...
                Uini(1:babaiLevel-1);
            babaiCenter = (UbarEmb(babaiLevel)-babaiHead)/ ...
                H_emb(babaiLevel,babaiLevel);
            Uini(babaiLevel) = double(babaiCenter > 0.5);
        end

        Jini = norm(H_exp*(Uini-U_unc_exp),2)^2;
        [bestCMVCost,initialCandidateFeasible] = evaluate_sequence( ...
            Uini,uprev,iabcCompensated,Np,nu, ...
            deltaIabcBase,deltaIabcSwitch,lambdaZero,lambdaDeadtime, ...
            deadtimeRatio,currentEpsMargin,currentLimit, ...
            constrainedBlocks,constraintEnabled);
        hasFeasibleIncumbent = initialCandidateFeasible;
    end
end

if hasFeasibleIncumbent
    bestMetric = Jini+bestCMVCost;
    Uopt = Uini;
else
    % An infeasible candidate may guide ordering but is never a finite upper
    % bound. The first feasible leaf found by the main search establishes it.
    bestMetric = inf;
    initialRadius = inf;
    Uopt = zeros(n,1);
    bestCMVCost = 0.0;
end
if hasFeasibleIncumbent
    initialRadius = bestMetric;
end

U = zeros(n,1);
kidx = ones(n,1);
d2s = zeros(n+1,1);
cmvD2s = zeros(n+1,1);
iabcPrediction = zeros(3,Np+1);
iabcPrediction(:,1) = iabcCompensated;
nodes = 0;
leafNodes = 0;
currentPruneCount = 0;
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
        head = H_exp(i,1:i-1)*U(1:i-1);
    end
    center = (UbarExp(i)-head)/H_exp(i,i);
    firstCandidate = double(center > 0.5);
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

    residual = UbarExp(i)-head-H_exp(i,i)*candidate;
    metricNew = d2s(i)+residual*residual;
    cmvMetricNew = cmvD2s(i);
    currentFeasible = true;
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
        iabcDeadEnd = iabcBeforeSwitch+deltaDead;
        iabcAfterStep = iabcBeforeSwitch+deltaStep;

        if predictionStep <= constrainedBlocks
            currentFeasible = current_is_feasible( ...
                iabcDeadEnd,iabcAfterStep,currentLimit,constraintEnabled);
        end
        if ~currentFeasible
            currentPruneCount = currentPruneCount+1;
            continue;
        end

        cmvIncrement = cmv_cost( ...
            uPrevious,uCurrent,iabcBeforeSwitch,deltaDead, ...
            lambdaZero,lambdaDeadtime,currentEpsMargin);
        metricNew = metricNew+cmvIncrement;
        cmvMetricNew = cmvMetricNew+cmvIncrement;
        iabcPrediction(:,predictionStep+1) = iabcAfterStep;
    end

    if hasFeasibleIncumbent && metricNew > bestMetric
        continue;
    end
    if i == n
        leafNodes = leafNodes+1;
        if ~hasFeasibleIncumbent || metricNew < bestMetric
            bestMetric = metricNew;
            Uopt = U;
            bestCMVCost = cmvMetricNew;
            hasFeasibleIncumbent = true;
        end
    else
        d2s(i+1) = metricNew;
        cmvD2s(i+1) = cmvMetricNew;
        i = i+1;
        kidx(i) = 1;
    end
end

feasibleSolutionFound = hasFeasibleIncumbent;
if ~feasibleSolutionFound
    % Keep the output deterministic but mark it invalid explicitly. The caller
    % must provide a separately validated safety fallback for this condition.
    Uopt = repmat(uprev,Np,1);
    bestMetric = inf;
    bestCMVCost = 0.0;
end

Uapply = Uopt(1:6);
UapplyFirst = Uopt(1:3);
UapplySecond = Uopt(4:6);

[deltaFirst,~] = deadtime_corrected_current_step( ...
    uprev,UapplyFirst,iabcCompensated, ...
    deltaIabcBase,deltaIabcSwitch,deadtimeRatio,currentEpsMargin);
iabcAfterFirst = iabcCompensated+deltaFirst;
[deltaSecond,~] = deadtime_corrected_current_step( ...
    UapplyFirst,UapplySecond,iabcAfterFirst, ...
    deltaIabcBase,deltaIabcSwitch,deadtimeRatio,currentEpsMargin);
iabcPredictedNext = iabcAfterFirst+deltaSecond;

switchCountApplied = sum(abs(UapplyFirst-uprev))+ ...
    sum(abs(UapplySecond-UapplyFirst));
Jopt = bestMetric;
if isfinite(Jopt) && Jopt > eps
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
predictionValidNext = feasibleSolutionFound;

persistent solutionToggleState
if isempty(solutionToggleState)
    solutionToggleState = false;
end
solutionToggleState = ~solutionToggleState;
solutionToggle = solutionToggleState;
end

function u = state_from_index(stateIndex)
u = zeros(3,1);
u(1) = floor(stateIndex/4);
u(2) = floor(mod(stateIndex,4)/2);
u(3) = mod(stateIndex,2);
end

function feasible = current_is_feasible( ...
    iabcDeadEnd,iabcAfterStep,currentLimit,constraintEnabled)
if ~constraintEnabled
    feasible = true;
else
    feasible = all(abs(iabcDeadEnd) <= currentLimit) && ...
        all(abs(iabcAfterStep) <= currentLimit);
end
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

function [totalPenalty,prefixFeasible] = evaluate_sequence( ...
    U,uprev,iabcInitial,Np,nu,deltaBase,deltaSwitch, ...
    lambdaZero,lambdaDeadtime,deadtimeRatio,currentEpsMargin, ...
    currentLimit,constrainedBlocks,constraintEnabled)
totalPenalty = 0.0;
prefixFeasible = true;
uPrevious = uprev;
iabcBeforeSwitch = iabcInitial;
for step = 1:Np
    rows = (step-1)*nu+(1:nu);
    uCurrent = U(rows);
    [deltaStep,deltaDead] = deadtime_corrected_current_step( ...
        uPrevious,uCurrent,iabcBeforeSwitch,deltaBase,deltaSwitch, ...
        deadtimeRatio,currentEpsMargin);
    iabcDeadEnd = iabcBeforeSwitch+deltaDead;
    iabcAfterStep = iabcBeforeSwitch+deltaStep;
    if step <= constrainedBlocks && ...
            ~current_is_feasible(iabcDeadEnd,iabcAfterStep, ...
            currentLimit,constraintEnabled)
        prefixFeasible = false;
        return;
    end
    totalPenalty = totalPenalty+cmv_cost( ...
        uPrevious,uCurrent,iabcBeforeSwitch,deltaDead, ...
        lambdaZero,lambdaDeadtime,currentEpsMargin);
    iabcBeforeSwitch = iabcAfterStep;
    uPrevious = uCurrent;
end
end
