function [HOut,U_unc,iabcReconstruction,idiqCompensated, ...
    deltaIabcBase,deltaIabcSwitch,Bs, ...
    predictionError,reconstructionError, ...
    predictionError_dq,reconstructionError_dq, ...
    iabcAverageEstimated,iabcRippleEstimated,rippleResidual, ...
    iabcSwitchCorrectionRaw] = ...
    QP_Runtime_40k(H,Gamma,Phi,Lambdafsw_SE,Upsil, ...
    Ad,Bd,fd,Bs_TI,idiqFiltered,idiq_ref, ...
    Sin_Cos_Theta,wel,Ts,U_applied,controllerDelay, ...
    iabcPredictedPrevious,Iabc_Motor,predictionValid, ...
    filterTimeConstant,sensorTimeConstant)
%#codegen
% Fast current reconstruction plus the runtime branch of Assembly_QP.
% filterDelayGain was removed because the original implementation did not
% use it after input conversion. H is passed through for SDA_Solve wiring.

[iabcReconstruction,idiqCompensated, ...
    deltaIabcBase,deltaIabcSwitch,Bs, ...
    predictionError,reconstructionError, ...
    predictionError_dq,reconstructionError_dq, ...
    iabcAverageEstimated,iabcRippleEstimated,rippleResidual, ...
    iabcSwitchCorrectionRaw] = current_reconstruction( ...
    Ad,Bd,fd,Bs_TI,idiqFiltered,Sin_Cos_Theta,wel,Ts,U_applied, ...
    controllerDelay,iabcPredictedPrevious,Iabc_Motor,predictionValid, ...
    filterTimeConstant,sensorTimeConstant);

% Assembly_QP runtime branch; equations and ordering are unchanged.
Np = 5;
r = Gamma*idiqCompensated+Phi-repmat(idiq_ref,Np,1);
uPrev = U_applied(4:6);
F = Upsil.'*r-Lambdafsw_SE*uPrev;
y = H.'\(-F);
U_unc = H\y;
HOut = H;
end

function [iabcReconstruction,idiqCompensated, ...
    deltaIabcBase,deltaIabcSwitch,Bs, ...
    predictionError,reconstructionError, ...
    predictionError_dq,reconstructionError_dq, ...
    iabcAverageEstimated,iabcRippleEstimated,rippleResidual, ...
    iabcSwitchCorrectionRaw] = current_reconstruction( ...
    Ad,Bd,fd,Bs_TI,idiqFiltered,Sin_Cos_Theta,wel,Ts,U_applied, ...
    controllerDelay,iabcPredictedPrevious,Iabc_Motor,predictionValid, ...
    filterTimeConstant,sensorTimeConstant)

Ad = double(Ad);
Bd = double(Bd);
fd = double(fd);
Bs_TI = double(Bs_TI);
idiqFiltered = double(idiqFiltered);
Sin_Cos_Theta = double(Sin_Cos_Theta);
wel = double(wel);
Ts = max(realmin('double'),double(Ts));
vectorStep = Ts;
doubleVectorPeriod = 2.0*vectorStep;
U_applied = double(U_applied);
controllerDelay = max(0.0,double(controllerDelay));
iabcPredictedPrevious = double(iabcPredictedPrevious);
Iabc_Motor = double(Iabc_Motor);
predictionValid = logical(predictionValid);
filterTimeConstant = max(0.0,double(filterTimeConstant));
sensorTimeConstant = max(0.0,double(sensorTimeConstant));

sinTheta = Sin_Cos_Theta(1);
cosTheta = Sin_Cos_Theta(2);
Tc = [2/3, -1/3, -1/3; 0, 1/sqrt(3), -1/sqrt(3)];
Rp = [cosTheta, sinTheta; -sinTheta, cosTheta];
TinvClarke = [1.0, 0.0; -0.5, sqrt(3)/2; -0.5, -sqrt(3)/2];
Tdq2abc = TinvClarke*Rp.';

uFirst = U_applied(1:3);
uSecond = U_applied(4:6);
uAverage = 0.5*(uFirst+uSecond);

omegaFilter = wel*filterTimeConstant;
omegaSensor = wel*sensorTimeConstant;
magnitudeGain = sqrt(1.0+omegaFilter*omegaFilter)*sqrt(1.0+omegaSensor*omegaSensor);
iabcAmpCorrected = magnitudeGain*(Tdq2abc*idiqFiltered);

w = abs(wel);
if w > 1.0e-6
    basePhaseDelay = (atan(w*filterTimeConstant)+atan(w*sensorTimeConstant))/w;
else
    basePhaseDelay = filterTimeConstant+sensorTimeConstant;
end

[~,deltaSwitchAtSample,~,~] = abc_current_increment( ...
    iabcAmpCorrected,Ad,Bd,fd,Bs_TI,wel,Ts,Tc,Rp,Tdq2abc);
switchFrequency_kHz = 17.0;
omegaSwitch = 2.0*pi*switchFrequency_kHz*1.0e3;
omegaRipple = max(1.0,omegaSwitch);
omegaRippleFilter = omegaRipple*filterTimeConstant;
omegaRippleSensor = omegaRipple*sensorTimeConstant;
rippleAttenuation = 1.0/(sqrt(1.0+omegaRippleFilter*omegaRippleFilter)* ...
    sqrt(1.0+omegaRippleSensor*omegaRippleSensor));
rippleReconstructionDelay = (atan(omegaRipple*filterTimeConstant)+ ...
    atan(omegaRipple*sensorTimeConstant))/omegaRipple;
rippleReconstructionDelay = min(max(0.0,rippleReconstructionDelay),doubleVectorPeriod);
samplePhase = doubleVectorPeriod;
rippleMeasurementPhase = samplePhase-rippleReconstructionDelay;
rippleAtMeasurement = double_vector_ripple_at_phase( ...
    deltaSwitchAtSample,uFirst,uSecond,uAverage,vectorStep,rippleMeasurementPhase);
rippleResidual = magnitudeGain*rippleAttenuation*rippleAtMeasurement;
iabcBaseLagged = iabcAmpCorrected-rippleResidual;

phaseCompensation = wel*basePhaseDelay;
cosPhaseComp = cos(phaseCompensation);
sinPhaseComp = sin(phaseCompensation);
phaseCompMatrix = [cosPhaseComp, -sinPhaseComp; sinPhaseComp, cosPhaseComp];
iabBeforePhaseComp = Tc*iabcBaseLagged;
iabAfterPhaseComp = phaseCompMatrix*iabBeforePhaseComp;
iabcAverageEstimated = TinvClarke*iabAfterPhaseComp;

if rippleReconstructionDelay > 0.0
    [deltaBase,deltaSwitch,~,~] = abc_current_increment( ...
        iabcAverageEstimated,Ad,Bd,fd,Bs_TI,wel,Ts,Tc,Rp,Tdq2abc);
    averageCurrentIncrement = (rippleReconstructionDelay/Ts)*( ...
        deltaBase+deltaSwitch*uAverage);
    iabcAverageEstimated = iabcAverageEstimated+averageCurrentIncrement;
end

[~,deltaSwitchAtSample,~,~] = abc_current_increment( ...
    iabcAverageEstimated,Ad,Bd,fd,Bs_TI,wel,Ts,Tc,Rp,Tdq2abc);
iabcRippleEstimated = double_vector_ripple_at_phase( ...
    deltaSwitchAtSample,uFirst,uSecond,uAverage,vectorStep,samplePhase);
iabcSwitchCorrectionRaw = ...
    (iabcAverageEstimated-iabcAmpCorrected)+iabcRippleEstimated;
iabcAtSample = iabcAverageEstimated+iabcRippleEstimated;

effectiveControllerDelay = min(controllerDelay,Ts);
[deltaBase,deltaSwitch,~,~] = abc_current_increment( ...
    iabcAtSample,Ad,Bd,fd,Bs_TI,wel,Ts,Tc,Rp,Tdq2abc);
iabcReconstruction = iabcAtSample+(effectiveControllerDelay/Ts)*( ...
    deltaBase+deltaSwitch*uSecond);

[deltaIabcBase,deltaIabcSwitch,idiqCompensated,Bs] = ...
    abc_current_increment(iabcReconstruction,Ad,Bd,fd,Bs_TI,wel,Ts, ...
    Tc,Rp,Tdq2abc);

if predictionValid
    predictionError = iabcReconstruction-iabcPredictedPrevious;
else
    predictionError = zeros(3,1);
end
reconstructionError = Iabc_Motor-iabcReconstruction;
predictionError_dq = Rp*(Tc*predictionError);
reconstructionError_dq = Rp*(Tc*reconstructionError);
end

function [deltaBase,deltaSwitch,idiqState,Bs] = abc_current_increment( ...
    iabc,Ad,Bd,fd,Bs_TI,wel,Ts,Tc,Rp,Tdq2abc)
Jrot = [0 -1; 1 0];
idiqState = Rp*(Tc*iabc);
Bs = Bd*Rp*Bs_TI;
deltaIdqBase = (Ad-eye(2))*idiqState+fd;
deltaBase = Tdq2abc*(deltaIdqBase+Jrot*idiqState*(wel*Ts));
deltaSwitch = Tdq2abc*Bs;
end

function rippleCurrent = double_vector_ripple_at_phase( ...
    deltaSwitchAtSample,uFirst,uSecond,uAverage,vectorStep,t)
doubleVectorPeriod = 2.0*vectorStep;
deltaFirstVectorStep = deltaSwitchAtSample*(uFirst-uAverage);
deltaSecondVectorStep = deltaSwitchAtSample*(uSecond-uAverage);
rippleAtPeriodStart = -0.5*deltaFirstVectorStep;
t = min(max(0.0,t),doubleVectorPeriod);
if t <= vectorStep
    rippleCurrent = rippleAtPeriodStart+(t/vectorStep)*deltaFirstVectorStep;
else
    rippleCurrent = rippleAtPeriodStart+deltaFirstVectorStep+ ...
        ((t-vectorStep)/vectorStep)*deltaSecondVectorStep;
end
end
