function [iabcReconstruction,idiqCompensated, ...
    deltaIabcBase,deltaIabcSwitch,Bs, ...
    predictionError,reconstructionError, ...
    predictionError_dq,reconstructionError_dq, ...
    iabcAverageEstimated,iabcRippleEstimated,rippleResidual, ...
    iabcSwitchCorrectionRaw] = ...
    Current_Reconstruction_hybrid( ...
    Ad,Bd,fd,Bs_TI,idiqFiltered, ...
    Sin_Cos_Theta,wel,Ts,U_applied,controllerDelay, ...
    iabcPredictedPrevious,Iabc_Motor,predictionValid, ...
    filterTimeConstant,sensorTimeConstant,filterDelayGain)
%CURRENT_RECONSTRUCTION_HYBRID Fundamental inverse plus ripple reconstruction.
%#codegen
%
% The two filter effects are explicitly separated:
%   1) basePhaseDelay describes the fundamental phase lag at abs(wel),
%   2) rippleReconstructionDelay locates the delayed switching ripple,
%   3) the filtered ripple residual is removed from the measurement,
%   4) the fundamental is restored by the complex inverse filter response,
%   5) the switching ripple is restored at the sampling boundary.
%
% filterDelayGain scales only the theoretical fundamental-frequency filter
% delay. It does not scale the switching-ripple amplitude.
%
% Input dimensions:
%   Ad, Bd                    2x2
%   fd                        2x1
%   Bs_TI                     2x3
%   idiqFiltered              2x1
%   Sin_Cos_Theta             2x1 = [sin(theta); cos(theta)]
%   U_applied                 6x1 = [uFirst; uSecond]
%   iabcPredictedPrevious     3x1
%   Iabc_Motor                3x1 (simulation/reference measurement only)
%   all remaining inputs      scalar
%
% Output dimensions:
%   iabcReconstruction        3x1
%   idiqCompensated           2x1
%   deltaIabcBase             3x1
%   deltaIabcSwitch           3x3
%   Bs                        2x3
%   predictionError           3x1 = reconstruction - previous prediction
%   reconstructionError       3x1 = motor current - reconstruction
%   predictionError_dq        2x1
%   reconstructionError_dq    2x1
%   iabcAverageEstimated      3x1
%   iabcRippleEstimated       3x1
%   rippleResidual            3x1
%   iabcSwitchCorrectionRaw   3x1

Ad = double(Ad);
Bd = double(Bd);
fd = double(fd);
Bs_TI = double(Bs_TI);
idiqFiltered = double(idiqFiltered);
Sin_Cos_Theta = double(Sin_Cos_Theta);
wel = double(wel);
Ts = max(realmin('double'),double(Ts));
% IMPORTANT TIME-BASE DEFINITION:
% Ts is the duration of ONE SDA switching-vector step. The Simulink model
% supplies Ts = 0.5*MPC_Ts. One complete applied double-vector sequence
% [uFirst; uSecond] therefore lasts 2*Ts.
vectorStep = Ts;
doubleVectorPeriod = 2.0*vectorStep;
U_applied = double(U_applied);
controllerDelay = max(0.0,double(controllerDelay));
iabcPredictedPrevious = double(iabcPredictedPrevious);
Iabc_Motor = double(Iabc_Motor);
predictionValid = logical(predictionValid);
filterTimeConstant = max(0.0,double(filterTimeConstant));
sensorTimeConstant = max(0.0,double(sensorTimeConstant));
filterDelayGain = max(0.0,double(filterDelayGain));

sinTheta = Sin_Cos_Theta(1);
cosTheta = Sin_Cos_Theta(2);
Tc = [2/3, -1/3, -1/3; ...
       0, 1/sqrt(3), -1/sqrt(3)];
Rp = [cosTheta, sinTheta; -sinTheta, cosTheta];
TinvClarke = [1.0, 0.0; ...
      -0.5, sqrt(3)/2; ...
      -0.5, -sqrt(3)/2];
Tdq2abc = TinvClarke*Rp.';

uFirst = U_applied(1:3);
uSecond = U_applied(4:6);
uAverage = 0.5*(uFirst+uSecond);

% Fundamental inverse-filter magnitude and phase at electrical frequency.
omegaFilter = wel*filterTimeConstant;
omegaSensor = wel*sensorTimeConstant;
magnitudeGain = sqrt(1.0+omegaFilter*omegaFilter)*sqrt(1.0+omegaSensor*omegaSensor);
iabcAmpCorrected = magnitudeGain*(Tdq2abc*idiqFiltered);

% basePhaseDelay belongs to the fundamental component. The empirical
% rippleReconstructionDelay is scheduled from it but is used only to locate
% the switching ripple contained in the filtered measurement.
w = abs(wel);
if w > 1.0e-6
    basePhaseDelay = (atan(w*filterTimeConstant)+ ...
        atan(w*sensorTimeConstant))/w;
else
    basePhaseDelay = filterTimeConstant+sensorTimeConstant;
end
rippleReconstructionDelay = filterDelayGain*basePhaseDelay;
rippleReconstructionDelay = min(max(0.0,rippleReconstructionDelay),doubleVectorPeriod);

% The filtered measurement still contains an attenuated switching ripple
% from the delayed phase. Remove that residual before treating the signal as
% the filtered base current. Because iabcAmpCorrected has already been
% scaled by magnitudeGain, apply the same scale to the residual ripple.
[~,deltaSwitchAtSample,~,~] = abc_current_increment( ...
    iabcAmpCorrected,Ad,Bd,fd,Bs_TI,wel,Ts,Tc,Rp,Tdq2abc);
% Fundamental repetition frequency of the complete two-vector sequence.
omegaSwitch = 2*pi/doubleVectorPeriod;
omegaRipple = max(1.0,omegaSwitch-abs(wel));
omegaRippleFilter = omegaRipple*filterTimeConstant;
omegaRippleSensor = omegaRipple*sensorTimeConstant;
rippleAttenuation = 1.0/( ...
    sqrt(1.0+omegaRippleFilter*omegaRippleFilter)* ...
    sqrt(1.0+omegaRippleSensor*omegaRippleSensor));
samplePhase = doubleVectorPeriod;
rippleMeasurementPhase = samplePhase-rippleReconstructionDelay;
rippleAtMeasurement = double_vector_ripple_at_phase( ...
    deltaSwitchAtSample,uFirst,uSecond,uAverage,vectorStep, ...
    rippleMeasurementPhase);

% Residual switching ripple remaining in the amplitude-corrected measurement.
rippleResidual = magnitudeGain*rippleAttenuation*rippleAtMeasurement;

% Fundamental with correct magnitude but still carrying the filter phase lag.
iabcBaseLagged = iabcAmpCorrected-rippleResidual;

% Explicit fundamental phase compensation. Rotate the stationary
% alpha-beta current vector by the signed filter phase error.
phaseCompensation = wel*basePhaseDelay;
cosPhaseComp = cos(phaseCompensation);
sinPhaseComp = sin(phaseCompensation);
phaseCompMatrix = [cosPhaseComp, -sinPhaseComp; ...
                   sinPhaseComp,  cosPhaseComp];
iabBeforePhaseComp = Tc*iabcBaseLagged;
iabAfterPhaseComp = phaseCompMatrix*iabBeforePhaseComp;
iabcAverageEstimated = TinvClarke*iabAfterPhaseComp;

% Restore the average current evolution over the ripple reconstruction
% interval. This is the state-space increment produced by the motor dynamics
% and the average of the two actually applied switching states; it is kept
% separate from the fundamental inverse-filter phase correction above.
if rippleReconstructionDelay > 0.0
    [deltaBase,deltaSwitch,~,~] = abc_current_increment( ...
        iabcAverageEstimated,Ad,Bd,fd,Bs_TI,wel,Ts,Tc,Rp,Tdq2abc);
    averageCurrentIncrement = (rippleReconstructionDelay/Ts)*( ...
        deltaBase+deltaSwitch*uAverage);
    iabcAverageEstimated = ...
        iabcAverageEstimated+averageCurrentIncrement;
end

% Re-evaluate the sensitivity at the reconstructed average and restore the
% piecewise triangular ripple at the present sampling boundary.
[~,deltaSwitchAtSample,~,~] = abc_current_increment( ...
    iabcAverageEstimated,Ad,Bd,fd,Bs_TI,wel,Ts,Tc,Rp,Tdq2abc);
iabcRippleEstimated = double_vector_ripple_at_phase( ...
    deltaSwitchAtSample,uFirst,uSecond,uAverage,vectorStep,samplePhase);

% Raw diagnostic correction relative to the magnitude-corrected measurement.
iabcSwitchCorrectionRaw = ...
    (iabcAverageEstimated-iabcAmpCorrected)+iabcRippleEstimated;
iabcAtSample = iabcAverageEstimated+iabcRippleEstimated;

% External controller delay is separate from the filter delay. The final
% applied vector remains active until the new command becomes effective.
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
% One-vector-step PMSM current increment in abc coordinates. In this model,
% Ts = 0.5*MPC_Ts and Ad, Bd and fd are discretized with the same step.
Jrot = [0 -1; 1 0];
idiqState = Rp*(Tc*iabc);
Bs = Bd*Rp*Bs_TI;
deltaIdqBase = (Ad-eye(2))*idiqState+fd;
deltaBase = Tdq2abc*(deltaIdqBase+Jrot*idiqState*(wel*Ts));
deltaSwitch = Tdq2abc*Bs;
end

function rippleCurrent = double_vector_ripple_at_phase( ...
    deltaSwitchAtSample,uFirst,uSecond,uAverage,vectorStep,t)
% Zero-mean triangular ripple over [uFirst,uSecond]. deltaSwitchAtSample
% is already the switching-current increment for one vectorStep
% (= 0.5*MPC_Ts), so no additional factor of 0.5 or 2 is required.
doubleVectorPeriod = 2.0*vectorStep;
deltaFirstVectorStep = ...
    deltaSwitchAtSample*(uFirst-uAverage);
deltaSecondVectorStep = ...
    deltaSwitchAtSample*(uSecond-uAverage);
rippleAtPeriodStart = -0.5*deltaFirstVectorStep;
t = min(max(0.0,t),doubleVectorPeriod);
if t <= vectorStep
    rippleCurrent = rippleAtPeriodStart+ ...
        (t/vectorStep)*deltaFirstVectorStep;
else
    rippleCurrent = rippleAtPeriodStart+deltaFirstVectorStep+ ...
        ((t-vectorStep)/vectorStep)*deltaSecondVectorStep;
end
end
