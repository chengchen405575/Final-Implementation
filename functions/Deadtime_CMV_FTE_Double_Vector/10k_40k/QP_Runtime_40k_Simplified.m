function [U_unc,iabcReconstruction, ...
    deltaIabcBase,deltaIabcSwitch, ...
    predictionError,reconstructionError, ...
    predictionError_dq,reconstructionError_dq, ...
    iabcAverageEstimated,iabcRippleEstimated,rippleResidual, ...
    iabcSwitchCorrectionRaw] = ...
    QP_Runtime_40k_Simplified( ...
    H,Gamma,Phi,Lambdafsw_SE,Upsil,Bd,Bs_TI, ...
    deltaIdqBaseOffset,deltaIdqState, ...
    magnitudeGain,phaseCompensation,rippleAttenuation,rippleDelay, ...
    Sin_Cos_Theta,idiqFiltered,idiq_ref,Ts,U_applied,controllerDelay, ...
    iabcPredictedPrevious,Iabc_Motor,predictionValid)
%#codegen
% Fast-rate current reconstruction, diagnostics, and QP linear solve.
% H/Upsil remain frozen at the 10-kHz midpoint angle. The instantaneous
% current transforms and switching-current increment use the current
% 40-kHz electrical angle so their error does not scale with load current.

Ts = max(realmin('double'),double(Ts));
idiqFiltered = double(idiqFiltered);
idiq_ref = double(idiq_ref);
U_applied = double(U_applied);
Bd = double(Bd);
Bs_TI = double(Bs_TI);
Sin_Cos_Theta = double(Sin_Cos_Theta);
iabcPredictedPrevious = double(iabcPredictedPrevious);
Iabc_Motor = double(Iabc_Motor);
predictionValid = logical(predictionValid);
uFirst = U_applied(1:3);
uSecond = U_applied(4:6);
uAverage = 0.5*(uFirst+uSecond);

sinTheta = Sin_Cos_Theta(1);
cosTheta = Sin_Cos_Theta(2);
Rp = [cosTheta, sinTheta; -sinTheta, cosTheta];
TinvClarke = [1.0, 0.0; ...
             -0.5, sqrt(3)/2; ...
             -0.5, -sqrt(3)/2];
Tclarke = [2/3, -1/3, -1/3; ...
           0, 1/sqrt(3), -1/sqrt(3)];
Tdq2abc = TinvClarke*Rp.';
Tabc2dq = Rp*Tclarke;
deltaIdqSwitch = Bd*Rp*Bs_TI;

% One frozen current-increment model, linearized at idiqFiltered.
deltaIdqBase = deltaIdqState*idiqFiltered+deltaIdqBaseOffset;
period = 2.0*Ts;
rippleAtMeasurementDq = ripple_at_phase( ...
    deltaIdqSwitch,uFirst,uSecond,uAverage, ...
    Ts,period-rippleDelay);
rippleResidualDq = ...
    magnitudeGain*rippleAttenuation*rippleAtMeasurementDq;
idiqAverage = phaseCompensation*( ...
    magnitudeGain*idiqFiltered-rippleResidualDq);
idiqAverage = idiqAverage+(rippleDelay/Ts)*( ...
    deltaIdqBase+deltaIdqSwitch*uAverage);

rippleNow = ripple_at_phase( ...
    deltaIdqSwitch,uFirst,uSecond,uAverage,Ts,period);
idiqAtSample = idiqAverage+rippleNow;
delayRatio = min(max(0.0,double(controllerDelay)),Ts)/Ts;
idiqCompensated = idiqAtSample+delayRatio*( ...
    deltaIdqBase+deltaIdqSwitch*uSecond);

% The only dq-to-abc conversion point in the runtime data path.
iabcReconstruction = Tdq2abc*idiqCompensated;
deltaIabcBase = Tdq2abc*deltaIdqBase;
deltaIabcSwitch = Tdq2abc*deltaIdqSwitch;

% Diagnostic ports match Current_Reconstruction_Switch_Frequency_kHz_Simplified.
iabcAverageEstimated = Tdq2abc*idiqAverage;
iabcRippleEstimated = Tdq2abc*rippleNow;
rippleResidual = Tdq2abc*rippleResidualDq;
iabcAmpCorrected = Tdq2abc*(magnitudeGain*idiqFiltered);
iabcSwitchCorrectionRaw = ...
    (iabcAverageEstimated-iabcAmpCorrected)+iabcRippleEstimated;
if predictionValid
    predictionError = iabcReconstruction-iabcPredictedPrevious;
else
    predictionError = zeros(3,1);
end
reconstructionError = Iabc_Motor-iabcReconstruction;
predictionError_dq = Tabc2dq*predictionError;
reconstructionError_dq = Tabc2dq*reconstructionError;

Np = 5;
r = Gamma*idiqCompensated+Phi-repmat(idiq_ref,Np,1);
F = Upsil.'*r-Lambdafsw_SE*uSecond;
y = H.'\(-F);
U_unc = H\y;
end

function rippleCurrent = ripple_at_phase( ...
    deltaSwitch,uFirst,uSecond,uAverage,vectorStep,t)
period = 2.0*vectorStep;
deltaFirst = deltaSwitch*(uFirst-uAverage);
deltaSecond = deltaSwitch*(uSecond-uAverage);
rippleStart = -0.5*deltaFirst;
t = min(max(0.0,t),period);
if t <= vectorStep
    rippleCurrent = rippleStart+(t/vectorStep)*deltaFirst;
else
    rippleCurrent = rippleStart+deltaFirst+ ...
        ((t-vectorStep)/vectorStep)*deltaSecond;
end
end
