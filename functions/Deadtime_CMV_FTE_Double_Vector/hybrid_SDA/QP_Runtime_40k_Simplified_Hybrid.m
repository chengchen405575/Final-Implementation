function [U_unc_exp,U_unc_emb,iabcReconstruction, ...
    deltaIabcBase,deltaIabcSwitch, ...
    predictionError,reconstructionError, ...
    predictionError_dq,reconstructionError_dq, ...
    iabcAverageEstimated,iabcRippleEstimated,rippleResidual, ...
    iabcSwitchCorrectionRaw] = ...
    QP_Runtime_40k_Simplified_Hybrid( ...
    H_exp,H_emb,Gamma,Phi,Lambdafsw_SE,Upsil,Bd,Bs_TI, ...
    deltaIdqBaseOffset,deltaIdqState,magnitudeGain,phaseCompensation, ...
    rippleAttenuation,rippleDelay,Sin_Cos_Theta,idiqFiltered, ...
    idiq_ref,lambdaZeroEmbedded,Ts,U_applied,controllerDelay, ...
    iabcPredictedPrevious,Iabc_Motor,predictionValid)
%#codegen
%QP_RUNTIME_40K_SIMPLIFIED_HYBRID Standalone hybrid runtime calculation.
% Current reconstruction is evaluated once. The two matched unconstrained
% centers are then formed using their respective factors and linear terms.

Ts = max(realmin('double'),double(Ts));
idiqFiltered = double(idiqFiltered); idiq_ref = double(idiq_ref);
U_applied = double(U_applied);
lambdaZeroEmbedded = max(0.0,double(lambdaZeroEmbedded));
Bd = double(Bd); Bs_TI = double(Bs_TI);
Sin_Cos_Theta = double(Sin_Cos_Theta);
iabcPredictedPrevious = double(iabcPredictedPrevious);
Iabc_Motor = double(Iabc_Motor);
predictionValid = logical(predictionValid);
uFirst = U_applied(1:3); uSecond = U_applied(4:6);
uAverage = 0.5*(uFirst+uSecond);

sinTheta = Sin_Cos_Theta(1); cosTheta = Sin_Cos_Theta(2);
Rp = [cosTheta, sinTheta; -sinTheta, cosTheta];
TinvClarke = [1.0, 0.0; -0.5, sqrt(3)/2; -0.5, -sqrt(3)/2];
Tclarke = [2/3, -1/3, -1/3; 0, 1/sqrt(3), -1/sqrt(3)];
Tdq2abc = TinvClarke*Rp.'; Tabc2dq = Rp*Tclarke;
deltaIdqSwitch = Bd*Rp*Bs_TI;

deltaIdqBase = deltaIdqState*idiqFiltered+deltaIdqBaseOffset;
period = 2.0*Ts;
rippleAtMeasurementDq = ripple_at_phase( ...
    deltaIdqSwitch,uFirst,uSecond,uAverage,Ts,period-rippleDelay);
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

iabcReconstruction = Tdq2abc*idiqCompensated;
deltaIabcBase = Tdq2abc*deltaIdqBase;
deltaIabcSwitch = Tdq2abc*deltaIdqSwitch;
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
F_exp = Upsil.'*r-Lambdafsw_SE*uSecond;
FzeroEmbedded = -0.75*lambdaZeroEmbedded*ones(3*Np,1);
F_emb = F_exp+FzeroEmbedded;
y_exp = H_exp.'\(-F_exp);
U_unc_exp = H_exp\y_exp;
y_emb = H_emb.'\(-F_emb);
U_unc_emb = H_emb\y_emb;
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
