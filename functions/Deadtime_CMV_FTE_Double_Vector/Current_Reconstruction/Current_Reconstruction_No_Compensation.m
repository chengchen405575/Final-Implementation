function [iabcReconstruction,idiqCompensated, ...
    deltaIabcBase,deltaIabcSwitch,Bs, ...
    predictionError,reconstructionError, ...
    predictionError_dq,reconstructionError_dq, ...
    iabcAverageEstimated,iabcRippleEstimated,rippleResidual, ...
    iabcSwitchCorrectionRaw] = ...
    Current_Reconstruction_No_Compensation( ...
    Ad,Bd,fd,Bs_TI,idiqFiltered, ...
    Sin_Cos_Theta,wel,Ts,U_applied,controllerDelay, ...
    iabcPredictedPrevious,Iabc_Motor,predictionValid, ...
    filterTimeConstant,sensorTimeConstant,filterDelayGain) %#ok<INUSD>
%CURRENT_RECONSTRUCTION_NO_COMPENSATION Uncompensated diagnostic baseline.
%#codegen
% The 16-input/13-output interface exactly matches
% Current_Reconstruction_Switch_Frequency_kHz and its simplified variant.
% idiqFiltered is passed through directly; abc current is obtained only by
% inverse Park and inverse Clarke transforms. No filter, ripple, or
% controller-delay compensation is applied.
% U_applied, controllerDelay,
% filterTimeConstant, sensorTimeConstant, and filterDelayGain are retained
% for interface compatibility and are intentionally unused.
% Prediction error uses the same validity gate and dq transform as hybrid.
% Ripple/correction diagnostics are disabled (zero).
% iabcAverageEstimated is the uncorrected transformed measurement.

Ad = double(Ad);
Bd = double(Bd);
fd = double(fd);
Bs_TI = double(Bs_TI);
idiqFiltered = double(idiqFiltered);
Sin_Cos_Theta = double(Sin_Cos_Theta);
wel = double(wel);
Ts = max(realmin('double'),double(Ts));
iabcPredictedPrevious = double(iabcPredictedPrevious);
Iabc_Motor = double(Iabc_Motor);
predictionValid = logical(predictionValid);
% Ts is the SDA vector-step duration supplied as 0.5*MPC_Ts. The discrete
% matrices and the increments exported to SDA use this same time base.

sinTheta = Sin_Cos_Theta(1);
cosTheta = Sin_Cos_Theta(2);
Rpark = [cosTheta, sinTheta; -sinTheta, cosTheta];
TinvClarke = [1.0, 0.0; ...
             -0.5, sqrt(3)/2; ...
             -0.5, -sqrt(3)/2];
Tdq2abc = TinvClarke*Rpark.';

idiqCompensated = idiqFiltered;
idiqState = idiqFiltered;
iabcReconstruction = Tdq2abc*idiqState;

% Preserve the original one-vector-step current-increment model, evaluated directly
% at the filtered dq current without a redundant abc-to-dq round trip.
Jrot = [0 -1; 1 0];
Bs = Bd*Rpark*Bs_TI;
deltaIdqBase = (Ad-eye(2))*idiqState+fd;
deltaIabcBase = Tdq2abc*(deltaIdqBase+Jrot*idiqState*(wel*Ts));
deltaIabcSwitch = Tdq2abc*Bs;

if predictionValid
    predictionError = iabcReconstruction-iabcPredictedPrevious;
else
    predictionError = zeros(3,1);
end
reconstructionError = Iabc_Motor-iabcReconstruction;
Tclarke = [2/3, -1/3, -1/3; ...
           0, 1/sqrt(3), -1/sqrt(3)];
predictionError_dq = Rpark*(Tclarke*predictionError);
reconstructionError_dq = Rpark*(Tclarke*reconstructionError);
iabcAverageEstimated = iabcReconstruction;
iabcRippleEstimated = zeros(3,1);
rippleResidual = zeros(3,1);
iabcSwitchCorrectionRaw = zeros(3,1);
end
