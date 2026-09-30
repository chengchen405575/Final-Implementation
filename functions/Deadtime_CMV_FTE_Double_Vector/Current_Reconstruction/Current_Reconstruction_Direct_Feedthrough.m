function [iabcReconstruction,idiqCompensated, ...
    deltaIabcBase,deltaIabcSwitch,Bs, ...
    predictionError,reconstructionError, ...
    predictionError_dq,reconstructionError_dq, ...
    iabcAverageEstimated,iabcRippleEstimated,rippleResidual, ...
    iabcSwitchCorrectionRaw] = ...
    Current_Reconstruction_Direct_Feedthrough( ...
    Ad,Bd,fd,Bs_TI,idiqFiltered, ...
    Sin_Cos_Theta,wel,Ts,U_applied,controllerDelay, ...
    iabcPredictedPrevious,Iabc_Motor,predictionValid, ...
    filterTimeConstant,sensorTimeConstant,filterDelayGain) %#ok<INUSD>
%CURRENT_RECONSTRUCTION_DIRECT_FEEDTHROUGH Ideal-current baseline for SDA.
%#codegen
%
% This variant bypasses the sensor/filter reconstruction and feeds the
% simulated motor phase current directly to SDA:
%   iabcReconstruction = Iabc_Motor.
% It isolates SDA prediction/search performance from reconstruction error.
%
% IMPORTANT: Iabc_Motor is available in simulation but is not an
% independently measurable signal in the real controller. This variant is
% therefore intended for benchmarking and diagnosis, not deployment.
%
% Ts is the duration of one SDA vector step and equals 0.5*MPC_Ts in the
% Simulink model. Ad, Bd and fd must be discretized with this same Ts.
%
% Input dimensions:
%   Ad, Bd                    2x2
%   fd                        2x1
%   Bs_TI                     2x3
%   idiqFiltered              2x1 (unused; retained for variant interface)
%   Sin_Cos_Theta             2x1 = [sin(theta); cos(theta)]
%   U_applied                 6x1 (unused; retained for variant interface)
%   iabcPredictedPrevious     3x1
%   Iabc_Motor                3x1
%   all remaining inputs      scalar
%
% Output dimensions:
%   iabcReconstruction        3x1
%   idiqCompensated           2x1
%   deltaIabcBase             3x1
%   deltaIabcSwitch           3x3
%   Bs                        2x3
%   predictionError           3x1
%   reconstructionError       3x1
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
Sin_Cos_Theta = double(Sin_Cos_Theta);
wel = double(wel);
Ts = max(realmin('double'),double(Ts));
iabcPredictedPrevious = double(iabcPredictedPrevious);
Iabc_Motor = double(Iabc_Motor);

% idiqFiltered, U_applied, controllerDelay, filterTimeConstant,
% sensorTimeConstant and filterDelayGain are intentionally unused. They are
% retained at their original positions so this diagnostic baseline can be
% selected without changing any MATLAB Function block wiring.

sinTheta = Sin_Cos_Theta(1);
cosTheta = Sin_Cos_Theta(2);
Tclarke = [2/3, -1/3, -1/3; ...
           0, 1/sqrt(3), -1/sqrt(3)];
Rpark = [cosTheta, sinTheta; -sinTheta, cosTheta];
TinvClarke = [1.0, 0.0; ...
             -0.5, sqrt(3)/2; ...
             -0.5, -sqrt(3)/2];
Tdq2abc = TinvClarke*Rpark.';

% Ideal current path used by SDA.
iabcReconstruction = Iabc_Motor;
idiqCompensated = Rpark*(Tclarke*iabcReconstruction);

% One-vector-step current increments used by Assembly_QP and SDA_Solve.
Jrot = [0 -1; 1 0];
Bs = Bd*Rpark*Bs_TI;
deltaIdqBase = (Ad-eye(2))*idiqCompensated+fd;
deltaIabcBase = Tdq2abc*( ...
    deltaIdqBase+Jrot*idiqCompensated*(wel*Ts));
deltaIabcSwitch = Tdq2abc*Bs;

% Prediction error remains observable for assessment of the SDA/model
% prediction made in the preceding vector step.
if logical(predictionValid)
    predictionError = iabcReconstruction-iabcPredictedPrevious;
else
    predictionError = zeros(3,1);
end
predictionError_dq = Rpark*(Tclarke*predictionError);

% Direct feedthrough is, by definition, identical to the simulation truth.
reconstructionError = Iabc_Motor-iabcReconstruction;
reconstructionError_dq = Rpark*(Tclarke*reconstructionError);

% Diagnostics that belong to filter/ripple reconstruction are disabled.
iabcAverageEstimated = iabcReconstruction;
iabcRippleEstimated = zeros(3,1);
rippleResidual = zeros(3,1);
iabcSwitchCorrectionRaw = zeros(3,1);
end
