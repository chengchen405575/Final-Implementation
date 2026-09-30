function [iabcReconstruction,idiqCompensated, ...
    deltaIabcBase,deltaIabcSwitch,Bs, ...
    predictionError,reconstructionError, ...
    predictionError_dq,reconstructionError_dq, ...
    iabcAverageEstimated,iabcRippleEstimated,rippleResidual, ...
    iabcSwitchCorrectionRaw] = ...
    Current_Reconstruction_Switch_Frequency_kHz_Simplified( ...
    Ad,Bd,fd,Bs_TI,idiqFiltered,Sin_Cos_Theta,wel,Ts,U_applied, ...
    controllerDelay,iabcPredictedPrevious,Iabc_Motor,predictionValid, ...
    filterTimeConstant,sensorTimeConstant,filterDelayGain)
%#codegen
% Simplified reconstruction using one current-increment linearization.
% All internal current calculations remain in dq.  idiqFiltered is used as
% the single linearization state; conversion to abc occurs only for outputs.
% The interface intentionally matches Current_Reconstruction_Switch_Frequency_kHz
% so the two implementations can be exchanged directly for error evaluation.

Ad = double(Ad);
Bd = double(Bd);
fd = double(fd);
Bs_TI = double(Bs_TI);
idiqFiltered = double(idiqFiltered);
Sin_Cos_Theta = double(Sin_Cos_Theta);
wel = double(wel);
Ts = max(realmin('double'),double(Ts));
U_applied = double(U_applied);
controllerDelay = max(0.0,double(controllerDelay));
iabcPredictedPrevious = double(iabcPredictedPrevious);
Iabc_Motor = double(Iabc_Motor);
predictionValid = logical(predictionValid);
filterTimeConstant = max(0.0,double(filterTimeConstant));
sensorTimeConstant = max(0.0,double(sensorTimeConstant));
filterDelayGain = double(filterDelayGain); %#ok<NASGU> retained for interface compatibility

sinTheta = Sin_Cos_Theta(1);
cosTheta = Sin_Cos_Theta(2);
Rp = [cosTheta, sinTheta; -sinTheta, cosTheta];
TinvClarke = [1.0, 0.0; ...
             -0.5, sqrt(3)/2; ...
             -0.5, -sqrt(3)/2];
Tdq2abc = TinvClarke*Rp.';

uFirst = U_applied(1:3);
uSecond = U_applied(4:6);
uAverage = 0.5*(uFirst+uSecond);

% The only current-increment evaluation.  Later propagation deliberately
% reuses this local model instead of repeatedly re-linearizing reconstructed
% abc currents and transforming them back to dq.
Jrot = [0 -1; 1 0];
BsDq = Bd*Rp*Bs_TI;
Bs = BsDq;
deltaIdqBase = (Ad-eye(2))*idiqFiltered+fd+ ...
    Jrot*idiqFiltered*(wel*Ts);
deltaIdqSwitch = BsDq;

omegaFilter = wel*filterTimeConstant;
omegaSensor = wel*sensorTimeConstant;
magnitudeGain = sqrt(1.0+omegaFilter*omegaFilter)* ...
    sqrt(1.0+omegaSensor*omegaSensor);

w = abs(wel);
if w > 1.0e-6
    basePhaseDelay = (atan(w*filterTimeConstant)+ ...
        atan(w*sensorTimeConstant))/w;
else
    basePhaseDelay = filterTimeConstant+sensorTimeConstant;
end

vectorStep = Ts;
doubleVectorPeriod = 2.0*vectorStep;
omegaRipple = 2.0*pi*17.0e3;
omegaRippleFilter = omegaRipple*filterTimeConstant;
omegaRippleSensor = omegaRipple*sensorTimeConstant;
rippleAttenuation = 1.0/( ...
    sqrt(1.0+omegaRippleFilter*omegaRippleFilter)* ...
    sqrt(1.0+omegaRippleSensor*omegaRippleSensor));
rippleDelay = (atan(omegaRipple*filterTimeConstant)+ ...
    atan(omegaRipple*sensorTimeConstant))/omegaRipple;
rippleDelay = min(max(0.0,rippleDelay),doubleVectorPeriod);

rippleAtMeasurementDq = double_vector_ripple_at_phase( ...
    deltaIdqSwitch,uFirst,uSecond,uAverage,vectorStep, ...
    doubleVectorPeriod-rippleDelay);
rippleResidualDq = magnitudeGain*rippleAttenuation*rippleAtMeasurementDq;

phase = wel*basePhaseDelay;
phaseCompensation = [cos(phase), -sin(phase); ...
                     sin(phase),  cos(phase)];
idiqAverageEstimated = phaseCompensation*( ...
    magnitudeGain*idiqFiltered-rippleResidualDq);

% Reuse the increment frozen at idiqFiltered, as requested.
idiqAverageEstimated = idiqAverageEstimated+(rippleDelay/Ts)*( ...
    deltaIdqBase+deltaIdqSwitch*uAverage);
rippleAtSampleDq = double_vector_ripple_at_phase( ...
    deltaIdqSwitch,uFirst,uSecond,uAverage,vectorStep,doubleVectorPeriod);
idiqAtSample = idiqAverageEstimated+rippleAtSampleDq;

delayRatio = min(controllerDelay,Ts)/Ts;
idiqCompensated = idiqAtSample+delayRatio*( ...
    deltaIdqBase+deltaIdqSwitch*uSecond);

% Convert to abc once, at the output boundary.
iabcReconstruction = Tdq2abc*idiqCompensated;
deltaIabcBase = Tdq2abc*deltaIdqBase;
deltaIabcSwitch = Tdq2abc*deltaIdqSwitch;
iabcAverageEstimated = Tdq2abc*idiqAverageEstimated;
iabcRippleEstimated = Tdq2abc*rippleAtSampleDq;
rippleResidual = Tdq2abc*rippleResidualDq;

% Diagnostics are evaluated only after the final dq-domain reconstruction.
% They therefore preserve the original interface without introducing extra
% abc-to-dq round trips into the reconstruction algorithm itself.
iabcAmpCorrected = Tdq2abc*(magnitudeGain*idiqFiltered);
iabcSwitchCorrectionRaw = ...
    (iabcAverageEstimated-iabcAmpCorrected)+iabcRippleEstimated;
if predictionValid
    predictionError = iabcReconstruction-iabcPredictedPrevious;
else
    predictionError = zeros(3,1);
end
reconstructionError = Iabc_Motor-iabcReconstruction;
predictionError_dq = Rp*clarke_transform(predictionError);
reconstructionError_dq = Rp*clarke_transform(reconstructionError);
end

function iab = clarke_transform(iabc)
iab = [2/3, -1/3, -1/3; ...
       0, 1/sqrt(3), -1/sqrt(3)]*iabc;
end

function rippleCurrent = double_vector_ripple_at_phase( ...
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
