function [idiqEstimated, innovation, idiqFilteredEstimated, estimateValid] = ...
    Ripple_ReconstructionDQ(idiqFiltered,Ad,Bd,fd,Bs_TI, ...
    Sin_Cos_Theta,theta_el,wel,Ts,U_applied,controllerDelay, ...
    filterTimeConstant,sensorTimeConstant,phaseShift_rad)
%#codegen
% Legacy branch of the Current Reconstruction Variant Subsystem.
% This is the phase/magnitude compensation and switching-ripple
% reconstruction extracted from Assembly_QP. Output is dq current at the
% actuation instant, matching the observer branch and Assembly_QP input.
% observerGain and reset are unused but retained for an identical Variant
% interface. theta_el is retained for interface/timing documentation.

idiqFiltered = double(idiqFiltered);
Ad = double(Ad); Bd = double(Bd); fd = double(fd);
Bs_TI = double(Bs_TI);
Sin_Cos_Theta = double(Sin_Cos_Theta);
theta_el = double(theta_el); %#ok<NASGU>
wel = double(wel); Ts = double(Ts);
U_applied = double(U_applied);
controllerDelay = max(0.0,double(controllerDelay));
filterTimeConstant = max(0.0,double(filterTimeConstant));
sensorTimeConstant = max(0.0,double(sensorTimeConstant));
phaseShift_rad = double(phaseShift_rad);

sinTheta = Sin_Cos_Theta(1);
cosTheta = Sin_Cos_Theta(2);
Tc = [2/3,-1/3,-1/3;0,1/sqrt(3),-1/sqrt(3)];
Rp = [cosTheta,sinTheta;-sinTheta,cosTheta];
TinvClarke = [1,0;-0.5,sqrt(3)/2;-0.5,-sqrt(3)/2];
Tdq2abc = TinvClarke*Rp.';
uFirst = U_applied(1:3);
uSecond = U_applied(4:6);
uAverage = 0.5*(uFirst+uSecond);

if abs(wel) > 1.0e-6
    filterEquivalentDelay = abs(phaseShift_rad)/abs(wel);
else
    filterEquivalentDelay = filterTimeConstant+sensorTimeConstant;
end
gain = sqrt(1+(wel*filterTimeConstant)^2)* ...
       sqrt(1+(wel*sensorTimeConstant)^2);
iabcAverage = gain*(Tdq2abc*idiqFiltered);

remainingDelay = min(filterEquivalentDelay+controllerDelay,8.0*Ts);
for delayStep = 1:8
    if remainingDelay <= 0.0
        break;
    end
    fraction = min(1.0,remainingDelay/Ts);
    [deltaBase,deltaSwitch] = abc_increment( ...
        iabcAverage,Ad,Bd,fd,Bs_TI,wel,Ts,Tc,Rp,Tdq2abc);
    iabcAverage = iabcAverage+fraction*(deltaBase+deltaSwitch*uAverage);
    remainingDelay = remainingDelay-fraction*Ts;
end

[~,deltaSwitchRipple] = abc_increment( ...
    iabcAverage,Ad,Bd,fd,Bs_TI,wel,Ts,Tc,Rp,Tdq2abc);
iabcRipple = -0.5*deltaSwitchRipple*(uFirst-uAverage);
externalFraction = min(1.0,controllerDelay/Ts);
iabcRipple = iabcRipple+externalFraction* ...
    deltaSwitchRipple*(uSecond-uAverage);
iabcEstimated = iabcAverage+iabcRipple;
idiqEstimated = Rp*(Tc*iabcEstimated);

% Common diagnostic ports. The legacy branch has no dynamic filter-state
% prediction, so its best same-domain estimate is the measured filtered dq.
innovation = zeros(2,1);
idiqFilteredEstimated = idiqFiltered;
estimateValid = true;
end

function [deltaBase,deltaSwitch] = abc_increment( ...
    iabcState,Ad,Bd,fd,Bs_TI,wel,Ts,Tc,Rp,Tdq2abc)
Jrot = [0,-1;1,0];
idiqState = Rp*(Tc*iabcState);
Bs = Bd*Rp*Bs_TI;
deltaBase = Tdq2abc*((Ad-eye(2))*idiqState+fd+ ...
    Jrot*idiqState*(wel*Ts));
deltaSwitch = Tdq2abc*Bs;
end

