function [idiqEstimated, innovation, idiqFilteredEstimated, estimateValid] = ...
    CurrentObserverDQ(idiqFiltered,Ad,Bd,fd,Bs_TI, ...
    Sin_Cos_Theta,theta_el,wel,Ts,U_applied,controllerDelay, ...
    filterTimeConstant,sensorTimeConstant,phaseShift_rad,observerGain,reset)
%#codegen
% Augmented-observer branch of the Current Reconstruction Variant.
% State = [motor dq; sensor-filter dq; digital-filter dq]. The actual
% filters precede the Park transform, so filter propagation is performed in
% stationary alpha-beta coordinates and returned to dq at each vector edge.
% U_applied contains the two actual states applied between the previous and
% current measurements. Output includes external-delay propagation and is
% therefore at the same actuation instant as LegacyRippleCompensatorDQ.
% phaseShift_rad is unused and exists only for the common Variant interface.

persistent xPosterior initialized
if isempty(initialized)
    initialized = false;
    xPosterior = zeros(6,1);
end

y = double(idiqFiltered);
Ad = double(Ad); Bd = double(Bd); fd = double(fd);
Bs_TI = double(Bs_TI);
Sin_Cos_Theta = double(Sin_Cos_Theta);
theta_el = double(theta_el); wel = double(wel); Ts = double(Ts);
U_applied = double(U_applied);
controllerDelay = max(0.0,double(controllerDelay));
filterTimeConstant = max(0.0,double(filterTimeConstant));
sensorTimeConstant = max(0.0,double(sensorTimeConstant));
phaseShift_rad = double(phaseShift_rad); %#ok<NASGU>
K = double(observerGain);
reset = logical(reset);

assert(Ts > 0.0);
assert(all(size(K) == [6,2]));

if reset || ~initialized
    % At initialization the internal filter states are known from the
    % measurement, while the unfiltered current starts from the same value.
    xPosterior = [y;y;y];
    innovation = zeros(2,1);
    estimateValid = false;
    initialized = true;
else
    theta0 = theta_el-2.0*wel*Ts;
    theta1 = theta_el-wel*Ts;
    theta2 = theta_el;
    xPrior = propagate_half(xPosterior,Ad,Bd,fd,Bs_TI, ...
        U_applied(1:3),theta0,theta1,Ts, ...
        sensorTimeConstant,filterTimeConstant);
    xPrior = propagate_half(xPrior,Ad,Bd,fd,Bs_TI, ...
        U_applied(4:6),theta1,theta2,Ts, ...
        sensorTimeConstant,filterTimeConstant);
    innovation = y-xPrior(5:6);
    xPosterior = xPrior+K*innovation;
    estimateValid = true;
end

idiqFilteredEstimated = xPosterior(5:6);
idiqEstimated = propagate_external_delay(xPosterior(1:2), ...
    Ad,Bd,fd,Bs_TI,Sin_Cos_Theta,wel,Ts,U_applied(4:6), ...
    controllerDelay);
end

function xNext = propagate_half(x,Ad,Bd,fd,Bs_TI,u, ...
    thetaStart,thetaEnd,Ts,sensorT,filterT)
thetaMid = 0.5*(thetaStart+thetaEnd);
Rstart = park_matrix(thetaStart);
Rmid = park_matrix(thetaMid);
Rend = park_matrix(thetaEnd);

iMotor0 = x(1:2);
iMotor1 = Ad*iMotor0+Bd*(Rmid*Bs_TI*u)+fd;
iMotorABavg = 0.5*(Rstart.'*iMotor0+Rend.'*iMotor1);

iSensorAB0 = Rstart.'*x(3:4);
if sensorT > 0.0
    aSensor = exp(-Ts/sensorT);
    iSensorAB1 = aSensor*iSensorAB0+(1.0-aSensor)*iMotorABavg;
else
    iSensorAB1 = iMotorABavg;
end

iFilterAB0 = Rstart.'*x(5:6);
iSensorABavg = 0.5*(iSensorAB0+iSensorAB1);
if filterT > 0.0
    aFilter = exp(-Ts/filterT);
    iFilterAB1 = aFilter*iFilterAB0+(1.0-aFilter)*iSensorABavg;
else
    iFilterAB1 = iSensorABavg;
end
xNext = [iMotor1;Rend*iSensorAB1;Rend*iFilterAB1];
end

function idiqDelayed = propagate_external_delay(idiq,Ad,Bd,fd,Bs_TI, ...
    Sin_Cos_Theta,wel,Ts,uLast,delay)
sinTheta = Sin_Cos_Theta(1);
cosTheta = Sin_Cos_Theta(2);
Rp = [cosTheta,sinTheta;-sinTheta,cosTheta];
Tc = [2/3,-1/3,-1/3;0,1/sqrt(3),-1/sqrt(3)];
Tinv = [1,0;-0.5,sqrt(3)/2;-0.5,-sqrt(3)/2];
Tdq2abc = Tinv*Rp.';
Jrot = [0,-1;1,0];
iabc = Tdq2abc*idiq;
remaining = min(delay,8.0*Ts);
for k = 1:8
    if remaining <= 0.0
        break;
    end
    fraction = min(1.0,remaining/Ts);
    idqNow = Rp*(Tc*iabc);
    delta = Tdq2abc*((Ad-eye(2))*idqNow+fd+ ...
        Jrot*idqNow*(wel*Ts)+Bd*Rp*Bs_TI*uLast);
    iabc = iabc+fraction*delta;
    remaining = remaining-fraction*Ts;
end
idiqDelayed = Rp*(Tc*iabc);
end

function R = park_matrix(theta)
R = [cos(theta),sin(theta);-sin(theta),cos(theta)];
end
