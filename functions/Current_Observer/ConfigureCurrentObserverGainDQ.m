function [K,info] = ConfigureCurrentObserverGainDQ(Ad,wel,Ts, ...
    sensorTimeConstant,filterTimeConstant,processVariance,measurementVariance)
%CONFIGURECURRENTOBSERVERGAINDQ Design one scheduled observer gain.
% Run during initialization, never inside a generated MATLAB Function block.
% Ad is the motor transition for one vector duration Ts at the same wel.
% processVariance (6x1) and measurementVariance (2x1) are tuning values.

validateattributes(Ad,{'double'},{'size',[2,2],'finite'});
validateattributes(wel,{'double'},{'scalar','finite'});
validateattributes(Ts,{'double'},{'scalar','positive','finite'});
validateattributes(sensorTimeConstant,{'double'},{'scalar','nonnegative','finite'});
validateattributes(filterTimeConstant,{'double'},{'scalar','nonnegative','finite'});
validateattributes(processVariance,{'double'},{'size',[6,1],'positive','finite'});
validateattributes(measurementVariance,{'double'},{'size',[2,1],'positive','finite'});

% Form the homogeneous one-vector error transition numerically. Input and
% affine terms cancel from observer error dynamics.
Fhalf = zeros(6);
for column = 1:6
    basis = zeros(6,1);
    basis(column) = 1.0;
    Fhalf(:,column) = homogeneous_half(basis,Ad,wel,Ts, ...
        sensorTimeConstant,filterTimeConstant);
end
Fperiod = Fhalf*Fhalf;
C = [zeros(2,4),eye(2)];
Q = diag(processVariance);
Rnoise = diag(measurementVariance);
Pprior = dare(Fperiod.',C.',Q,Rnoise);
K = (Pprior*C.')/(C*Pprior*C.'+Rnoise);
errorTransition = (eye(6)-K*C)*Fperiod;
info.Fhalf = Fhalf;
info.Fperiod = Fperiod;
info.actualPoles = eig(errorTransition);
info.spectralRadius = max(abs(info.actualPoles));
info.observabilityRank = rank(obsv(Fperiod,C));
info.gainNorm = norm(K,2);
assert(info.observabilityRank == 6,'Augmented model is not observable.');
assert(info.spectralRadius < 1,'Configured observer is unstable.');
end

function xNext = homogeneous_half(x,Ad,wel,Ts,sensorT,filterT)
thetaEnd = wel*Ts;
Rstart = [1,0;0,1];
Rend = [cos(thetaEnd),sin(thetaEnd);-sin(thetaEnd),cos(thetaEnd)];
iMotor0 = x(1:2);
iMotor1 = Ad*iMotor0;
iMotorABavg = 0.5*(Rstart.'*iMotor0+Rend.'*iMotor1);
iSensorAB0 = Rstart.'*x(3:4);
if sensorT > 0.0
    aSensor = exp(-Ts/sensorT);
    iSensorAB1 = aSensor*iSensorAB0+(1-aSensor)*iMotorABavg;
else
    iSensorAB1 = iMotorABavg;
end
iFilterAB0 = Rstart.'*x(5:6);
iSensorABavg = 0.5*(iSensorAB0+iSensorAB1);
if filterT > 0.0
    aFilter = exp(-Ts/filterT);
    iFilterAB1 = aFilter*iFilterAB0+(1-aFilter)*iSensorABavg;
else
    iFilterAB1 = iSensorABavg;
end
xNext = [iMotor1;Rend*iSensorAB1;Rend*iFilterAB1];
end
