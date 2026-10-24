function [H, U_unc, iabcCompensated, ...
    deltaIabcBase, deltaIabcSwitch, predictionError, e_dq, ...
    iabcAverageEstimated, iabcRippleEstimated] = ...
    Assemby_QP_Ripple_Reconstruction(Ad, Bd, fd, Gamma, Phi, Apow, ...
    Lambdafsw_Ssquare, Lambdafsw_SE, Bs_TI, ...
    idiqFiltered, idiq_ref, Sin_Cos_Theta, theta_el, ...
    wel, Ts, U_applied, regularization, controllerDelay, ...
    iabcPredictedPrevious, predictionValid, ...
    filterTimeConstant, sensorTimeConstant, rippleGain)
%ASSEMBY_QP_RIPPLE_RECONSTRUCTION QP assembly with current reconstruction.
%#codegen
%
% Signal dimensions:
%   Ad, Bd, fd                 2x2, 2x2, 2x1
%   Gamma, Phi, Apow          10x2, 10x1, 10x2
%   Lambdafsw_Ssquare         15x15
%   Lambdafsw_SE              15x3
%   Bs_TI                     2x3
%   idiqFiltered, idiq_ref    2x1, 2x1
%   Sin_Cos_Theta             2x1 = [sin(theta_el); cos(theta_el)]
%   U_applied                 6x1 = [uFirst; uSecond]
%   iabcPredictedPrevious     3x1
%   all remaining inputs      scalar
% Outputs iabcAverageEstimated and iabcRippleEstimated are both 3x1 and
% refer to the sampling boundary before controller-delay propagation. The
% ripple diagnostic is the raw, unscaled model result so that rippleGain
% can be identified independently from logged data.
%
% The measured current is separated into a predicted filtered switching-
% ripple component and a filtered average-current component. The dominant
% phase-current ripple is the lower switching sideband
%   wRipple = 2*pi*(2/Ts) - abs(wel).
% Its attenuation and phase delay through the two filters are evaluated
% analytically. The delayed residual ripple is reconstructed from the
% known double-vector switching sequence and removed from the measurement.
%
% The remaining average current is then inverted only at the electrical
% fundamental frequency. For
%   Hf(s) = 1/((1+s*Tf)*(1+s*Tsensor)),
% the inverse frequency response is
%   Hf(j*w)^-1 = a + j*b,
%   a = 1-w^2*Tf*Tsensor,  b = w*(Tf+Tsensor).
% This restores the steady fundamental-current magnitude and phase. A
% separate short-horizon state propagation over the switching-ripple delay
% restores the non-periodic current movement caused by the known applied
% vectors; this term remains nonzero when uFirst == uSecond. The periodic
% zero-mean boundary ripple is then added independently. Both switching-
% related corrections are scaled by rippleGain.
% Use rippleGain=1 until an independently validated schedule is connected.
%
% Timing contract: at the sampling boundary uSecond is the physically
% active switch state and the gate driver holds it until the new command
% becomes effective. controllerDelay may therefore exceed the nominal
% 0.5*Ts duration of the second vector. With only one period of switching
% history available, compensation is intentionally limited to one Ts. A
% controllerDelay greater than Ts requires an explicit multi-period input
% delay model and additional applied-state history.

Np = 5;
nu = 3;
ny = 2;

Ad = double(Ad);
Bd = double(Bd);
fd = double(fd);
Gamma = double(Gamma);
Phi = double(Phi);
Apow = double(Apow);
Lambdafsw_Ssquare = double(Lambdafsw_Ssquare);
Lambdafsw_SE = double(Lambdafsw_SE);
Bs_TI = double(Bs_TI);
idiqFiltered = double(idiqFiltered);
idiq_ref = double(idiq_ref);
Sin_Cos_Theta = double(Sin_Cos_Theta);
theta_el = double(theta_el); %#ok<NASGU> interface/timing documentation
wel = double(wel);
Ts = max(realmin('double'),double(Ts));
U_applied = double(U_applied);
regularization = max(0.0,double(regularization));
controllerDelay = max(0.0,double(controllerDelay));
iabcPredictedPrevious = double(iabcPredictedPrevious);
predictionValid = logical(predictionValid);
filterTimeConstant = max(0.0,double(filterTimeConstant));
sensorTimeConstant = max(0.0,double(sensorTimeConstant));
rippleGain = max(0.0,double(rippleGain));

sinT = Sin_Cos_Theta(1);
cosT = Sin_Cos_Theta(2);
Tab = [2/3, -1/3, -1/3; ...
      0, 1/sqrt(3), -1/sqrt(3)];
Tdq = [cosT, sinT; -sinT, cosT];
Tba = [1.0, 0.0; ...
             -0.5, sqrt(3)/2; ...
             -0.5, -sqrt(3)/2];
Tiq = Tba*Tdq.';

u1 = U_applied(1:3);
u2 = U_applied(4:6);
uAvg = 0.5*(u1+u2);
uPrev = u2;

% Full-Ts switching-current sensitivity. Unlike the base increment, this
% term is independent of the current state.
Bq = Bd*Tdq*Bs_TI;
dIdu = Tiq*Bq;

% Dominant phase-current switching sideband. The applied double-vector
% waveform has two switching intervals per SDA period. In the recorded
% data its dominant lower sideband is 2/Ts-fe, where fe=abs(wel)/(2*pi).
wSw = 4.0*pi/Ts;
wR = max(1.0,wSw-abs(wel));
xFilt = wR*filterTimeConstant;
xSens = wR*sensorTimeConstant;
Ar = 1.0/(sqrt(1.0+xFilt*xFilt)*sqrt(1.0+xSens*xSens));
tauR = (atan(xFilt)+atan(xSens))/wR;
tauR = min(max(0.0,tauR),Ts);

% Reconstruct both the current boundary ripple and the earlier ripple seen
% through the filter phase delay. The current sample is assumed to occur at
% the end/start boundary of the equal-duration double-vector period.
tNow = Ts;
tDelayed = tNow-tauR;
iabcRippleEstimated = ripple_at_phase(dIdu,u1,u2,uAvg,Ts,tNow);
iRippleOld = ripple_at_phase(dIdu,u1,u2,uAvg,Ts,tDelayed);

% Remove the small switching ripple that remains in the measured filtered
% current. This avoids treating the residual (about 8.5 percent for the
% current filter settings) as part of the fundamental/average current.
iFilt = Tiq*idiqFiltered;
iAvgFilt = iFilt-Ar*iRippleOld;
iqAvgFilt = Tdq*(Tab*iAvgFilt);

% Exact fundamental-frequency inverse of the two cascaded filters. With
% With the Park convention used by Tdq, multiplication by a+j*b becomes the
% real
% 2x2 operation [a -b; b a].
aF = 1.0-wel*wel*filterTimeConstant*sensorTimeConstant;
bF = wel*(filterTimeConstant+sensorTimeConstant);
iqAvg = [aF*iqAvgFilt(1)-bF*iqAvgFilt(2); ...
         bF*iqAvgFilt(1)+aF*iqAvgFilt(2)];
iabcAverageEstimated = Tiq*iqAvg;

% The sinusoidal inverse above cannot recover a non-periodic current ramp
% occurring inside the filter's switching-frequency delay window. Propagate
% the reconstructed average state through the known recent switching
% history. Looking backwards from the sampling boundary, uSecond occupies
% the most recent half-period and uFirst the preceding half-period. This is
% deliberately separate from the zero-mean ripple definition: if
% uFirst == uSecond, iabcRippleEstimated is zero but the state propagation
% can still be nonzero.
dISw = recent_switch_delta( ...
    iabcAverageEstimated,Ad,Bd,fd,Bs_TI,wel,Ts,Tab,Tdq,Tiq, ...
    u1,u2,tauR);

% Reconstruct the current at the present sampling boundary. rippleGain is
% an identifiable gain for the complete switching-frequency correction; it
% does not alter the analytically inverted fundamental component.
iSample = iabcAverageEstimated+rippleGain*(dISw+iabcRippleEstimated);

% External-delay propagation only. The last actually applied vector is
% held during computation. One fractional discrete increment is sufficient
% because the supported delay is bounded by one control period.
tCtrl = min(controllerDelay,Ts);
kCtrl = tCtrl/Ts;
[dI0,dIu,~,~] = current_delta_abc( ...
    iSample,Ad,Bd,fd,Bs_TI,wel,Ts,Tab,Tdq,Tiq);
iabcCompensated = iSample+kCtrl*(dI0+dIu*u2);

% Assemble the QP from the reconstructed current at the actuation instant.
[deltaIabcBase,deltaIabcSwitch,idiqCompensated,Bs] = ...
    current_delta_abc(iabcCompensated,Ad,Bd,fd,Bs_TI,wel,Ts, ...
    Tab,Tdq,Tiq);

Upsil = zeros(ny*Np,nu*Np);
for i = 1:Np
    ri = (i-1)*ny+(1:ny);
    for j = 1:i
        cj = (j-1)*nu+(1:nu);
        Upsil(ri,cj) = Apow((i-j)*ny+(1:ny),:)*Bs;
    end
end

if predictionValid
    predictionError = iabcCompensated-iabcPredictedPrevious;
else
    predictionError = zeros(3,1);
end
e_dq = Tdq*(Tab*predictionError);

r = Gamma*idiqCompensated+Phi-repmat(idiq_ref,Np,1);
W = Upsil.'*Upsil+Lambdafsw_Ssquare;
F = Upsil.'*r-Lambdafsw_SE*uPrev;
W = 0.5*(W+W.');
scaleW = max(1.0,norm(W,'fro'));
n = size(W,1);
W = W+regularization*scaleW*eye(n);

Jx = fliplr(eye(n));
[Rt,p] = chol(Jx*W*Jx);
if p ~= 0
    error('H is not positive definite. Increase lambda_sw or regularization.');
end
H = Jx*Rt*Jx;
y = H.'\(-F);
U_unc = H\y;
end

function dI = recent_switch_delta( ...
    i0,Ad,Bd,fd,Bs_TI,wel,Ts,Tab,Tdq,Tiq,u1,u2,tau)
% Approximate the state change over the final historyDuration of the last
% equal-duration double-vector period. Each discrete increment returned by
% current_delta_abc corresponds to one full Ts and is scaled by the
% duration fraction. The state is updated between the two possible pieces
% so the base-current term is evaluated at the latest available state.
halfTs = 0.5*Ts;
tau = min(max(0.0,tau),Ts);
t2 = min(tau,halfTs);
t1 = max(0.0,tau-halfTs);

i = i0;
if t1 > 0.0
    [dI0,dIu,~,~] = current_delta_abc( ...
        i,Ad,Bd,fd,Bs_TI,wel,Ts,Tab,Tdq,Tiq);
    i = i+(t1/Ts)*(dI0+dIu*u1);
end
if t2 > 0.0
    [dI0,dIu,~,~] = current_delta_abc( ...
        i,Ad,Bd,fd,Bs_TI,wel,Ts,Tab,Tdq,Tiq);
    i = i+(t2/Ts)*(dI0+dIu*u2);
end
dI = i-i0;
end

function [dI0,dIu,iq,Bs] = current_delta_abc( ...
    iabc,Ad,Bd,fd,Bs_TI,wel,Ts,Tab,Tdq,Tiq)
Jrot = [0 -1; 1 0];
iq = Tdq*(Tab*iabc);
Bs = Bd*Tdq*Bs_TI;
dIq0 = (Ad-eye(2))*iq+fd;
dI0 = Tiq*(dIq0+Jrot*iq*(wel*Ts));
dIu = Tiq*Bs;
end

function iR = ripple_at_phase(dIdu,u1,u2,uAvg,Ts,t)
% Periodic zero-mean triangular ripple for an equal-duration two-vector
% sequence. deltaSwitch represents a full-Ts current increment.
duty1 = 0.5;
halfTs = 0.5*Ts;
dI1Full = dIdu*(u1-uAvg);
dI2Full = dIdu*(u2-uAvg);
dI1 = duty1*dI1Full;
iR0 = -0.5*dI1;
t = min(max(0.0,t),Ts);
if t <= halfTs
    iR = iR0+(t/Ts)*dI1Full;
else
    iR = iR0+dI1+((t-halfTs)/Ts)*dI2Full;
end
end
