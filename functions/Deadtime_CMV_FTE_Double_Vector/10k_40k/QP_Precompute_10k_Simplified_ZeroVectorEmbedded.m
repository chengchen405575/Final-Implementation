function [H,Gamma,Phi,Lambdafsw_SE,Upsil, ...
    Bd,Bs_TI,deltaIdqBaseOffset,deltaIdqState, ...
    magnitudeGain,phaseCompensation,rippleAttenuation,rippleDelay] = ...
    QP_Precompute_10k_Simplified_ZeroVectorEmbedded( ...
    Ld,Lq,Rs,Psi,wel,Ts,Vdc,lambda_fsw,lambdaZeroEmbedded, ...
    Sin_Cos_Theta,filterTimeConstant,sensorTimeConstant)
%#codegen
% Slow-rate model/QP preparation with an exactly embedded zero-vector cost.
%
% For every three-bit switching vector u_j, let m_j = [1 1 1]*u_j.
% On the binary domain, m_j belongs to {0,1,2,3}, and
%
%   c0(u_j) = 0.5*(m_j-1)*(m_j-2)
%
% is exactly one for 000 and 111 and zero for all active vectors. Hence
%
%   lambdaZeroEmbedded*c0(u_j)
%     = u_j.'*(0.5*lambdaZeroEmbedded*ones(3))*u_j ...
%       + 2*(-0.75*lambdaZeroEmbedded*ones(3,1)).'*u_j ...
%       + lambdaZeroEmbedded.
%
% The quadratic term is embedded in H here. The matching linear term is
% rebuilt directly from the scalar lambdaZeroEmbedded in the 40-kHz
% runtime calculation, so no 15-element coefficient vector is transferred
% between the two rate domains. The constant Np*lambdaZeroEmbedded is
% omitted because it cannot change the minimizer.
% Set the separate zero-vector penalty in SDA to zero when this formulation
% is used; otherwise the commanded zero-vector cost is counted twice.

Np = 5;
nu = 3;
ny = 2;
Ts = max(realmin('double'),double(Ts));
filterTimeConstant = max(0.0,double(filterTimeConstant));
sensorTimeConstant = max(0.0,double(sensorTimeConstant));
lambdaZeroEmbedded = max(0.0,double(lambdaZeroEmbedded));

Ac = [-Rs/Ld, wel*Lq/Ld; -wel*Ld/Lq, -Rs/Lq];
Bc = [1/Ld, 0; 0, 1/Lq];
fc = [0; -wel*Psi/Lq];
M = [Ac, Bc, fc; zeros(3,5)];
Md = expm(M*Ts);
Ad = Md(1:2,1:2);
Bd = Md(1:2,3:4);
fd = Md(1:2,5);

Apow = zeros(ny*(Np+1),ny);
Apow(1:ny,:) = eye(ny);
for m = 1:Np
    Apow(m*ny+(1:ny),:) = Apow((m-1)*ny+(1:ny),:)*Ad;
end

Gamma = zeros(ny*Np,ny);
Phi = zeros(ny*Np,1);
for i = 1:Np
    rows = (i-1)*ny+(1:ny);
    Gamma(rows,:) = Apow(i*ny+(1:ny),:);
    acc = zeros(ny,1);
    for m = 0:i-1
        acc = acc+Apow(m*ny+(1:ny),:)*fd;
    end
    Phi(rows) = acc;
end

S = eye(nu*Np);
for i = 2:Np
    S((i-1)*nu+(1:nu),(i-2)*nu+(1:nu)) = -eye(nu);
end
E = [eye(nu); zeros(nu*(Np-1),nu)];
lambdaEffective = max(lambda_fsw,1.0e-6);
Lambdafsw_SE = lambdaEffective*(S.'*E);
Lambdafsw_Ssquare = lambdaEffective*(S.'*S);

% Exact quadratic coefficient of the zero-vector indicator.
WzeroEmbedded = 0.5*lambdaZeroEmbedded* ...
    kron(eye(Np),ones(nu));

% One common midpoint electrical angle for the entire controller model.
sinTheta = Sin_Cos_Theta(1);
cosTheta = Sin_Cos_Theta(2);
deltaTheta = wel*50.0e-6;
sinDelta = sin(deltaTheta);
cosDelta = cos(deltaTheta);
sinMid = sinTheta*cosDelta+cosTheta*sinDelta;
cosMid = cosTheta*cosDelta-sinTheta*sinDelta;
RpMid = [cosMid, sinMid; -sinMid, cosMid];
Msw = eye(3)-ones(3)/3;
Tc = (2/3)*[1, -1/2, -1/2; 0, sqrt(3)/2, -sqrt(3)/2];
Bs_TI = Tc*Vdc*Msw;
deltaIdqSwitchMid = Bd*RpMid*Bs_TI;
Jrot = [0 -1; 1 0];
deltaIdqState = Ad-eye(2)+Jrot*(wel*Ts);
deltaIdqBaseOffset = fd;

Upsil = zeros(ny*Np,nu*Np);
for i = 1:Np
    rows = (i-1)*ny+(1:ny);
    for j = 1:i
        cols = (j-1)*nu+(1:nu);
        Upsil(rows,cols) = ...
            Apow((i-j)*ny+(1:ny),:)*deltaIdqSwitchMid;
    end
end

W = Upsil.'*Upsil+Lambdafsw_Ssquare+WzeroEmbedded;
W = 0.5*(W+W.');
W = W+1.0e-12*max(1.0,norm(W,'fro'))*eye(nu*Np);
Jx = fliplr(eye(nu*Np));
[Rt,p] = chol(Jx*W*Jx);
if p ~= 0
    error('H is not positive definite. Increase lambda_sw.');
end
H = Jx*Rt*Jx;

% Angle/frequency-related filter coefficients are also slow-rate values.
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
phase = wel*basePhaseDelay;
phaseCompensation = ...
    [cos(phase), -sin(phase); sin(phase), cos(phase)];

omegaRipple = 2.0*pi*17.0e3;
xFilter = omegaRipple*filterTimeConstant;
xSensor = omegaRipple*sensorTimeConstant;
rippleAttenuation = 1.0/(sqrt(1.0+xFilter*xFilter)* ...
    sqrt(1.0+xSensor*xSensor));
rippleDelay = (atan(xFilter)+atan(xSensor))/omegaRipple;
rippleDelay = min(max(0.0,rippleDelay),2.0*Ts);
end
