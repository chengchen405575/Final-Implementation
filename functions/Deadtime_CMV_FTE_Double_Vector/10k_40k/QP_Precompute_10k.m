function [H,Gamma,Phi,Lambdafsw_SE,Upsil,Ad,Bd,fd,Bs_TI,BsFrozen] = ...
    QP_Precompute_10k(Ld,Lq,Rs,Psi,wel,Ts,Vdc,lambda_fsw,Sin_Cos_Theta)
%#codegen
% Slow QP/model preparation. The equations are copied from
% Discretize_PMSM, Build_Model and the H branch of Assembly_QP.
% H is frozen at theta + wel*50 us, the midpoint of the next 100 us
% processor interval.

Np = 5;
nu = 3;
ny = 2;

%% Discretize_PMSM (unchanged equations)
Ac = [-Rs/Ld, wel*Lq/Ld; ...
      -wel*Ld/Lq, -Rs/Lq];
Bc = [1/Ld, 0; 0, 1/Lq];
fc = [0; -wel*Psi/Lq];
M = [Ac, Bc, fc; zeros(3,5)];
Md = expm(M*Ts);
Ad = Md(1:2,1:2);
Bd = Md(1:2,3:4);
fd = Md(1:2,5);

%% Build_Model (unchanged equations)
C = eye(ny);
Apow = zeros(ny*(Np+1),ny);
Apow(1:ny,:) = eye(ny);
for m = 1:Np
    Apow(m*ny+(1:ny),:) = Apow((m-1)*ny+(1:ny),:)*Ad;
end

Bs_TI = switch_to_alphabeta(Vdc);
Gamma = zeros(ny*Np,ny);
Phi = zeros(ny*Np,1);
for i = 1:Np
    ri = (i-1)*ny+(1:ny);
    Gamma(ri,:) = C*Apow(i*ny+(1:ny),:);
    acc = zeros(ny,1);
    for m = 0:i-1
        acc = acc+Apow(m*ny+(1:ny),:)*fd;
    end
    Phi(ri) = C*acc;
end

S = eye(nu*Np);
for i = 2:Np
    ri = (i-1)*nu+(1:nu);
    rj = (i-2)*nu+(1:nu);
    S(ri,rj) = -eye(nu);
end
E = [eye(nu); zeros(nu*(Np-1),nu)];
lambda_eff = max(lambda_fsw,1e-6);
Lambdafsw_SE = lambda_eff*(S.'*E);
Lambdafsw_Ssquare = lambda_eff*(S.'*S);

%% Freeze H at the midpoint angle theta + wel*50 us.
sinTheta = Sin_Cos_Theta(1);
cosTheta = Sin_Cos_Theta(2);
deltaTheta = wel*50.0e-6;
sinDelta = sin(deltaTheta);
cosDelta = cos(deltaTheta);
sinThetaMid = sinTheta*cosDelta+cosTheta*sinDelta;
cosThetaMid = cosTheta*cosDelta-sinTheta*sinDelta;
RpMid = [cosThetaMid, sinThetaMid; ...
        -sinThetaMid, cosThetaMid];
BsFrozen = Bd*RpMid*Bs_TI;

%% Assembly_QP: construct Upsil, W and H (same operation order).
Upsil = zeros(ny*Np,nu*Np);
for i = 1:Np
    rows = (i-1)*ny+(1:ny);
    for j = 1:i
        cols = (j-1)*nu+(1:nu);
        power = i-j;
        Upsil(rows,cols) = Apow(power*ny+(1:ny),:)*BsFrozen;
    end
end

W = Upsil.'*Upsil+Lambdafsw_Ssquare;
W = 0.5*(W+W.');
regularization = 1.0e-12; % Former external input, retained internally.
scaleW = max(1,norm(W,'fro'));
W = W+regularization*scaleW*eye(nu*Np);

n = nu*Np;
Jx = fliplr(eye(n));
[Rt,p] = chol(Jx*W*Jx);
if p ~= 0
    error('H is not positive definite. Increase lambda_sw or regularization.');
end
H = Jx*Rt*Jx;
end

function Bs_TI = switch_to_alphabeta(Vdc)
% Original Build_Model helper.
Msw = eye(3)-ones(3)/3;
Tc = (2/3)*[1, -1/2, -1/2; ...
            0, sqrt(3)/2, -sqrt(3)/2];
Bs_TI = Tc*Vdc*Msw;
end
