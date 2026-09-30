function [H, U_unc] = fcn(Ad, Bd, fd, Gamma, Phi, Apow, ...
    Lambdafsw_Ssquare, Lambdafsw_SE, Bs_TI, ...
    idiq_fb, idiq_ref, theta_el, wel, Ts, uprev)
Np = 5;
nu = 3;  ny = 2;
% --- Schaltmatrizen: l = 0 fuer die Totzeit, l = 1..Np fuer den Horizont
Bs = zeros(ny, nu, Np+1);
for l = 0:Np
    th = theta_el + wel*Ts*l;
    Rp = [ cos(th), sin(th); -sin(th), cos(th) ];
    Bs(:,:,l+1) = Bd * Rp * Bs_TI;
end

% --- Totzeitkompensation: uprev wirkt im Intervall k -> k+1
idiq_k1 = Ad*idiq_fb + Bs(:,:,1)*uprev + fd;

% --- Praediktion ab k+1: U(j) wirkt im Intervall k+j -> k+j+1
Upsil = zeros(ny*Np, nu*Np);
for i = 1:Np
    ri = (i-1)*ny + (1:ny);
    for j = 1:i
        cj = (j-1)*nu + (1:nu);
        Upsil(ri,cj) = Apow((i-j)*ny + (1:ny), :) * Bs(:,:,j+1);
    end
end

W = Upsil.'*Upsil + Lambdafsw_Ssquare;

% --- Reverse Cholesky
n  = size(W,1);
Jx = fliplr(eye(n));
[Rt, p] = chol(Jx*W*Jx);
if p > 0
    Rt = eye(n);
end
H = Jx*Rt*Jx;

% --- lineare Terme
r = Gamma*idiq_k1 + Phi - repmat(idiq_ref, Np, 1);
F = Upsil.'*r - Lambdafsw_SE*uprev;

y     = H.' \ (-F);
U_unc = H   \ y;
end