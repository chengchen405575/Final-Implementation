function [H,U_unc] = Assembly_QP(Gamma,Phi,Apow, ...
    Lambdafsw_Ssquare,Lambdafsw_SE,Bs, ...
    idiqCompensated,idiq_ref,U_applied,regularization)
%ASSEMBLY_QP Assemble the SDA quadratic program.
%#codegen
%
% Current reconstruction and current-increment calculation are performed
% upstream by Current_Reconstruction. This function only builds and
% factorizes the unconstrained QP.
%
% Input dimensions:
%   Gamma, Apow               10x2
%   Phi                       10x1
%   Lambdafsw_Ssquare         15x15
%   Lambdafsw_SE              15x3
%   Bs                        2x3
%   idiqCompensated, idiq_ref 2x1
%   U_applied                 6x1 = [u1; u2]
%   regularization            scalar
% Outputs H and U_unc are 15x15 and 15x1.

Np = 5;
nu = 3;
ny = 2;

Gamma = double(Gamma);
Phi = double(Phi);
Apow = double(Apow);
Lambdafsw_Ssquare = double(Lambdafsw_Ssquare);
Lambdafsw_SE = double(Lambdafsw_SE);
Bs = double(Bs);
idiqCompensated = double(idiqCompensated);
idiq_ref = double(idiq_ref);
U_applied = double(U_applied);
regularization = max(0.0,double(regularization));

uPrev = U_applied(4:6);

Upsil = zeros(ny*Np,nu*Np);
for i = 1:Np
    rows = (i-1)*ny+(1:ny);
    for j = 1:i
        cols = (j-1)*nu+(1:nu);
        Upsil(rows,cols) = Apow((i-j)*ny+(1:ny),:)*Bs;
    end
end

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
