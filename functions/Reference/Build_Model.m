function [Gamma, Phi, Lambdafsw_Ssquare, Lambdafsw_SE, Apow, Bs_TI]    = BuildModel(Ad, fd, Vdc, lambda_fsw)
Np = 5;
nu = 3;  ny = 2;
C  = eye(ny);
Apow = zeros(ny*(Np+1), ny);
Apow(1:ny, :) = eye(ny);
for m = 1:Np
    Apow(m*ny + (1:ny), :) = Apow((m-1)*ny + (1:ny), :) * Ad;
end
Bs_TI = switch_to_alphabeta(Vdc);

Gamma = zeros(ny*Np, ny);
Phi   = zeros(ny*Np, 1);
for i = 1:Np
    ri = (i-1)*ny + (1:ny);
    Gamma(ri,:) = C * Apow(i*ny + (1:ny), :);
    acc = zeros(ny,1);
    for m = 0:i-1, acc = acc + Apow(m*ny + (1:ny), :)*fd; end
    Phi(ri) = C * acc;
end

S = eye(nu*Np);
for i = 2:Np
    ri = (i-1)*nu + (1:nu);
    rj = (i-2)*nu + (1:nu);
    S(ri,rj) = -eye(nu);
end
E = [eye(nu); zeros(nu*(Np-1), nu)];
lambda_eff = max(lambda_fsw, 1e-6);
Lambdafsw_SE = lambda_eff * (S.'*E);
Lambdafsw_Ssquare =  lambda_eff*(S.'*S);


end

function Bs_TI = switch_to_alphabeta(Vdc)
Msw = eye(3) - ones(3)/3;
Tc  = (2/3) * [ 1, -1/2,      -1/2;
    0,  sqrt(3)/2, -sqrt(3)/2 ];   % Clarke (2x3)

Bs_TI = Tc * Vdc * Msw;                % 2x3

end