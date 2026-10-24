function [Bd, Ad, fd] = Discretize_PMSM(Ld, Lq, Rs, Psi, wel, Ts)

Ac = [ -Rs/Ld,        wel*Lq/Ld;
    -wel*Ld/Lq,  -Rs/Lq ];

Bc = [ 1/Ld, 0;
    0,    1/Lq ];

fc = [ 0;
    -wel*Psi/Lq ];

% Augmented matrix: states (2) + voltage inputs (2) + affine (1) = 5
M = [ Ac, Bc, fc;
    zeros(3,5) ];

Md = expm(M * Ts);

Ad   = Md(1:2, 1:2);
Bd = Md(1:2, 3:4);     % maps v_dq -> state increment
fd   = Md(1:2, 5);     % back-EMF contribution per step