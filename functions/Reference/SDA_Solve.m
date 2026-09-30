function [Jopt, nodes, optimal, Uopt, Uapply] = SDA_Solve(H, U_unc, NmaxNod, UoptPrev)
nu = 3;
V  = [0 1];
n  = numel(U_unc);
MAXIT = 100000;         % harte Obergrenze der Schleifendurchlaeufe

H     = double(H);
U_unc = double(U_unc);
Nmax  = double(NmaxNod);
Uprev = double(UoptPrev);
Ubar = H * U_unc;

Uround  = min(max(round(U_unc), V(1)), V(2));
Jround    = sum((H*(Uround - U_unc)).^2);

Ushift               = zeros(n,1);
Ushift(1:n-nu)       = Uprev(nu+1:n);
Ushift(n-nu+1:n)     = Uprev(n-nu+1:n);
Jshift               = sum((H*(Ushift - U_unc)).^2);

% --- den besseren als Inkumbente
if Jshift < Jround
    best = Jshift;
    Uopt = Ushift;
else
    best = Jround;
    Uopt = Uround;
end
nodes   = 0;
optimal = true;

U     = zeros(n,1);
d2s   = zeros(n+1,1);
cand  = zeros(n,2);
kidx  = zeros(n,1);
baseS = zeros(n,1);

i        = 1;
newLevel = true;

for it = 1:MAXIT
    if it >= MAXIT
        optimal = false;
    end
    ii = min(max(i,1), n);          % dem Coder die Schranke zeigen

    if newLevel
        acc = 0;
        for j = 1:n
            if j <= ii-1
                acc = acc + H(ii,j)*U(j);
            end
        end
        baseS(ii) = Ubar(ii) - acc;
        if baseS(ii) <= 0.5*H(ii,ii)
            cand(ii,1) = V(1);  cand(ii,2) = V(2);
        else
            cand(ii,1) = V(2);  cand(ii,2) = V(1);
        end
        kidx(ii) = 1;
        newLevel = false;
    end

    if kidx(ii) > 2
        i = i - 1;
        if i >= 1
            kidx(i) = kidx(i) + 1;
        end
        continue;
    end

    v     = cand(ii, kidx(ii));
    nodes = nodes + 1;
    if Nmax > 0 && nodes > Nmax
        optimal = false;
        break;
    end

    U(ii) = v;
    inc   = (H(ii,ii)*v - baseS(ii))^2;
    tot   = d2s(ii) + inc;

    if tot < best
        if ii == n
            best     = tot;
            Uopt     = U;
            kidx(ii) = kidx(ii) + 1;
        else
            d2s(ii+1) = tot;
            i         = i + 1;
            newLevel  = true;
        end
    else
        kidx(ii) = 3;
    end
end

Jopt   = best;
Uapply = Uopt(1:nu);
end