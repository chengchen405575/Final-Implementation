%% Tabellen fuer Simscape PMSM-Block "Tabulated Ld, Lq and PM"
% Erzeugt Ld(id,iq), Lq(id,iq) aus MG_LUT.mat; PM bleibt KONSTANT (skalar).
%
% Block-Definition:  psi_d = Ld*id + PM ,  psi_q = Lq*iq   (Sekanten-Induktivitaet)
%
% Zwei Reparaturen gegen die i->0-Singularitaet:
%  Lq: psi_q hat bei iq=0 einen Messoffset (~0.5 mWb) -> vor der Division
%      pro id-Spalte zentrieren (psi_q(id,0)=0). Danach ist die Sekante
%      symmetrisch und glatt durch Null. Der Block kann diesen Offset
%      ohnehin nicht abbilden (psi_q=Lq*iq ist bei iq=0 zwangslaeufig 0).
%  Ld: PM ist konstant, daher darf der Offset psi_d(0,iq)-PM NICHT in PM
%      wandern. Rohe Sekante (psi_d-PM)/id wuerde die Niedrig-id-Region
%      zerstoeren. Stattdessen Ld aus der id-AENDERUNG bilden, referenziert
%      auf psi_d(id=0,iq) -> glatte, physikalische Ld-Karte.
%      Preis: der d-Fluss traegt einen beschraenkten Fehler (= psi_d(0,iq)-PM),
%      der mit iq waechst (Kreuzsaettigung, die ein konstantes PM nicht fasst).
%      -> Skript gibt diesen Fehler am Ende aus.

clear; clc;

AINV   = sqrt(2/3);
MG_LUT = load('MG_LUT.mat');

id = double(MG_LUT.id_vec(:)) * AINV;    % Nid
iq = double(MG_LUT.iq_vec(:)) * AINV;    % Niq
Fd = MG_LUT.IdIq_Fluxd_POS_grid * AINV;  % [Niq x Nid] : Zeile=iq, Spalte=id
Fq = MG_LUT.IdIq_Fluxq_POS_grid * AINV;

Nid = numel(id);  Niq = numel(iq);
[~, id0] = min(abs(id));
[~, iq0] = min(abs(iq));
eps_z = 3;                               % [A] ersetzt nur die exakte Nulllinie

PM_const = 27.6/(2*pi*4*(1200/60)) * AINV;   % konstantes PM (Datenblatt)

%% --- Ld: referenziert auf psi_d(id=0,iq), Nulllinie via Differential ---
Ld = (Fd - Fd(:, id0)) ./ id.';          % [Niq x Nid], glatt
dLd = grad_along(Fd, id, 2);
Ld(:, abs(id) < eps_z) = dLd(:, abs(id) < eps_z);

%% --- Lq: psi_q offset-zentriert, Nulllinie via Differential ---
Fq_c = Fq - Fq(iq0, :);                   % psi_q(id,0)=0 erzwingen
Lq   = Fq_c ./ iq;                        % [Niq x Nid], glatt/symmetrisch
dLq  = grad_along(Fq_c, iq, 1);
Lq(abs(iq) < eps_z, :) = dLq(abs(iq) < eps_z, :);

%% --- Orientierung fuer den Block: [Nid x Niq] (Zeile=iD, Spalte=iQ) ---
iD_vec = id;
iQ_vec = iq;
Ld_tab = Ld.';                            % [Nid x Niq]
Lq_tab = Lq.';
PM_tab = PM_const * ones(Nid, Niq);       % konstant, als 2D-Tabelle fuer den Block

%% --- Reports ---
psid_rec = Ld_tab .* iD_vec + PM_tab;     % [Nid x Niq]
err = abs(psid_rec - Fd.') * 1e3;         % mWb
fprintf('Ld: %.4f..%.4f mH | Lq: %.4f..%.4f mH | negativ: %d/%d | finite: %d\n', ...
    min(Ld_tab,[],'all')*1e3, max(Ld_tab,[],'all')*1e3, ...
    min(Lq_tab,[],'all')*1e3, max(Lq_tab,[],'all')*1e3, ...
    nnz(Ld_tab<0)+nnz(Lq_tab<0), numel(Ld_tab)+numel(Lq_tab), ...
    all(isfinite(Ld_tab),'all') & all(isfinite(Lq_tab),'all'));
fprintf('const-PM d-Fluss-Fehler [mWb]: median %.2f | p95 %.2f | max %.2f (waechst mit |iq|)\n', ...
    median(err,'all'), prctile(err(:),95), max(err,[],'all'));

save('PMSM_tables.mat','iD_vec','iQ_vec','Ld_tab','Lq_tab','PM_tab','PM_const');

%% --- Sichtprobe ---
[IDg, IQg] = ndgrid(iD_vec, iQ_vec);
figure('Color','w','Name','PMSM-Tabellen (PM konstant)');
subplot(1,2,1); surf(IDg,IQg,Ld_tab*1e3,'EdgeColor','none'); view(40,30);
xlabel('i_d [A]'); ylabel('i_q [A]'); zlabel('L_d [mH]'); title('L_d(i_d,i_q)'); colorbar;
subplot(1,2,2); surf(IDg,IQg,Lq_tab*1e3,'EdgeColor','none'); view(40,30);
xlabel('i_d [A]'); ylabel('i_q [A]'); zlabel('L_q [mH]'); title('L_q(i_d,i_q)'); colorbar;

%% ===== Hilfsfunktion: differentielle Ableitung entlang Achse =====
function G = grad_along(F, x, dim)
    if dim == 2, F = F.'; end
    x = x(:); G = zeros(size(F));
    G(2:end-1,:) = (F(3:end,:) - F(1:end-2,:)) ./ (x(3:end) - x(1:end-2));
    G(1,:)       = (F(2,:)     - F(1,:))       ./ (x(2)     - x(1));
    G(end,:)     = (F(end,:)   - F(end-1,:))   ./ (x(end)   - x(end-1));
    if dim == 2, G = G.'; end
end