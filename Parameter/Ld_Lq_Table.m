AINV = sqrt(2/3); % Factor for Amplitudeninvariant id_IANV = id_PINV*sqrt(2/3)

MG_LUT = load('MG_LUT.mat');
MG_LUT.id_vec = MG_LUT.id_vec*AINV;
MG_LUT.iq_vec = MG_LUT.iq_vec*AINV;
MG_LUT.IdIq_Fluxd_POS_grid = MG_LUT.IdIq_Fluxd_POS_grid*AINV;
MG_LUT.IdIq_Fluxq_POS_grid = MG_LUT.IdIq_Fluxq_POS_grid*AINV;

iq = MG_LUT.iq_vec;
id = MG_LUT.id_vec;
PsipMeas = 27.6/(2*pi*4*(1200/60))*AINV;
Fluxq = MG_LUT.IdIq_Fluxq_POS_grid;
Fluxd_MinusPsip = MG_LUT.IdIq_Fluxd_POS_grid-PsipMeas;


epsilon = 1e-10;  % Strom unterhalb dieser Grenze wird als numerisch zu klein behandelt

for i = 1:length(iq)
    if abs(iq(i)) > epsilon
        LqTab(i,:) = Fluxq(i,:) / iq(i);
    else
        % Glätten: Interpolation aus benachbarten Werten
        if i > 1 && i < length(iq)
            LqTab(i,:) = (Fluxq(i+1,:) - Fluxq(i-1,:)) / (iq(i+1) - iq(i-1));
        else
            LqTab(i,:) = NaN;
        end
    end
end


for j =1:numel(Fluxq(1,:))
LqVec(j) =mean(LqTab(:,j));
end
save('LqVec','LqVec');
for i = 1:length(id)
    if abs(id(i)) > epsilon
        LdTab(i,:) = Fluxd_MinusPsip(:,i)/id(i);
    else
        % Glätten: Interpolation aus benachbarten Werten
        if i > 1 && i < length(id)
            LdTab(i,:) = (Fluxd_MinusPsip(:,i+1) - Fluxd_MinusPsip(:,i-1)) / (id(i+1) - id(i-1));
        else
            LdTab(i,:) = NaN;
        end
    end
end
for j =1:numel(Fluxd_MinusPsip(:,1))
LdVec(j) =mean(LdTab(:,j));
end
save('LdVec','LdVec');


%% 2D-Plots: Ld/Lq-Tabellen über id/iq-Gitter
[ID, IQ] = meshgrid(id, iq);   % id = Spalten, iq = Zeilen (wie in LdTab/LqTab indiziert)

figure('Name','Lq(id,iq) - Contour');
contourf(ID, IQ, LqTab, 20, 'LineColor','none');
xlabel('i_d [A]'); ylabel('i_q [A]');
title('L_q Kennfeld'); colorbar;

figure('Name','Ld(id,iq) - Contour');
contourf(ID, IQ, LdTab, 20, 'LineColor','none');
xlabel('i_d [A]'); ylabel('i_q [A]');
title('L_d Kennfeld'); colorbar;