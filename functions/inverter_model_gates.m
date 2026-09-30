function [v_ph, v_cmv, v_legN, v_err] = inverter_model_gates(g_H, g_L, i_ph, U_dc)
%#codegen
% INVERTER_MODEL_GATES  Zweistufiger B6-Wechselrichter, zustandslos.
%
%   Erwartet die sechs Gate-Signale, in denen die Totzeit bereits
%   enthalten ist (waehrend der Totzeit sind g_H(k) und g_L(k) beide 0).
%   Dadurch benoetigt der Block keinen internen Zaehler und keine
%   Rueckfuehrung -- er ist rein kombinatorisch.
%
%   Eingaenge:
%     g_H   3x1 logical  Gate oberer Schalter je Bein
%     g_L   3x1 logical  Gate unterer Schalter je Bein
%     i_ph  3x1 double   Phasenstroeme [A], Zaehlpfeil aus dem Bein heraus
%     U_dc  1x1 double   Zwischenkreisspannung [V]
%
%   Ausgaenge:
%     v_ph   3x1 double  Strangspannungen gegen Sternpunkt [V]
%     v_cmv  1x1 double  Gleichtaktspannung gegen DC-Mittelpunkt [V]
%     v_legN 3x1 double  Beinpotenziale gegen DC-Minus [V]
%     v_err  3x1 double  Abweichung vom idealen Kommandowert [V] (Diagnose)

% ------------------------------------------------------------------
% Parameter
% ------------------------------------------------------------------
U_CE = 1.5;    % IGBT-Durchlassspannung [V]  (0 -> idealer Schalter)
U_F  = 1.2;    % Diodenflussspannung [V]     (0 -> ideale Diode)

v_legN    = zeros(3,1);
v_leg_cmd = zeros(3,1);

for k = 1:3

    ik = i_ph(k);
    hi = g_H(k);
    lo = g_L(k);

    if hi && ~lo
        % ===== Oberer Schalter eingeschaltet =====
        if ik >= 0
            v_legN(k) = U_dc - U_CE;    % oberer IGBT leitet
        else
            v_legN(k) = U_dc + U_F;     % obere Diode leitet
        end
        v_leg_cmd(k) = U_dc;

    elseif ~hi && lo
        % ===== Unterer Schalter eingeschaltet =====
        if ik > 0
            v_legN(k) = -U_F;           % untere Diode leitet
        else
            v_legN(k) = U_CE;           % unterer IGBT leitet
        end
        v_leg_cmd(k) = 0;

    elseif ~hi && ~lo
        % ===== TOTZEIT: beide Gates aus, Diode entscheidet =====
        if ik > 0
            v_legN(k) = -U_F;           % untere Freilaufdiode
            v_leg_cmd(k) = 0;
        else
            v_legN(k) = U_dc + U_F;     % obere Freilaufdiode
            v_leg_cmd(k) = U_dc;
        end

    else
        % ===== Beide Gates an: Bruecken-Kurzschluss =====
        % Physikalisch ein Fehlerfall. Hier als halbe Zwischenkreis-
        % spannung modelliert, damit die Simulation nicht divergiert.
        v_legN(k)    = 0.5 * U_dc;
        v_leg_cmd(k) = 0.5 * U_dc;
    end
end

% ------------------------------------------------------------------
% Sternpunktpotenzial, Strangspannungen, Gleichtaktspannung
% ------------------------------------------------------------------
% Symmetrische Last mit isoliertem Sternpunkt: Summe der Strangspannungen
% ist null, daraus folgt v_nN als Mittelwert der Beinpotenziale.
v_nN  = (v_legN(1) + v_legN(2) + v_legN(3)) / 3;

v_ph  = v_legN - v_nN;              % treiben die Maschine
v_cmv = v_nN - 0.5 * U_dc;          % gegen DC-Mittelpunkt

v_err = v_legN - v_leg_cmd;         % Diagnose: Abweichung vom Ideal

end