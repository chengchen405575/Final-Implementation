
% %% Feedforward Control Data
% % BEMF PI Current Control
Plant.AINV = sqrt(2/3); % Factor for Amplitudeninvariant id_IANV = id_PINV*sqrt(2/3)

Plant.MG_LUT = load('MG_LUT.mat');
Plant.MG_LUT.id_vec = Plant.MG_LUT.id_vec*Plant.AINV;
Plant.MG_LUT.iq_vec = Plant.MG_LUT.iq_vec*Plant.AINV;
Plant.MG_LUT.IdIq_Fluxd_POS_grid = Plant.MG_LUT.IdIq_Fluxd_POS_grid*Plant.AINV;
Plant.MG_LUT.IdIq_Fluxq_POS_grid = Plant.MG_LUT.IdIq_Fluxq_POS_grid*Plant.AINV;



% 
% % Feedforward Control Lookup Table
FOC.Control.MTPA_MTPV = load('MG_CTRL_450Arms_400VDC_M0.8_SVPWM.mat');
FOC.Control.MTPA_MTPV.nT_id_grid= FOC.Control.MTPA_MTPV.nT_id_grid*Plant.AINV;
FOC.Control.MTPA_MTPV.nT_iq_grid= FOC.Control.MTPA_MTPV.nT_iq_grid*Plant.AINV;
FOC.Control.MTPA_MTPV.w_vec = (FOC.Control.MTPA_MTPV.n_vec*2*pi)/(60);

