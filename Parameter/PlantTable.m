Index = 1;
for d=1:length(Plant.MG_LUT.id_vec)
    for q=1:length(Plant.MG_LUT.iq_vec)
        Plant.id(Index,1) = Plant.MG_LUT.id_vec(d);
        Plant.iq(Index,1) = Plant.MG_LUT.iq_vec(q);
        Plant.fluxd(Index,1) = Plant.MG_LUT.IdIq_Fluxd_POS_grid(q,d);
        Plant.fluxq(Index,1) = Plant.MG_LUT.IdIq_Fluxq_POS_grid(q,d);
        Index = Index + 1;
    end
end
clear Index d q

Plant.Fluxd_vec = linspace(min(Plant.fluxd),max(Plant.fluxd),201);
Plant.Fluxq_vec = linspace(min(Plant.fluxq),max(Plant.fluxq),199);
Plant.FluxdFluxq_Id_grid = gridfit(Plant.fluxd,Plant.fluxq,Plant.id,Plant.Fluxd_vec,Plant.Fluxq_vec);
Plant.FluxdFluxq_Iq_grid = gridfit(Plant.fluxd,Plant.fluxq,Plant.iq,Plant.Fluxd_vec,Plant.Fluxq_vec);

Plant.SimScapeTable = load("PMSM_tables.mat");
Plant.FeedForward = Plant.SimScapeTable;

