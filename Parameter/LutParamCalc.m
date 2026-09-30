% LUT_ParamCalc
% Park/Clarke
FOC.LUT.NumOfPoints = 1024;
FOC.LUT.ParkClake.theta_Vec = 0:2*pi/(FOC.LUT.NumOfPoints-1):2*pi;
FOC.LUT.ParkClake.SinLut = sin(FOC.LUT.ParkClake.theta_Vec);
FOC.LUT.ParkClake.CosLut = cos(FOC.LUT.ParkClake.theta_Vec);
FOC.LUT.ParkClake.AtanxValue = 0:10/FOC.LUT.NumOfPoints:10;
FOC.LUT.ParkClake.AtanLut = atan(FOC.LUT.ParkClake.AtanxValue)/4;

FOC.LUT.DQLimiter.SqrtLutx = 0:1/(FOC.LUT.NumOfPoints-1):1;
FOC.LUT.DQLimiter.SqrtLuty = sqrt(FOC.LUT.DQLimiter.SqrtLutx*Plant.Inverter.Umax^2);
FOC.LUT.DQLimiter.DQ_LUTScaling= 1/FOC.Control.Umax_V^2;

FOC.LUT.ModIndex.SqrtLutx = 0:1/(FOC.LUT.NumOfPoints-1):3;
FOC.LUT.ModIndex.SqrtLuty = sqrt(FOC.LUT.ModIndex.SqrtLutx);