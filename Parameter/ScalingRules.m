% Scaling rules for the FOC parameter struct.
% Each row maps one FOC parameter path to its Simulink data type.
% PathPattern supports "*" wildcards.

if strcmp(FOC.DataType,'fixpoint')
RoughScaling = fixdt(1,32,10);
SIScaling = fixdt(1,32,18);
HighScaling = fixdt(1, 32, 27); 
UHighScaling = fixdt(0, 32, 27);  
FineScaling = fixdt(1,32,30);
PWMGenSacling = fixdt(0,32,31);
else
RoughScaling = 'double';
SIScaling = 'double';
HighScaling = 'double'; 
UHighScaling = 'double';  
FineScaling = 'double';
PWMGenSacling = 'double';
end
scalingTable = {
    % Current controller
    'FOC.Control.Current.TclCur',              'fixdt(0,32,31)';  % s
    'FOC.Control.Current.KPiq',                'fixdt(1,32,18)';
    'FOC.Control.Current.KPiqMat',             'fixdt(1,32,18)';
    'FOC.Control.Current.KIiq',                'fixdt(1,32,18)';
    'FOC.Control.Current.KPid',                'fixdt(1,32,18)';
    'FOC.Control.Current.KPidMat',             'fixdt(1,32,18)';
    'FOC.Control.Current.KIid',                'fixdt(1,32,18)';
    'FOC.Control.Current.Krq',                 'fixdt(1,32,18)';
    'FOC.Control.Current.Tnq',                 'fixdt(1,32,18)';
    'FOC.Control.Current.Krd',                 'fixdt(1,32,18)';
    'FOC.Control.Current.Tnd',                 'fixdt(1,32,18)';
    'FOC.SampleTime.TsCur',                    'fixdt(0,32,32)';

    % Current controller LUT
    'FOC.Control.Current.PILUT.Ld_lut',        'fixdt(0,32,32)';
    'FOC.Control.Current.PILUT.Lq_lut',        'fixdt(0,32,32)';
    'FOC.Control.Current.PILUT.iq_bp',         'fixdt(1,32,18)';
    'FOC.Control.Current.PILUT.id_bp',         'fixdt(1,32,18)';

    % Speed controller
    'FOC.Control.Speed.PhiR',                  'fixdt(0,32,28)';  % rad
    'FOC.Control.Speed.alpha',                 'fixdt(0,32,28)';
    'FOC.Control.Speed.Te',                    'fixdt(0,32,28)';  % s
    'FOC.Control.Speed.Tns',                   'fixdt(0,32,28)';  % s
    'FOC.Control.Speed.KPs',                   'fixdt(1,32,18)';
    'FOC.Control.Speed.KIs',                   'fixdt(1,32,18)';
    'FOC.SampleTime.TsSpeed',                  'fixdt(0,32,32)';

    % Open-loop and references
    'FOC.Control.OpenLoop.Uq_OpenLoop_V',      'fixdt(1,32,18)';  % V
    'FOC.Control.OpenLoop.Ud_OpenLoop_V',      'fixdt(1,32,18)';  % V
    'FOC.Control.OpenLoop.Iq_Ref_A',           'fixdt(1,32,18)';  % A
    'FOC.Control.OpenLoop.Id_Ref_A',           'fixdt(1,32,18)';  % A
    'FOC.Control.TorqueRef_Nm',                'fixdt(1,32,18)';  % Nm
    'FOC.Control.SpeedRef_rpm',                'fixdt(1,32,0)';   % rpm

    % Rate limiters
    'FOC.Control.RF_RateTrq_Nm',               'fixdt(1,32,18)';  % Nm/s
    'FOC.Control.RF_RateSpeed_rpm',            'fixdt(1,32,0)';   % rpm/s
    'FOC.Control.RF_RateCurrent_A',            'fixdt(1,32,18)';  % A/s
    'FOC.Control.RF_omegamech_rad_s'          'fixdt(1,32,18)';  % rad/s

    % Plant values used by FOC
    'FOC.Control.InverterVdc_V',               'fixdt(1,32,18)';  % V
    'FOC.Control.MotorImax_A',                 'fixdt(1,32,18)';  % A
    'FOC.Control.Umax_V',                      'fixdt(1,32,18)';  % V
    'FOC.Control.U_fb',                        'fixdt(1,32,18)'; % V
    'FOC.Plant.Motor.Ld',                      'fixdt(0,32,32)'; % H             
    'FOC.Plant.Motor.Lq',                       'fixdt(0,32,32)'; % H  

    % MTPA / MTPV feed-forward LUT
    'FOC.Control.MTPA_MTPV.w_vec',             'fixdt(1,32,18)';
    'FOC.Control.MTPA_MTPV.T_vec',             'fixdt(1,32,18)';
    'FOC.Control.MTPA_MTPV.nT_id_grid',        'fixdt(1,32,18)';
    'FOC.Control.MTPA_MTPV.nT_iq_grid',        'fixdt(1,32,18)';

    % Park/Clarke LUT
    'FOC.LUT.ParkClake.theta_Vec',             'fixdt(0,16,13)';  % rad
    'FOC.LUT.ParkClake.SinLut',                'fixdt(1,16,14)';
    'FOC.LUT.ParkClake.CosLut',                'fixdt(1,16,14)';

    % DQ limiter LUT
    'FOC.LUT.DQLimiter.SqrtLutx',              'fixdt(0,16,15)';
    'FOC.LUT.DQLimiter.SqrtLuty',              'fixdt(0,16,8)';
    'FOC.LUT.DQLimiter.DQ_LUTScaling',         'fixdt(0,32,30)';
    
    % Modulation Index Sqrt LUT
    'FOC.LUT.ModIndex.SqrtLutx',              'fixdt(0,16,13)';
    'FOC.LUT.ModIndex.SqrtLuty',              'fixdt(0,16,13)';

    % Flags
    'FOC.Flag.ClosedLoopControl',               'boolean';
    'FOC.Flag.TorqueControl',                   'boolean';
    'FOC.Flag.PWMEnable',                       'boolean';
    'FOC.Flag.ResetControl',                    'boolean';
    'FOC.Flag.ControlStrategy',                 'uint8';
    'FOC.Flag.SpeedControl',                    'boolean';

    % Offsets
    'FOC.Control.theta_mechOffsetFix',          'fixdt(0,32,32)';
    'FOC.Control.theta_mechOffsetTime',         'fixdt(0,32,32)';

    % PWM
    'FOC.PWM.HybridMode',                       'uint8';
    'FOC.PWM.PWMManuel',                        'uint8';
    'FOC.PWM.AZSPWMMethod',                     'uint8';
    'FOC.PWM.RSPWMMethod',                      'uint8';
    'FOC.PWM.LB1',                              'fixdt(0,32,31)';
    'FOC.PWM.UB1',                              'fixdt(0,32,31)';
    'FOC.PWM.LB2',                              'fixdt(0,32,31)';
    'FOC.PWM.UB2',                              'fixdt(0,32,31)';
    };

rules = cell2struct(scalingTable, {'PathPattern', 'DataType'}, 2);

clear scalingTable;

