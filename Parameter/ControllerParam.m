
%% Sample Time & Time Constant Settings
FOC.PWM_frequency 	= 10e3;    			%Hz     // converter s/w freq
FOC.SampleTime.TsCur = 1/FOC.PWM_frequency; %          % Sample Time Current Control
FOC.SampleTime.TsSpeed = 1e-03;     % Sample Time Speed Control
FOC.SampleTime.BSW.PulseGenFrequency = 200*FOC.PWM_frequency; % Sample Time Pulse Generation
FOC.SampleTime.BSW.DeadTime = 500*1e-8; % DeadTime
FOC.SampleTime.BSW.DeadTimeSample = FOC.SampleTime.BSW.DeadTime/10;

% Simulink/Plant Sample Time
SimulinkTs = 1/FOC.SampleTime.BSW.PulseGenFrequency; 


%% PI Settings
% Betragsoptimum - PI-Based
FOC.Control.Current.PILUT= load('LdLqLut.mat');
% FOC.Control.Current.TclCur = 1e-03;  % desired closed loop current control time constant
FOC.Control.Current.TclCur = 0.1;  % desired closed loop current control time constant

FOC.Control.Current.KPiq = Plant.Motor.Lq/(Plant.Motor.Rs*FOC.Control.Current.TclCur);
% FOC.Control.Current.KPiqMat = FOC.Control.Current.PILUT.Lq_lut./(Plant.Motor.Rs*FOC.Control.Current.TclCur);
% FOC.Control.Current.KIiq = Plant.Motor.Rs/FOC.Control.Current.TclCur;
FOC.Control.Current.Lq_By_Rs = FOC.Control.Current.PILUT.Lq_lut./(Plant.Motor.Rs);
FOC.Control.Current.KIiq = 1/FOC.Control.Current.TclCur;

FOC.Control.Current.KPid = Plant.Motor.Ld/(Plant.Motor.Rs*FOC.Control.Current.TclCur);
% FOC.Control.Current.KPidMat = FOC.Control.Current.PILUT.Ld_lut./(Plant.Motor.Rs*FOC.Control.Current.TclCur);
% FOC.Control.Current.KIid = Plant.Motor.Rs/FOC.Control.Current.TclCur;
FOC.Control.Current.Ld_By_Rs = FOC.Control.Current.PILUT.Ld_lut./(Plant.Motor.Rs);
FOC.Control.Current.KIid = 1/FOC.Control.Current.TclCur;


FOC.Control.Current.Krq = 0.8;
FOC.Control.Current.Tnq = 0.1;
FOC.Control.Current.Krd = 0.8;
FOC.Control.Current.Tnd = 0.1;




%% Speed Controller
% Symmetrisches Optimum
FOC.Control.Speed.PhiR = 60 *(pi/180); % Phasenreserve
FOC.Control.Speed.alpha = (1+sin(FOC.Control.Speed.PhiR))/cos(FOC.Control.Speed.PhiR);
FOC.Control.Speed.Te = FOC.Control.Current.TclCur+FOC.SampleTime.TsSpeed;
FOC.Control.Speed.Tns = FOC.Control.Speed.alpha^2*FOC.Control.Speed.Te;
FOC.Control.Speed.KPs = 2*Plant.Motor.J/(FOC.Control.Speed.alpha*FOC.Control.Speed.Te);
FOC.Control.Speed.KIs = FOC.Control.Speed.KPs/FOC.Control.Speed.Tns;



%% Reference
% OpenLoop
FOC.Control.OpenLoop.Uq_OpenLoop_V = 0; % V
FOC.Control.OpenLoop.Ud_OpenLoop_V = 0; % V

% Current
FOC.Control.OpenLoop.Iq_Ref_A = 0; % A
FOC.Control.OpenLoop.Id_Ref_A = 0; % A 

% Torque
FOC.Control.TorqueRef_Nm = 10; % Nm

% Speed 
FOC.Control.SpeedRef_rpm = 9000; %rpm

%% Raising Fallin Rates
FOC.Control.RF_RateTrq_Nm = 500; %Nm
FOC.Control.RF_RateSpeed_rpm = 2000; % rpm
FOC.Control.RF_RateCurrent_A = 500; % A
FOC.Control.RF_omegamech_rad_s = 2*pi*30000/60; % rad/s 


%% Plant Parameter
FOC.Control.InverterVdc_V = Plant.Inverter.V_dc; 
FOC.Control.MotorImax_A = Plant.Motor.Imax;
FOC.Control.Umax_V = Plant.Inverter.Umax;
FOC.Control.U_fb = FOC.Control.Umax_V;

%% Offsets
FOC.Control.theta_mechOffsetFix = 0; % normized fix 
FOC.Control.theta_mechOffsetTime= 0; % depending on speed

%% Feedforward Constant
FOC.Plant.Motor.Ld       = Plant.Motor.Ld;             %H          // D-axis inductance value
FOC.Plant.Motor.Lq       = Plant.Motor.Lq;             %H          // Q-axis inductance value
