% ClosedLoopControl
FOC.Flag.ClosedLoopControl = 1;   % 1= id/iq; Control 0 = Ud/Uq

% TorqueControl
FOC.Flag.TorqueControl = 1;   % 1= Torque reference; 0 = Current reference

% PWMEnable
FOC.Flag.PWMEnable = 1;   % 1= PWM On; 0 = PWM Off 

% ResetControl
FOC.Flag.ResetControl = 0;   % 

% ControlStrategy
FOC.Flag.ControlStrategy = 2;   % % 0 = KP KI 1= Kr Tn 2 = BO 

% Speed Control
FOC.Flag.SpeedControl = 1; % 1= Speed Control Active; 0= Speed Control Deactive 