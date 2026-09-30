%% MotorParam        
Plant.Motor.p        = 4;                  %           // Pole Pairs for the motor
Plant.Motor.Rs       = 3.35e-03;           %Ohm        // Stator Resistor
Plant.Motor.Ld       = 6.8402e-05;             %H          // D-axis inductance value
Plant.Motor.Lq       = 1.9656e-04;             %H          // Q-axis inductance value
Plant.Motor.J        = 0.08;  %Kg-m2      
Plant.Motor.B        = 0.0001386;  %Kg-m2/s    
Plant.Motor.Imax = 650; % A

Plant.Motor.FluxPM= 27.6/(2*pi*4*(1200/60))*sqrt(2/3);
%% InverterParam
Plant.Inverter.V_dc          = 400;       				%V      // DC Link Voltage of the Inverter
Plant.Inverter.Umax = (Plant.Inverter.V_dc/sqrt(3));
Plant.Power= 150e03;

%% High-Voltage System Parameters
Plant.Inverter.Cdc  = 0.001;      % DC-link capacitor      [F]
Plant.Inverter.Vnom = 325;        % Nominal DC voltage     [V] 
V1   = 290;        % Voltage V1(< Vnom)     [V]
Plant.Inverter.AH0  = 400;        % Initial battery charge [hr*A]

