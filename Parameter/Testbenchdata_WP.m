%% MotorParam        
Plant.Motor.p        = 4;                  %           // Pole Pairs for the motor
Plant.Motor.Rs       = 0.36;           %Ohm        // Stator Resistor
Plant.Motor.Ld       = 0.2e-3;             %H          // D-axis inductance value
Plant.Motor.Lq       = 0.2e-3;             %H          // Q-axis inductance value
Plant.Motor.J        = 20e-06;  %Kg-m2      
Plant.Motor.B        = 0.0001386;  %Kg-m2/s    
Plant.Motor.Imax = 7.1; % A

Plant.Motor.FluxPM= 6.4e-3;
%% InverterParam
Plant.Inverter.V_dc          = 24;       				%V      // DC Link Voltage of the Inverter
Plant.Inverter.Umax = (Plant.Inverter.V_dc/sqrt(3));
Plant.Power= Plant.Inverter.V_dc*Plant.Motor.Imax;
Plant.TMax = 0.25;



