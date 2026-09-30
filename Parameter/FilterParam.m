
% Filter for Ref Tracking 
Filter.Torque.TFiltRefTr = 0.005;
Filter.Torque.RefTr = tf(1,[Filter.Torque.TFiltRefTr 1])*tf(1,[Filter.Torque.TFiltRefTr 1]);
Filter.Torque.RefTrDis = c2d(Filter.Torque.RefTr,SimulinkTs,'tustin');
Filter.Torque.RefTrNum = Filter.Torque.RefTrDis.Numerator{1};
Filter.Torque.RefTrDen = Filter.Torque.RefTrDis.Denominator{1};

% Filter for CurrentFilt Tracking 
Filter.Current.Filterw0 = 30e03;
FOC.Filter.Current.FilterTimeConst = 1/Filter.Current.Filterw0;
Filter.Current.Filter  = tf(1,[1/Filter.Current.Filterw0 1]);
Filter.Current.DisFilter = c2d(Filter.Current.Filter,SimulinkTs,'tustin');
Filter.Current.DisFilterNum = Filter.Current.DisFilter .Numerator{1};
Filter.Current.DisFilterDen = Filter.Current.DisFilter .Denominator{1};

Filter.Current.FilterDecoupw0 = 200;
Filter.Current.FilterDecoup  = tf(1,[1/Filter.Current.FilterDecoupw0 1]);
Filter.Current.DisFilterDecoup = c2d(Filter.Current.FilterDecoup,FOC.SampleTime.TsCur,'tustin');
Filter.Current.DisFilterNumDecoup = Filter.Current.DisFilterDecoup .Numerator{1};
Filter.Current.DisFilterDenDecoup = Filter.Current.DisFilterDecoup .Denominator{1};

% Filter for CurrentFilt Tracking 
Filter.Current.Sensorw0 = 2*pi*100e03;
FOC.Filter.Current.SensorTimeConst = 1/Filter.Current.Sensorw0;
% Filter.Current.Sensor  = tf(1,[1/Filter.Current.Sensorw0 1])*tf(1,[1/Filter.Current.Sensorw0 1]);
% Filter.Current.DisSensor = c2d(Filter.Current.Sensor,SimulinkTs,'tustin');
% Filter.Current.DisSensorNum = Filter.Current.DisSensor .Numerator{1};
% Filter.Current.DisSensorDen = Filter.Current.DisSensor .Denominator{1};


Filter.Current.Sensor  = tf(1,[1/Filter.Current.Sensorw0 1]);
Filter.Current.DisSensor = c2d(Filter.Current.Sensor,SimulinkTs,'tustin');
Filter.Current.DisSensorNum = Filter.Current.DisSensor .Numerator{1};
Filter.Current.DisSensorDen = Filter.Current.DisSensor .Denominator{1};


