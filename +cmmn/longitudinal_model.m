function [ model ] = longitudinal_model(Ts)
%LONGITUDINAL_MODEL Returns state space model for one vehicle

A = [    0.0253   -0.0831   -0.0131   -0.0113;
    0.5431   -2.3698    1.1077   -0.7262;
   -0.8098    1.0431    4.5808    5.7846;
    2.0996   -2.5170  -11.2614   -9.0554];
B = [   -0.0043;
   -0.2857;
    0.9264;
   -1.8387];
C = [  121.8504   -1.1452    0.0361   -0.1053;
    2.2046   -7.5476   -1.7323    0.5725];
D = 0;
vehicle_model_ss = ss(A, B, C, D);

% Convert to discrete time
model = c2d(vehicle_model_ss,Ts);
% Absorb delay into system dynamics
model = absorbDelay(model);
model.InputName = 'VelocityIn';
model.InputUnit = 'm/s';
model.OutputName{1} = 'Distance';
model.OutputUnit{1} = 'm';
model.OutputName{2} = 'Velocity';
model.OutputUnit{2} = 'm/s';
end