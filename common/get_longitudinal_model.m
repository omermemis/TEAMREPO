function [ model ] = get_longitudinal_model(Ts)

A = 0;
B = 0;
C = 0;
D = 0;

vehicle_model_ss = ss(A, B, C, D);
vehicle_model_ss.InputDelay = 0;

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

