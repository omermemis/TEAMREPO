function [ model ] = central_model(Ts, n)
%LONGITUDINAL_MODEL Returns state space model for one vehicle
%n: number of vehicles
longitudinal_model = cmmn.longitudinal_model(Ts);
A_ = longitudinal_model.A;
B_ = longitudinal_model.B;
C_ = longitudinal_model.C;

order = size(A_,1);
A =  zeros(n*order,n*order);
B =  zeros(n*order,n);
C =  zeros(n,n*order);

for i=1:n
    A((i-1)*order+1:i*order,(i-1)*order+1:i*order) = A_;
    B((i-1)*order+1:i*order,i) = B_;
    if i==1
        C(i,(i-1)*order+1:i*order) = C_(2,:);
    else
        C(i,(i-2)*order+1:(i-1)*order) = C_(1,:);
        C(i,(i-1)*order+1:i*order) = -C_(1,:);
    end
end
D = 0;
networked_model_ss = ss(A, B, C, D);

% Convert to discrete time
model = c2d(networked_model_ss,Ts);
% Absorb delay into system dynamics
model = absorbDelay(model);
% model.InputName = 'VelocityIn';
% model.InputUnit = 'm/s';
% model.OutputName{1} = 'Velocity';
% model.OutputUnit{1} = 'm';
% model.OutputName{2} = 'DistanceDiff';
% model.OutputUnit{2} = 'm/s';
end