function [ model ] = central_model(Ts, n)
%LONGITUDINAL_MODEL Returns state space model for one vehicle
%n: number of vehicles
% longitudinal_model = cmmn.longitudinal_model(Ts);
% A_ = longitudinal_model.A;
% B_ = longitudinal_model.B;
% C_ = longitudinal_model.C;
A_ = [0.00656635216633482,-0.0322620913894631,-0.0160034194253441,0.0311973429453407;
    0.768336436243678,-2.50314801667369,-1.70181706723765,2.90410070226302;
    -0.283661453946157,3.94689676287932,0.380503701605602,0.884714765658832;
    0.322096197063316,-3.85525962499882,0.659033648930461,-6.52878221826174];
B_ = [-0.000116100137588649;
    0.642124608353234;
    -1.13948605776869;
    1.08827063403906];
C_ = [-108.744371203648,0.00926272109770916,-0.0408323254687915,0.0187161719673953;
    -0.300014292260243,3.54825363303500,1.25787139468587,-3.23416122290576];


order = size(A_,1);
A =  zeros(n*order,n*order);
B =  zeros(n*order,n);
C =  zeros(n,n*order);

for i=1:n
    A((i-1)*order+1:i*order,(i-1)*order+1:i*order) = A_;
    B((i-1)*order+1:i*order,i) = B_;
%     if i==1
%         C(i,(i-1)*order+1:i*order) = C_(2,:);
%     else
%         C(i,(i-2)*order+1:(i-1)*order) = C_(1,:);
%         C(i,(i-1)*order+1:i*order) = -C_(1,:);
%     end
    if i==n
        C(i,(i-1)*order+1:i*order) = C_(2,:);
    else
        C(i,(i-1)*order+1:i*order) = -C_(1,:);
        C(i,i*order+1:(i+1)*order) = C_(1,:); % output y = [d21; d32...; dn(n-1); vn], where dn(n-1)=distance(n)-distance(n-1)
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