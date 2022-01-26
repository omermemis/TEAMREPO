classdef Hlc < cmmn.InterfaceHlc
% HlcIdentification implements an HLC for model identification
    properties
        mt
        Ts
        path_points
        input
        s_ref
        v_ref
        output
        counter
        t
        HP
        HU
        mpcObj
        s0
        v_max
        v_min
        slack_var
        delta_u
        s_maxdouble
        val_objective_fcn % objective function
        nVeh % number of vehicles
        ny % output length
        nu % input length
        nx % number of states
        d_ref % relative distance reference
        v_ref_ % reference speed
        a_min % min accelelation constraint
        a_max % max accelelation constraint
        d_min % min relative distance constraint
        d_max % max relative distance constraint
        vehicle_ids
        reader_vehicleOutput
        writer_vehicleOutput
        currentVehicleID
        targetVehicleID
    end

    methods
        function obj = Hlc(Ts,vehicle_ids)
            obj = obj@cmmn.InterfaceHlc(vehicle_ids);
            obj.vehicle_ids = vehicle_ids;     
            obj.Ts = Ts;

            obj.path_points = cmmn.outer_lane_path();
            obj.mt = cmmn.MeasurementTransformer(obj.path_points,vehicle_ids);
            obj.input = [];
            obj.output = [];
            obj.t = [];
            obj.HP = 10; % HP = 30 and HU =3 is the best conbination until now (see results under folder saved/dmpc/HP30-HU3)
            obj.HU = 3;
            obj.v_min = 0;
            obj.v_max = 1.5;
            obj.d_ref = 0.5;
            obj.d_min = 0.3;
            obj.a_min = -1;
            obj.a_max = 0.5;
            
            MODEL = cmmn.longitudinal_model(obj.Ts);  
            obj.ny = size(MODEL.C,1);
            obj.nu = size(MODEL.B,2);
            obj.nx = size(MODEL.A,2);
            UMIN = obj.v_min*ones(obj.nu,1); % min. input constraint
            UMAX = obj.v_max*ones(obj.nu,1); % max. input constraint
            DUMIN = obj.a_min*obj.Ts; % max. acceleration constraint
            DUMAX = obj.a_max*obj.Ts; % min. acceleration constraint  
            YMIN = repmat([-inf;-inf], obj.HP, 1);
            YMAX = repmat([inf;inf], obj.HP, 1);
            Q_leader = [0 0;0 1]; % Q-matrix of mpc for leading vehicle
            Q_others = [1 0;0 0]; % Q-matrix of mpc for other vehicles

            R = zeros(obj.nu); % for input changes du
            Q_KALMAN = eye(obj.nx); % handle process noise
            R_KALMAN = eye(obj.ny); % handle measurement noise
            
            % create mpc instances and filters for each vehicle
            obj.mpcObj = {}; % cell to store mpc instances
            matlabOutputTopicName = 'vehicleOutput'; 
            obj.writer_vehicleOutput = DDS.DataWriter(DDS.Publisher(obj.cpmLab.matlabParticipant), 'dmpc.HlcPlan', matlabOutputTopicName);
            obj.nVeh = numel(obj.vehicle_ids);
            for idx=1:obj.nVeh
                obj.currentVehicleID = obj.vehicle_ids(end-idx+1); % from higher ID to lower ID
                
                if idx==1 % leading vehicle
                    obj.reader_vehicleOutput{idx} = DDS.DataReader(DDS.Subscriber(obj.cpmLab.matlabParticipant), 'dmpc.HlcPlan', matlabOutputTopicName); % reader without filter for leading vehicle (maybe not necessary, because the leading vehicle doesn't need to read messages from other vehicles)
                    obj.reader_vehicleOutput{idx}.WaitSet = true;
                    obj.reader_vehicleOutput{idx}.WaitSetTimeout = 2; % [s]
                    obj.mpcObj{idx} = cmmn.ModelPredictiveControl(MODEL,obj.HP,obj.HU,UMIN,UMAX,DUMIN,DUMAX,YMIN,YMAX,Q_leader,R,Q_KALMAN,R_KALMAN); % mpc instance for leading vehicle
                else % other vehicles
                    obj.targetVehicleID = obj.vehicle_ids(end-idx+2); % the vehicle, which should be considered by the current vehicle 
                    Filter = DDS.contentFilter; % create an instance of Filter
                    Filter.FilterExpression = 'vehicle_id = %0';
                    Filter.FilterParameters = {num2str(obj.targetVehicleID)};
                    obj.reader_vehicleOutput{idx} = DDS.DataReader(DDS.Subscriber(obj.cpmLab.matlabParticipant), 'dmpc.HlcPlan', matlabOutputTopicName,'',Filter); % reader with filter of vehicles (except for the leading vehicle)
                    obj.reader_vehicleOutput{idx}.WaitSet = true;
                    obj.reader_vehicleOutput{idx}.WaitSetTimeout = 2; % [s]
                    obj.mpcObj{idx} = cmmn.ModelPredictiveControl(MODEL,obj.HP,obj.HU,UMIN,UMAX,DUMIN,DUMAX,YMIN,YMAX,Q_others,R,Q_KALMAN,R_KALMAN); % mpc instance for other vehicles
                end
                x_init = zeros(obj.nx,1);
                obj.mpcObj{idx}.observer.x_k_minus_one = x_init; % initial state for Kalman filter
            end  
            
        end
        
        function on_first_timestep(obj, vehicle_state_list)
            on_first_timestep@cmmn.InterfaceHlc(obj,vehicle_state_list);
            obj.counter = 0;
            obj.t_exp = 0.1;
            y1 = obj.mt.measure_longitudinal(vehicle_state_list);
            obj.s0 = y1(1);

        end

        function on_each_timestep(obj, vehicle_state_list)
            obj.counter = obj.counter + 1;
            on_each_timestep@cmmn.InterfaceHlc(obj,vehicle_state_list);

            % pose to distance on path f/e veh
%             ym = obj.mt.measure_longitudinal(vehicle_state_list);
            ym = obj.mt.measure_longitudinal(vehicle_state_list);
%             ym = [ym(1:2:end-2);ym(end-1);ym(end)]; % ym=[s1;s2;...;s(n-1);s(n);v(n)], where, for example, v(n) and s(n) is the speed and distance of the leading vehicle.
            disp(ym)
            weight_slack_var = 1e5;
            u = zeros(obj.nVeh,1); % initialize the control input for the platoon

            for idx=1:obj.nVeh
                obj.currentVehicleID = obj.vehicle_ids(end-idx+1); % from higher ID to lower ID

                if idx==1 % leading vehicle
                    ym_currentVehicle = ym(end-1:end); % measured output of the leading vehicle
                    obj.v_ref_ = 0.5*(0<=obj.t_exp & obj.t_exp<15) + 1.4*(15<=obj.t_exp & obj.t_exp<25) + 0.8*(25<=obj.t_exp & obj.t_exp<35); 
%                     obj.v_ref_ = 1.0; 
                    ref_leader = repmat([100; obj.v_ref_], obj.HP, 1); % reference for y over the prediction horizon, dimensions=(ny*Hp,1). If dimensions=(ny,1), vector will be repeated Hp times.
                    obj.output(obj.counter,idx) = ym_currentVehicle(2); % store real speed
                    obj.v_ref(obj.counter,idx) = obj.v_ref_; % store reference speed
                    [u(obj.nVeh,1), y, obj.slack_var(obj.counter,idx), obj.delta_u(obj.counter,idx), obj.val_objective_fcn(obj.counter,idx)] = obj.mpcObj{idx}.step(ym_currentVehicle,ref_leader,weight_slack_var);
    
                else % other vehicles
                    ym_currentVehicle = ym(end-2*idx+1:end-2*idx+2); % measured output of other vehicles
%                     ref = zeros(obj.ny*obj.HP,1); % reference for y over the prediction horizon, dimensions=(ny*Hp,1). If dimensions=(ny,1), vector will be repeated Hp times.
                    obj.targetVehicleID = obj.vehicle_ids(end-idx+2); % vehicle which should be considered by the current vehicle 
                    [dataRead,~,count,~] = obj.reader_vehicleOutput{idx}.take(); % take the message sent by the target vehicle
                    while(count==0) % sometimes, obj.reader_vehicleOutput{idx}.take() failed to take the message, so I use while(), until message was taken
                        [dataRead,~,count,~] = obj.reader_vehicleOutput{idx}.take(); 
                        warning('Try to read from domain again...')
                    end

                    ref_others = (dataRead.output - obj.d_ref)'; % reference trajectory for other vehicles
                    ymin = repmat(-inf, obj.HP*obj.ny, 1); % update min. output constraint
                    ymax = ref_others+obj.d_min; % update max. output constraint
                    ymax(2:2:end) = inf; 
%                     s_ref_func = @(t) 1.1*t + 0.5*sin(t) + 0;
%                     for i=1:obj.HP
%                         ref_others(i) = s_ref_func(obj.t_exp+i*obj.Ts) + obj.s0;
%                     end
%                     obj.s_ref(obj.counter,idx) = ref_others(1);
%                     obj.output(obj.counter,idx) = ym_currentVehicle;
                    [u(obj.nVeh-idx+1,1), y, obj.slack_var(obj.counter,idx), obj.delta_u(obj.counter,idx), obj.val_objective_fcn(obj.counter,idx)] = obj.mpcObj{idx}.step(ym_currentVehicle,ref_others,weight_slack_var,ymin,ymax);

                end
                    
                if idx<obj.nVeh % write data to damain if the current vehicle is not the last one
                    dataWrite = dmpc.HlcPlan; % initialize the data type to be sent
                    dataWrite.vehicle_id = uint8(obj.currentVehicleID);
                    dataWrite.output = y';
%                     if idx==1 % leading vehicle
%                         dataWrite.output = y(1:2:end)'; % leading vehicle has two outputs, we only care distance 
%                     else % other vehicles
%                         dataWrite.output = y'; % other vehicles only has one output, namely distance
%                     end
                    obj.writer_vehicleOutput.write(dataWrite);
                end
                   

            end
            
            % Control action
            obj.cpmLab.apply_path_tracking(u, obj.path_points, obj.dt_valid_after_nanos)

            obj.t(obj.counter,1) = obj.t_exp;
        end

        function on_stop(obj)
            on_stop@cmmn.InterfaceHlc(obj);
            % TODO plot results, see plot_platooning.m
            newFolderName = ['HP' num2str(obj.HP) '-HU' num2str(obj.HU)];
            pathStoreFig = ['saved/dmpc/' newFolderName];
            mkdir(pathStoreFig); % make new folder to store figures created by plot_platooning, in order to make it easier to compare the different results when tuning HP and HU
%             cmmn.plot_platooning(obj.vehicle_ids,getenv('DDS_DOMAIN'),pathStoreFig)
%               figure(10)
%               subplot(obj.nVeh,1,1)
%               plot(obj.t,[obj.v_ref(:,1) obj.output(:,1)]);
%               legend('reference speed','real speed')
%               title(['VehicleID: ' num2str(obj.vehicle_ids(obj.nVeh))])
%               for i=2:obj.nVeh
%                   subplot(obj.nVeh,1,i)
%                   plot(obj.t,[obj.s_ref(:,i) obj.output(:,i)]);
%                   legend('reference distance','real distance')
%                   title(['VehicleID: ' num2str(obj.vehicle_ids(obj.nVeh-i+1))])
%               end
%               
%               ylabel('distance')
%               subplot(5,1,1)
%               plot(obj.t, [obj.output(:,1), (obj.s_max+obj.s0)*ones(obj.counter,1)]);
%               grid on
%               legend('reference distance','real distance','max distance constraint')
%               ylabel('distance [m]')
%               subplot(5,1,2)
%               plot(obj.t, [obj.input, obj.output(:,obj.ny), obj.v_max*ones(obj.counter,1), obj.v_min*ones(obj.counter,1)]);
%               grid on
%               legend('input speed','real speed','max output speed constraint','min output speed constraint')
%               ylabel('speed [m/s]')
%               ylim([obj.v_min-0.2, obj.v_max+0.2])
%               subplot(5,1,3)
%               plot(obj.t(2:end), [obj.delta_u(2:end)/obj.Ts, diff(obj.output(:,2))/obj.Ts,...
%                   obj.DUMAX/obj.Ts*ones(obj.counter-1,1), obj.DUMIN/obj.Ts*ones(obj.counter-1,1)])
%               grid on
%               legend('input acceleration','real acceleration','max acceleration constraint','min acceleration constraint')
%               xlabel('t [s]')
%               ylabel('acceleration [m/s^2]')
%               ylim([obj.DUMIN/obj.Ts-0.2; obj.DUMAX/obj.Ts+0.2])
%               figure(20)
%               subplot(2,1,1)
%               plot(obj.t, obj.slack_var)
%               grid on
%               xlabel('t [s]')
%               ylabel('slack variable')
%               legend_ = cellstr('VehicleID: '+string(obj.vehicle_ids(end:-1:1)));
%               legend (legend_)
%               subplot(2,1,2)
%               plot(obj.t, obj.val_objective_fcn)
%               grid on
%               xlabel('t [s]')
%               ylabel('objective function') 
%               legend (legend_)
                
        end

    end
end
