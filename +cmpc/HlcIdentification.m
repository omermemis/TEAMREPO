classdef HlcIdentification < cmmn.InterfaceHlc
% HlcIdentification implements an HLC for model identification
    properties
        mt
        Ts
        path_points
        input
        s_ref
        output
        counter
        t
        HP
        HU
        mpcObj
        s0
        DUMIN
        DUMAX
        v_max
        v_min
        slack_var
        delta_u
        s_max
        val_objective_fcn % objective function
        nVeh % number of vehicles
        ny % output length
        nu % input length
        nx % number of states
        d_ref % relative distance reference
        v_ref % reference speed
        a_min % min accelelation constraint
        a_max % max accelelation constraint
        d_min % min relative distance constraint
        d_max % max relative distance constraint
        vehicle_ids
        v_ref_fuc % reference function
        path_save_results 
    end

    methods
        function obj = HlcIdentification(Ts,vehicle_ids)
            obj = obj@cmmn.InterfaceHlc(vehicle_ids);
            obj.vehicle_ids = vehicle_ids;

            obj.Ts = Ts;

            obj.nVeh = numel(vehicle_ids);
%             assert(obj.nVeh==1);

            obj.path_points = cmmn.outer_lane_path();
            obj.mt = cmmn.MeasurementTransformer(obj.path_points,vehicle_ids);
            obj.input = [];
            obj.output = [];
            obj.t = [];
            obj.HP = 25;
            obj.HU = 12;
            obj.v_min = 0;
            obj.v_max = 1.5;
            obj.d_ref = 0.5;
            obj.a_min = -1;
            obj.a_max = 0.5;
            obj.d_min = 0.3;
            obj.d_max = inf;
%             MODEL = cmmn.longitudinal_model(obj.Ts);
            MODEL = cmpc.central_model(obj.Ts,obj.nVeh);
            obj.ny = size(MODEL.C,1);
            obj.nu = size(MODEL.B,2);
            obj.nx = size(MODEL.A,2);
            UMIN = zeros(obj.nu,1); % min. input constraint
            UMAX = 1.5*ones(obj.nu,1); % max. input constraint
            obj.DUMIN = obj.a_min*obj.Ts; % max. acceleration constraint
            obj.DUMAX = obj.a_max*obj.Ts; % min. acceleration constraint
            YMIN = zeros(obj.ny,1); % min. output constraint
            YMIN(1:end-1) = obj.d_min;
            YMIN(end) = obj.v_min;
            YMIN = repmat(YMIN,obj.HP,1);
            YMAX = zeros*ones(obj.ny,1); % max. output constraint
            YMAX(1:end-1) = obj.d_max;
            YMAX(end) = obj.v_max;
            YMAX = repmat(YMAX,obj.HP,1);
            Q = eye(obj.ny); % for system output
%             Q = diag([1,2,3,4,5]);
%             Q(5,5) = 0.1; 
            R = .1*zeros(obj.nu); % for input changes du
            Q_KALMAN = eye(obj.nx); % handle process noise
            R_KALMAN = eye(obj.ny); % handle measurement noise
            obj.mpcObj = cmmn.ModelPredictiveControl(MODEL,obj.HP,obj.HU,UMIN,UMAX,obj.DUMIN,obj.DUMAX,YMIN,YMAX,Q,R,Q_KALMAN,R_KALMAN);

        end
        
        function on_first_timestep(obj, vehicle_state_list)
            on_first_timestep@cmmn.InterfaceHlc(obj,vehicle_state_list);
            obj.counter = 0;
            obj.t_exp = 0.1;
            x_init = zeros(obj.nx,1);
            obj.mpcObj.observer.x_k_minus_one = x_init;
            obj.v_ref_fuc = @(t) 0.5*(0<=t & t<15) + 1.4*(15<=t & t<25) + 0.8*(25<=t & t<35); % reference speed function
%             y1 = obj.mt.measure_longitudinal(vehicle_state_list);
%             obj.s0 = y1(1);
            new_folder_name = ['HP' num2str(obj.HP) '-HU' num2str(obj.HU) '-nVeh' num2str(obj.nVeh) '-Ts' num2str(obj.Ts)];
            obj.path_save_results = ['assets/saved/cmpc/' new_folder_name];
        end

        function on_each_timestep(obj, vehicle_state_list)
            obj.counter = obj.counter + 1;
            on_each_timestep@cmmn.InterfaceHlc(obj,vehicle_state_list);

            % pose to distance on path f/e veh
%             ym = obj.mt.measure_longitudinal(vehicle_state_list);
            ym = obj.mt.measure_central(vehicle_state_list);
            disp(ym)
            
            ref = zeros(obj.ny*obj.HP,1); % reference for y over the prediction horizon, dimensions=(ny*Hp,1). If dimensions=(ny,1), vector will be repeated Hp times.
%             obj.v_ref(obj.counter,1) = v_ref_;
%             for i=1:obj.ny*obj.HP
%                 ref(2*i-1) = s_ref_func(obj.t_exp+i*obj.Ts) + obj.s0;
%                 ref(2*i) = v_ref_;
%             end
%             obj.v_ref = 0.5*(0<=obj.t_exp & obj.t_exp<15) + 1.4*(15<=obj.t_exp & obj.t_exp<25) + 0.8*(25<=obj.t_exp & obj.t_exp<35);
            for i=1:obj.HP
                ref((i-1)*obj.ny+1:i*obj.ny-1)=obj.d_ref;
                ref(i*obj.ny)=obj.v_ref_fuc(obj.t_exp+(i-1)*obj.Ts);
            end
%             ref(1:end-1) = obj.d_ref; % [m]
%             ref(end) = obj.v_ref; % [m/s]
            weight_slack_var = 1e5;
            % use only one slack variable
%             [u,y,obj.slack_var(obj.counter,1),obj.delta_u(obj.counter,1:obj.nu),obj.val_objective_fcn(obj.counter,1)] = obj.mpcObj.step(ym,ref,weight_slack_var);
%             use many slack variables
            [u,y,obj.slack_var(obj.counter,1:obj.ny),obj.delta_u(obj.counter,1:obj.nu),obj.val_objective_fcn(obj.counter,1)] = obj.mpcObj.step(ym,ref,weight_slack_var);
            % Compute control action
            % ----------------------

%             if (obj.t_exp > 18)
%                 u = 1;  
%             elseif (obj.t_exp > 14)
%                 u = 1.5;  
%             elseif(obj.t_exp > 10)
%                 u = 1;
%             elseif(obj.t_exp > 6)
%                 u = 1.5;
%             elseif(obj.t_exp > 2)
%                 u = 1;
%             else
%                 u = 0;
%             end


            
            obj.input(obj.counter,1:obj.nu) = u';
            obj.output(obj.counter,1:obj.ny) = ym';
            obj.t(obj.counter,1) = obj.t_exp;
            % Apply control action
            % --------------------
            obj.cpmLab.apply_path_tracking(u, obj.path_points, obj.dt_valid_after_nanos)
        end

        function on_stop(obj)
            on_stop@cmmn.InterfaceHlc(obj);
            % TODO plot results, see plot_platooning.m
            cmmn.plot_platooning(obj.vehicle_ids,getenv('DDS_DOMAIN'),obj.path_save_results)
%               figure(1)
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
%               subplot(5,1,4)
%               plot(obj.t, obj.slack_var)
%               grid on
%               xlabel('t [s]')
%               ylabel('slack variable')
%               subplot(5,1,5)
%               plot(obj.t, obj.val_objective_fcn)
%               grid on
%               xlabel('t [s]')
%               ylabel('objective function') 

                
        end

    end
end
