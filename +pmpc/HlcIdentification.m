classdef HlcIdentification < cmmn.InterfaceHlc
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
        DUMIN
        DUMAX
        v_max
        v_min
        slack_var
        delta_u
        s_max
        val_objective_fcn
    end

    methods
        function obj = HlcIdentification(Ts,vehicle_ids)
            obj = obj@cmmn.InterfaceHlc(vehicle_ids);

            obj.Ts = Ts;

            nVeh = numel(vehicle_ids);
            assert(nVeh==1);

            obj.path_points = cmmn.outer_lane_path();
            obj.mt = cmmn.MeasurementTransformer(obj.path_points,vehicle_ids);
            obj.input = [];
            obj.output = [];
            obj.t = [];
            obj.HP = 10;
            obj.HU = 3;
            obj.v_max = 8;
            obj.v_min = -2;
            obj.s_max = 5;
            obj.slack_var = [];
            obj.delta_u = [];
            obj.val_objective_fcn = [];
        end
        
        function on_first_timestep(obj, vehicle_state_list)
            on_first_timestep@cmmn.InterfaceHlc(obj,vehicle_state_list);
            obj.counter = 0;
            obj.Ts = 0.1;
            obj.t_exp = 0.1;
%             MODEL = cmmn.longitudinal_model(obj.Ts);
            MODEL = cmmn.longitudinal_model(obj.Ts);
            y1 = obj.mt.measure_longitudinal(vehicle_state_list);
            obj.s0 = y1(1);
            UMIN = 0; % min. input constraint
            UMAX = inf; % max. input constraint
            obj.DUMIN = -1*20*obj.Ts; % max. acceleration constraint
            obj.DUMAX = 0.5*20*obj.Ts; % min. acceleration constraint
            YMIN = -inf*ones(2*obj.HP,1); % min. output constraint
            YMAX = inf*ones(2*obj.HP,1); % max. output constraint
            for i=1:obj.HP
                YMIN(2*i-1) = -inf;
                YMIN(2*i) = obj.v_min;
            end
            for i=1:obj.HP
                YMAX(2*i-1) = obj.s_max+obj.s0;
                YMAX(2*i) = obj.v_max;
            end
            Q = [1 0; 0 0];
            R = 0;
            Q_KALMAN = 1;
            R_KALMAN = [1 0; 0 1];
            obj.mpcObj = cmmn.ModelPredictiveControl(MODEL,obj.HP,obj.HU,UMIN,UMAX,obj.DUMIN,obj.DUMAX,YMIN,YMAX,Q,R,Q_KALMAN,R_KALMAN);
            x_init = [0;0;0;0];
            obj.mpcObj.observer.x_k_minus_one = x_init;

        end

        function on_each_timestep(obj, vehicle_state_list)
            obj.counter = obj.counter + 1;
            on_each_timestep@cmmn.InterfaceHlc(obj,vehicle_state_list);

            % pose to distance on path f/e veh
            ym = obj.mt.measure_longitudinal(vehicle_state_list);
            disp(ym)
            
            s_ref_func = @(t) 1.1*t + 0.5*sin(t) + 0;
            v_ref_ = 1;
            ref = zeros(2*obj.HP,1);
            obj.s_ref(obj.counter,1) = s_ref_func(obj.t_exp) + obj.s0;
            obj.v_ref(obj.counter,1) = v_ref_;
            for i=1:obj.HP
                ref(2*i-1) = s_ref_func(obj.t_exp+i*obj.Ts) + obj.s0;
                ref(2*i) = v_ref_;
            end
            weight_slack_var = 20;
            [u,y,obj.slack_var(obj.counter),obj.delta_u(obj.counter,1),obj.val_objective_fcn(obj.counter,1)] = obj.mpcObj.step(ym,ref,weight_slack_var);
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


            
            obj.input(obj.counter,1) = u;
            obj.output(obj.counter,1) = ym(1);
            obj.output(obj.counter,2) = ym(2);
            obj.t(obj.counter,1) = obj.t_exp;
            % Apply control action
            % --------------------
            obj.cpmLab.apply_path_tracking(u, obj.path_points, obj.dt_valid_after_nanos)
        end

        function on_stop(obj)
            on_stop@cmmn.InterfaceHlc(obj);
            % TODO plot results, see plot_platooning.m
              figure(1)
              subplot(5,1,1)
              plot(obj.t, [obj.s_ref, obj.output(:,1), (obj.s_max+obj.s0)*ones(obj.counter,1)]);
              grid on
              legend('reference distance','real distance','max distance constraint')
              ylabel('distance [m]')
              subplot(5,1,2)
              plot(obj.t, [obj.input, obj.output(:,2), obj.v_max*ones(obj.counter,1), obj.v_min*ones(obj.counter,1)]);
              grid on
              legend('input speed','real speed','max output speed constraint','min output speed constraint')
              ylabel('speed [m/s]')
              ylim([obj.v_min-0.2, obj.v_max+0.2])
              subplot(5,1,3)
              plot(obj.t(2:end), [obj.delta_u(2:end)/obj.Ts, diff(obj.output(:,2))/obj.Ts,...
                  obj.DUMAX/obj.Ts*ones(obj.counter-1,1), obj.DUMIN/obj.Ts*ones(obj.counter-1,1)])
              grid on
              legend('input acceleration','real acceleration','max acceleration constraint','min acceleration constraint')
              xlabel('t [s]')
              ylabel('acceleration [m/s^2]')
              ylim([obj.DUMIN/obj.Ts-0.2; obj.DUMAX/obj.Ts+0.2])
              subplot(5,1,4)
              plot(obj.t, obj.slack_var)
              grid on
              xlabel('t [s]')
              ylabel('slack variable')
              subplot(5,1,5)
              plot(obj.t, obj.val_objective_fcn)
              grid on
              xlabel('t [s]')
              ylabel('objective function') 
%             data = iddata(obj.output, obj.input, obj.Ts);
%             data.InputName  = {'VelocityIn'};
%             data.OutputName = {'DistanceOut';'VelocityOut'};
%             save('data.mat','data')
%             order = (1:1:10); 
%             sys = ssest(data, order);    
%             save('sys.mat','sys')
%             save('data.mat','data')
%             figure(1)
%             subplot(2,1,1)
%             plot(data)
%             subplot(2,1,2)
%             compare(data, sys)
                
        end

    end
end
