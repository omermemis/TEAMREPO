classdef HlcIdentification < cmmn.InterfaceHlc
% HlcIdentification implements an HLC for model identification
    properties
        mt
        Ts
        path_points
        input
        output
        counter
        t
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
            
            
        end
        
        function on_first_timestep(obj, vehicle_state_list)
            on_first_timestep@cmmn.InterfaceHlc(obj,vehicle_state_list);
            obj.counter = 0;
            MODEL = cmmn.longitudinal_model(obj.Ts);
            HP = 10;
            HU = 3;
            UMIN = 0;
            UMAX = 1.5;
            DUMIN = -1;
            DUMAX = 0.5;
            YMIN = zeros(2*HP,1);
            s0 = 10;
            YMAX = [s0, UMAX];
            for i =1:HP-1
                YMAX(2*i+1) = YMAX(2*i-1) + obj.t_exp*UMAX;
                YMAX(2*i+2) = UMAX;
            end
            YMAX = YMAX';
            Q = [1 0; 0 0.2];
            R = 0;
            Q_KALMAN = 1;
            R_KALMAN = [1 0; 0 1];
            mpcObj = cmmn.ModelPredictiveControl(MODEL,HP,HU,UMIN,UMAX,DUMIN,DUMAX,YMIN,YMAX,Q,R,Q_KALMAN,R_KALMAN);
        end

        function on_each_timestep(obj, vehicle_state_list)
            on_each_timestep@cmmn.InterfaceHlc(obj,vehicle_state_list);

            % pose to distance on path f/e veh
            y = obj.mt.measure_longitudinal(vehicle_state_list);
            disp(y)
            obj.counter = obj.counter + 1;

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
            obj.output(obj.counter,1) = y(1);
            obj.output(obj.counter,2) = y(2);
            obj.t(obj.counter) = obj.t_exp;
            % Apply control action
            % --------------------
            obj.cpmLab.apply_path_tracking(u, obj.path_points, obj.dt_valid_after_nanos)
        end

        function on_stop(obj)
            on_stop@cmmn.InterfaceHlc(obj);
            % TODO plot results, see plot_platooning.m
            data = iddata(obj.output, obj.input, obj.Ts);
            data.InputName  = {'VelocityIn'};
            data.OutputName = {'DistanceOut';'VelocityOut'};
            save('data.mat','data')
            order = (1:1:10); 
            sys = ssest(data, order);    
            save('sys.mat','sys')
            save('data.mat','data')
            figure(1)
            subplot(2,1,1)
            plot(data)
            subplot(2,1,2)
            compare(data, sys)
        end

    end
end
