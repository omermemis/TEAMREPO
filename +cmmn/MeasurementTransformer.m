classdef MeasurementTransformer < handle
% MeasurementTransformer Provides the functionality to return the expected
%   measurements for a controller from a VehicleStateList sample
    
    properties (Access=private)
        s_on_loop
        s
        path_points
        vehicle_ids
    end

    methods
        function obj = MeasurementTransformer(path_points,vehicle_ids)
            obj.path_points = path_points;
            obj.vehicle_ids = vehicle_ids;
            nVeh = numel(vehicle_ids);
            obj.s_on_loop = zeros(1,nVeh);
            obj.s = zeros(1,nVeh);
        end

        function y = measure_longitudinal(obj,vehicle_state_list)
            nVeh = numel(obj.vehicle_ids);
            y = zeros(2*nVeh,1);
            cntr_veh = 1;
            for veh_id = obj.vehicle_ids
                iVeh = find([vehicle_state_list.state_list.vehicle_id]==veh_id);
                % TODO read position from vehicle_state_list
                % position = [0,0];
                position = [vehicle_state_list.state_list(iVeh).pose.x, vehicle_state_list.state_list(iVeh).pose.y];
                s_new = cmmn.compute_distance_on_path(position, obj.path_points);
                ds = cmmn.compute_rel_distance_on_path(obj.path_points,obj.s_on_loop(cntr_veh), s_new);
                obj.s_on_loop(cntr_veh) = s_new;
                obj.s(cntr_veh) = obj.s(cntr_veh) + ds;
                iPos = (cntr_veh-1)*2+1;
                iSpd = (cntr_veh-1)*2+2;
                y(iPos,1) = obj.s(cntr_veh);
                y(iSpd,1) = vehicle_state_list.state_list(iVeh).speed;
                cntr_veh = cntr_veh + 1;
            end
        end

        function y = measure_central(obj,vehicle_state_list)
            % TODO get measurements for central model
            nVeh = numel(obj.vehicle_ids);
            y_ = zeros(2*nVeh,1);
            y = zeros(nVeh,1);
            cntr_veh = 1;
            for veh_id = obj.vehicle_ids
                iVeh = find([vehicle_state_list.state_list.vehicle_id]==veh_id);
                position = [vehicle_state_list.state_list(iVeh).pose.x, vehicle_state_list.state_list(iVeh).pose.y];
                s_new = cmmn.compute_distance_on_path(position, obj.path_points);
                ds = cmmn.compute_rel_distance_on_path(obj.path_points,obj.s_on_loop(cntr_veh), s_new);
                obj.s_on_loop(cntr_veh) = s_new;
                obj.s(cntr_veh) = obj.s(cntr_veh) + ds;
                iPos = (cntr_veh-1)*2+1;
                iSpd = (cntr_veh-1)*2+2;
                y_(iPos,1) = obj.s(cntr_veh);
                y_(iSpd,1) = vehicle_state_list.state_list(iVeh).speed;
                cntr_veh = cntr_veh + 1;
            end
            % y = [d21; d32...; dn(n-1); vn], where dn(n-1)=distance(n)-distance(n-1)
            for i=1:nVeh
                if i==nVeh
                    y(i,1) = y_(2*i);
                else
                    y(i,1) = y_(2*i+1)-y_(2*i-1);
                end
            end
                
        end
        
    end
end