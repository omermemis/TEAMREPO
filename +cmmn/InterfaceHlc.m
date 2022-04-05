classdef InterfaceHlc < handle
% InterfaceHlc Interface for a High-Level Controller
    properties
        cpmLab                  % Instance of CpmLab Class
        is_first_timestep       % boolean
        dt_period_nanos         % Controller / MW period
        t_now_nanos             % unix timestamp from received MW sample
        t_start_nanos           % unix timestamp from first MW sample
        t_exp                   % time elapsed since start
        dt_valid_after_nanos    % duration after which messages become valid
        computation_time
    end
    
    methods
        function obj = InterfaceHlc(vehicle_ids)
            obj.is_first_timestep = true;
            obj.cpmLab = cmmn.CpmLab(vehicle_ids);

        end

        function start(obj)
            obj.cpmLab.send_ready_message(); 
         
            while(~obj.cpmLab.is_stop())
                try
                    vehicle_state_list = obj.cpmLab.measure();

                    if obj.is_first_timestep
                        obj.on_first_timestep(vehicle_state_list);
                        obj.is_first_timestep = false;
                    end

                    t_hlc = tic;
                    obj.on_each_timestep(vehicle_state_list);
                    t_elapsed = toc(t_hlc);
                    fprintf("T_hlc: %5.0f ms | t_exp = %5.1f s\n",t_elapsed*1e3,obj.t_exp);
                    obj.computation_time = [obj.computation_time t_elapsed*1e3];
                catch ME
                    switch ME.identifier
                        case 'CpmLab:measure'
                            warning(ME.message);
                        otherwise
                            rethrow(ME);
                    end
                    % continue
                end
            end
         
            obj.on_stop();
        end

        function on_first_timestep(obj, vehicle_state_list)
            disp('first timestep')
            obj.t_start_nanos = vehicle_state_list.t_now;
            % Middleware period for valid_after stamp
            obj.dt_period_nanos = uint64(vehicle_state_list.period_ms*1e6);
            dt_max_computation_communication = uint64(100e6); % multiples of dt_period
            obj.dt_valid_after_nanos = max(obj.dt_period_nanos,dt_max_computation_communication);
        end

        function on_each_timestep(obj, vehicle_state_list)
            obj.t_now_nanos = uint64(vehicle_state_list.t_now);
            % Middleware period for valid_after stamp
            obj.dt_period_nanos = uint64(vehicle_state_list.period_ms*1e6);
            obj.t_exp = double(vehicle_state_list.t_now - obj.t_start_nanos)*1e-9;
        end

        function on_stop(obj)
            disp('stop')
            fprintf("average computaion time each step: %5.0f ms\n",mean(obj.computation_time));
            fprintf("minimal computaion time each step: %5.0f ms\n",min(obj.computation_time));
            fprintf("maximal computaion time each step: %5.0f ms\n",max(obj.computation_time));
            % save to .txt
            mkdir(obj.path_save_results); % make new folder to store figures created by plot_platooning, in order to make it easier to compare the different results when tuning HP and HU
            fileID = fopen([obj.path_save_results '/computationTime.txt'],'wt');
            fprintf(fileID, "average computaion time each step: %5.0f ms\n",mean(obj.computation_time));
            fprintf(fileID, "minimal computaion time each step: %5.0f ms\n",min(obj.computation_time));
            fprintf(fileID, "maximal computaion time each step: %5.0f ms\n",max(obj.computation_time));
            fclose(fileID);
        end
    end
end

