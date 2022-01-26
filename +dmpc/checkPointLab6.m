classdef checkPointLab6 < cmmn.InterfaceHlc
% HlcIdentification implements an HLC for model identification
    properties
%         mt
        Ts
%         path_points
%         input
%         s_ref
%         output
%         counter
%         t
%         HP
%         HU
%         mpcObj
%         s0
%         DUMIN
%         DUMAX
%         v_max
%         v_min
%         slack_var
%         delta_u
%         s_max
%         val_objective_fcn % objective function
%         nVeh % number of vehicles
%         ny % output length
%         nu % input length
%         nx % number of states
%         d_ref % relative distance reference
%         v_ref % reference speed
%         a_min % min accelelation constraint
%         a_max % max accelelation constraint
%         d_min % min relative distance constraint
%         d_max % max relative distance constraint
        vehicle_ids
        reader_vehicleOutput_without_filter
        reader_vehicleOutput_without_filter2
        reader_vehicleOutput_with_filter
        reader_vehicleOutput_with_filter2
        writer_vehicleOutput
    end

    methods
        function obj = checkPointLab6(Ts,vehicle_ids)
            obj = obj@cmmn.InterfaceHlc(vehicle_ids);
            obj.vehicle_ids = vehicle_ids;
            matlabOutputTopicName = 'vehicleOutput';
            obj.writer_vehicleOutput = DDS.DataWriter(DDS.Publisher(obj.cpmLab.matlabParticipant), 'dmpc.HlcPlan', matlabOutputTopicName);
            obj.reader_vehicleOutput_without_filter = DDS.DataReader(DDS.Subscriber(obj.cpmLab.matlabParticipant), 'dmpc.HlcPlan', matlabOutputTopicName); % without filter
            obj.reader_vehicleOutput_without_filter2 = DDS.DataReader(DDS.Subscriber(obj.cpmLab.matlabParticipant), 'dmpc.HlcPlan', matlabOutputTopicName); % without filter

            Filter = DDS.contentFilter; % create Filter instance
            Filter.FilterExpression = 'vehicle_id = %0'; % only the specific vehicleID will be filtered
            myID = 5;
            Filter.FilterParameters = {num2str(myID)};
            obj.reader_vehicleOutput_with_filter = DDS.DataReader(DDS.Subscriber(obj.cpmLab.matlabParticipant), 'dmpc.HlcPlan', matlabOutputTopicName,'',Filter); % with filter
            obj.reader_vehicleOutput_with_filter2 = DDS.DataReader(DDS.Subscriber(obj.cpmLab.matlabParticipant), 'dmpc.HlcPlan', matlabOutputTopicName,'',Filter); % with filter

            vehicleOutput = dmpc.HlcPlan;
            for i=[4,5,6,7] % write 4 messages to domain
                vehicleOutput.vehicle_id = uint8(i);
                vehicleOutput.output = repmat([1;i],10,1);
                obj.writer_vehicleOutput.write(vehicleOutput);
            end

            [topicData1, ~,  sampleCount1, ~] = obj.reader_vehicleOutput_with_filter.take();
            disp('display topicData1:')
            disp(topicData1)
            [topicData2, ~,  sampleCount2, ~] = obj.reader_vehicleOutput_with_filter.take(); % the second time you use the same reader, you'll read nothing
            disp('display topicData2:')
            disp(topicData2)
            [topicData3, ~,  sampleCount3, ~] = obj.reader_vehicleOutput_with_filter2.take(); % but if you use another reader, you'll read something, why?
            disp('display topicData3:')
            disp(topicData3)
            [topicData4, ~,  sampleCount4, ~] = obj.reader_vehicleOutput_without_filter.take(); 
            disp('display topicData4:')
            disp(topicData4)
            [topicData5, ~,  sampleCount5, ~] = obj.reader_vehicleOutput_without_filter2.take(); % all the messages should be taken by 'reader_vehicleOutput_without_filter', but here you can still take messages from domain, why?
            disp('display topicData5:')
            disp(topicData5)

        end

        function on_stop(obj)
            on_stop@cmmn.InterfaceHlc(obj); 
        end

    end
end
