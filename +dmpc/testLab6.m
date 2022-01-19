classdef testLab6
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Participant
    end
    
    methods
        function init_dds(obj)
            script_directoy = fileparts([mfilename('fullpath') '.m']);
            software_directory = fullfile(script_directoy,'../../..');

            % Import IDL files from cpm library
            dds_idl_matlab = fullfile(software_directory,'cpm_lib/dds_idl_matlab/');
            assert(isfolder(dds_idl_matlab),...
                'Missing directory "%s".', dds_idl_matlab);
            assert(~isempty(dir([dds_idl_matlab, '*.m'])),...
                'No MATLAB IDL-files found in %s', dds_idl_matlab);
            addpath(dds_idl_matlab)

            % XML files for quality of service settings
            middleware_local_qos_xml = fullfile(software_directory, 'middleware/build/QOS_LOCAL_COMMUNICATION.xml');
            assert(isfile(middleware_local_qos_xml),...
                'Missing middleware local QOS XML "%s"', middleware_local_qos_xml);
            
            ready_trigger_qos_xml = fullfile(software_directory,'high_level_controller/examples/matlab/QOS_READY_TRIGGER.xml');
            assert(isfile(ready_trigger_qos_xml),...
                'Missing ready trigger QOS XML "%s"', ready_trigger_qos_xml);
            
            setenv("NDDS_QOS_PROFILES", ['file://' ready_trigger_qos_xml ';file://' middleware_local_qos_xml]);
            
            %% variables for the communication
            matlabStateTopicName = 'vehicleStateList';
            matlabCommandTrajectoryTopicName = 'vehicleCommandTrajectory';
            matlabCommandPathTrackingTopicName = 'vehicleCommandPathTracking';
            matlabCommandDirectTopicName = 'vehicleCommandDirect';
            systemTriggerTopicName = 'systemTrigger';
            readyStatusTopicName = 'readyStatus';
            obj.trigger_stop = uint64(18446744073709551615);

            %% create participants
            matlab_domain_id = 1;
            obj.Participant = DDS.DomainParticipant('MatlabLibrary::LocalCommunicationProfile', matlab_domain_id);

            %% create reader and writer
            obj.reader_vehicleStateList = DDS.DataReader(DDS.Subscriber(obj.Participant), 'VehicleStateList', matlabStateTopicName);
            obj.writer_vehicleCommandTrajectory = DDS.DataWriter(DDS.Publisher(obj.Participant), 'VehicleCommandTrajectory', matlabCommandTrajectoryTopicName);
            obj.writer_vehicleCommandPathTracking = DDS.DataWriter(DDS.Publisher(obj.Participant), 'VehicleCommandPathTracking', matlabCommandPathTrackingTopicName);
            obj.writer_vehicleCommandDirect = DDS.DataWriter(DDS.Publisher(obj.Participant), 'VehicleCommandDirect', matlabCommandDirectTopicName);
            obj.reader_systemTrigger = DDS.DataReader(DDS.Subscriber(obj.Participant), 'SystemTrigger', systemTriggerTopicName, 'TriggerLibrary::ReadyTrigger');
            obj.writer_readyStatus = DDS.DataWriter(DDS.Publisher(obj.Participant), 'ReadyStatus', readyStatusTopicName, 'TriggerLibrary::ReadyTrigger');

            % Set reader properties
            obj.reader_vehicleStateList.WaitSet = true;
            obj.reader_vehicleStateList.WaitSetTimeout = 2; % [s]
        end
        
        function write_and_read
            
        end    
        
    end
end

