function test()
    vehicle_ids = 1;
    hlc = cmmn.testLab6(vehicle_ids);
%     write_and_read()
end

% %% create participants
% matlab_domain_id = 1;
% obj.Participant = DDS.DomainParticipant
% vehicleOutputTopicName='vehicleOutput';
% obj.writer_vehicleOutput = DDS.DataWriter(DDS.Publisher(obj.Participant), 'vehicleOutput', 'vehicleOutput');
% obj.read_vehicleOutput = DDS.DataReader(DDS.Subscriber(obj.Participant), 'vehicleOutput', 'vehicleOutput');
% output = dmpc.HlcPlan;
% output.vehicle_id = 1;
% output.output = [1;2];
% obj.writer_vehicleOutput.write();
% output = obj.read_vehicleOutput.take()