function main(varargin)
    % MAIN  main function
        
    vehicle_ids = cell2mat(varargin);
    Ts = 0.1; % [s] sample time
%     hlc = pmpc.HlcIdentification(Ts,vehicle_ids);
%     hlc = pmpc.Hlc(Ts,vehicle_ids);
%     hlc = cmpc.HlcIdentification(Ts,vehicle_ids);
    hlc = dmpc.Hlc(Ts,vehicle_ids);   
%     hlc = dmpc.checkPointLab6(Ts,vehicle_ids);  

    hlc.start();
end 