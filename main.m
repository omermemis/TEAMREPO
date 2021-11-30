function main(varargin)
    % MAIN  main function

    vehicle_ids = cell2mat(varargin);
    Ts = 0.4; %[s]
<<<<<<< HEAD
    hlc = pmpc.Hlc(Ts,vehicle_ids);
=======
    hlc = pmpc.HlcIdentification(Ts,vehicle_ids);
%     hlc = pmpc.Hlc(Ts,vehicle_ids);
>>>>>>> 28efdb3b121e0b1ce94a4ba4f7e6f255f9435d45
%     hlc = cmpc.Hlc(Ts,vehicle_ids);
%     hlc = dmpc.Hlc(Ts,vehicle_ids);

    hlc.start();
end