%CZYSTY POŒCIG

% Utworzenie œcie¿ki
path = [2.00    1.00; 
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        11.75   10.75;
        12.00   10.00];
    
%ustawienie startowej i koncowej pozycji    
start_Location = [path(1,:), pi/2]; % [x y angle]
stop_Location = path(end,:);

% minimalna odleg³oœæ od punktu koñcowego
goalRadius = 0.1; % [m]
% robotKinematic = differentialDriveKinematics("TrackWidth", 0.2, "VehicleInputs", "VehicleSpeedHeadingRate");
% robotKinematic.WheelSpeedRange = [-10 10]*2*pi; % zakres prêdkosci

% controller init
currentPosition = start_Location;
controller = controllerPurePursuit("Waypoints",path,"DesiredLinearVelocity",0.5,"MaxAngularVelocity", pi/4, "LookaheadDistance", 0.3);




[v, omega] = controller(start_Locatrion);
%     
%     V_LeftWheel = v - (omega * robotKinematic.TrackWidth/2);
%     V_RightWheel= v +(omega * robotKinematic.TrackWidth/2);      