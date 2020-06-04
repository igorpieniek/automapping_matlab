function  map_out = LidarAq(map, subscriberObj, scanAngleOffset)
% W funkcji tej nast�puje odczyt skanu z Lidaru oraz przypisanie go do mapy
% INPUT:
%  - map - objekt lidarSLAM
%  - subscriberObj - objekt Subscriber 
% OUTPUT:
%  - map_output - obiekt lidarSLAM, uzupe�niona mapa o dodatkowy skan

if ~isa(map, 'lidarSLAM')
    error("map - objekt lidarSLAM");
end

disp("Lidar acquisition START!")

while true
    % Surowy odczyt 
    scandata_raw = receive(subscriberObj,10); 
    
    % K�ty - zwi�zane z rozdzielczo�ci� lidaru
    angles =((scandata_raw.AngleMin +  scanAngleOffset):(scandata_raw.AngleIncrement):(scandata_raw.AngleMax + angleOffset))';
    
    ranges = scandata_raw.Ranges;
    
    % Po��czernie k�t�w oraz pojednyczych pomiar�w    
    scan = lidarScan(double(ranges), double(angles));
    
    %Dodanie skanu do mapy
    [isScanAccepted, ~, ~] = addScan(map, scan);
    if ~isScanAccepted
        continue;
    else
        break;
    end
end
disp("Lidar acquisition DONE!")
map_out = map;
