function  map_out = LidarAq(map, subscriberObj, scanAngleOffset)
% W funkcji tej nastêpuje odczyt skanu z Lidaru oraz przypisanie go do mapy
% INPUT:
%  - map - objekt lidarSLAM
%  - subscriberObj - objekt Subscriber 
% OUTPUT:
%  - map_output - obiekt lidarSLAM, uzupe³niona mapa o dodatkowy skan

if ~isa(map, 'lidarSLAM')
    error("map - objekt lidarSLAM");
end

disp("Lidar acquisition START!")

while true
    % Surowy odczyt 
    scandata_raw = receive(subscriberObj,10); 
    
    % K¹ty - zwi¹zane z rozdzielczoœci¹ lidaru
    angles =((scandata_raw.AngleMin +  scanAngleOffset):(scandata_raw.AngleIncrement):(scandata_raw.AngleMax + angleOffset))';
    
    ranges = scandata_raw.Ranges;
    
    % Po³¹czernie k¹tów oraz pojednyczych pomiarów    
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
