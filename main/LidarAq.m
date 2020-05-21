function  map_out = LidarAq(map, subscriberObj)
% %OPIS FUNKCJI
% W funkcji tej nast�puje pomiar z lidaru i wpisanie go do aktualnej mapy
% 
% INPUT:
%   -map - objekt lidarSLAM
%   -subscriberObj - objekt Subscriber 
% OUTPUT:
%   -map_output - obiekt lidarSLAM, uzupe�niona mapa o dodatkowy pomiar

if ~isa(map, 'lidarSLAM')
    error("map - objekt lidarSLAM");
% elseif ~isa(subscriberObj, 'subscriber')
%     error("subscriberObj - objekt Subscriber");
end

angleOffset = -pi/2;
disp("Lidar acquisition START!")

while true
    scandata_raw = receive(subscriberObj,10);
    
    angles =((scandata_raw.AngleMin + angleOffset):(scandata_raw.AngleIncrement):(scandata_raw.AngleMax + angleOffset))';

    ranges = scandata_raw.Ranges;

    scan = lidarScan(double(ranges), double(angles));
    [isScanAccepted, ~, ~] = addScan(map, scan);
    if ~isScanAccepted
        continue;
    else
        break;
    end
end
disp("Lidar acquisition DONE!")
map_out = map;