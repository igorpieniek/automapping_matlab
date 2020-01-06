function  map_out = LidarAq(map, subscriberObj)
% %OPIS FUNKCJI
% W funkcji tej nastêpuje pomiar z lidaru i wpisanie go do aktualnej mapy
% 
% INPUT:
%   -map - objekt lidarSLAM
%   -subscriberObj - objekt Subscriber 
% OUTPUT:
%   -map_output - obiekt lidarSLAM, uzupe³niona mapa o dodatkowy pomiar

if ~isa(map, 'lidarSLAM')
    error("map - objekt lidarSLAM");
elseif ~isa(subscriberObj, 'Subscriber')
    error("subscriberObj - objekt Subscriber");
end

while true
    scandata_raw = receive(subscriberObj,10);
    
    angles =((scandata_raw.AngleMin):(scandata_raw.AngleIncrement):(scandata_raw.AngleMax))';

    ranges = scandata_raw.Ranges;

    scan = lidarScan(double(ranges), double(angles));
    [isScanAccepted, ~, ~] = addScan(map, scan);
    if ~isScanAccepted
        continue;
    else
        break;
    end
end
map_out = map;
