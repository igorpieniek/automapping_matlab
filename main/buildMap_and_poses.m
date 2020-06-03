function [occMap, poses] = buildMap_and_poses(lidarScan, mapResolution, maxLidarRange)
% Funkcja dodająca skan do mapy
% INPUT
% - lidarScan - skan z lidaru, jako obiekt klasy lidarSLAM
% - mapResolution - rozdzielczość budowanej mapy zajętości
% - maxLidarRange - zakres pomiarowy lidaru
% OUTPUT
% - occMap - occupancyMap
% - poses - pozycje estymowane według algorytmu SLAM

if ~isa(lidarScan, 'lidarSLAM')
    error("map - objekt lidarSLAM");
end

[scansSLAM,poses] = scansAndPoses(lidarScan);
occMap = buildMap(scansSLAM,poses,mapResolution,maxLidarRange);

