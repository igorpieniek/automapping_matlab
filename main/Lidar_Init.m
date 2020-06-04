function map = Lidar_Init(maxLidarRange, mapResolution )
% Funkcja inicjująca obiekt klasy lidarSLAM
% INPUT:
% - maxLidarRange - maksymalny zakres pomiarowy lidaru
% - mapResolution - rozdzielczość mapy
% OUTPUT:
% - map - obiekt lidarSLAM

MAX_NumOfScans = 100;

map = lidarSLAM(mapResolution, maxLidarRange,MAX_NumOfScans);
map.LoopClosureThreshold = 360;  
map.LoopClosureSearchRadius = 8;