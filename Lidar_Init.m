function map = Lidar_Init

maxLidarRange = 8;
mapResolution = 70;
MAX_NumOfScans = 100;

map = lidarSLAM(mapResolution, maxLidarRange,MAX_NumOfScans);
map.LoopClosureThreshold = 360;  
map.LoopClosureSearchRadius = 8;