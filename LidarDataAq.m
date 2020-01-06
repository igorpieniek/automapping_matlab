close
clear
close all
clear all
clc

laser = rossubscriber('/scan');


maxLidarRange = 6;
mapResolution = 10; %70
slamAlg = lidarSLAM(mapResolution, maxLidarRange,50);
slamAlg.LoopClosureThreshold = 360;  
slamAlg.LoopClosureSearchRadius = 6;
i = 0;
ranges=[];
angles=[];




while i <1

    scandata_raw = receive(laser,10);
    
    if (i == 0)
        angles_raw =((scandata_raw.AngleMin):(scandata_raw.AngleIncrement):(scandata_raw.AngleMax))';
        disp('Receive first lidar data')
    end
    angles = angles_raw;

    disp (['Data num: ', num2str(i)]);
    scan = lidarScan(double(scandata_raw.Ranges), double(angles));
    [isScanAccepted, ~, ~] = addScan(slamAlg, scan);
    if ~isScanAccepted
        continue;
    end
    if mod(i,5) == 0
        disp(' ');
    end
    i = i+1;
end
show(slamAlg);
grid off
%%
% maxLidarRange = 6;
% mapResolution = 70;
% show(slamAlg);
% [scansSLAM,poses] = scansAndPoses(slamAlg);
% occMap = buildMap(scansSLAM,poses,mapResolution,maxLidarRange);
% figure
% show(occMap)





