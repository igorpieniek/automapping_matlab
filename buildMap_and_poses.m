function [occMap, poses] = buildMap_and_poses(map_lidar, mapResolution, maxLidarRange)

if ~isa(map_lidar, 'lidarSLAM')
    error("map - objekt lidarSLAM");
end

[scansSLAM,poses] = scansAndPoses(map_lidar);
occMap = buildMap(scansSLAM,poses,mapResolution,maxLidarRange);


% for i = 1:40
%     msg = rosmessage('std_msgs/Int32MultiArray');
%     msg.Data = [i 10*i 100*i];
%     send(pub.msg)
%     pause(1);
% end