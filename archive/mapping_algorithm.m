maxLidarRange = 6;
mapResolution = 70;
margin = 0.2;

% show(slamAlg);
[scansSLAM,poses] = scansAndPoses(slamAlg);
occMap = buildMap(scansSLAM,poses,mapResolution,maxLidarRange);
% figure
% show(occMap)


reserch_angles = 0:(pi()/4):(2*pi);
% RESERCH LEVEL 0
for pos_num = 1%:length(poses)
    disp("AA")
    intersectionPoints = rayIntersection(occMap,poses(pos_num,:),reserch_angles,maxLidarRange);
    NO_intersection_check = isnan(intersectionPoints(:,1));
    if ~isempty(NO_intersection_check)
        disp("not empty")
        k =1;
        NO_intersection_angle = [];
        for NO_intercection_num = 1: length(NO_intersection_check)
            if NO_intersection_check(NO_intercection_num,1) == 1
                NO_intersection_angle(k,:) = reserch_angles(1,NO_intercection_num);
                k=k+1;
                % get one border
                problem_angle1 = reserch_angles(1,NO_intercection_num);
                while isnan(rayIntersection(occMap,poses(pos_num,:),problem_angle1,maxLidarRange))
                    problem_angle1= problem_angle1+(pi()/36);
                end
                border1 = rayIntersection(occMap,poses(pos_num,:),problem_angle1,maxLidarRange);
                
                % get second border
                problem_angle2 = reserch_angles(1,NO_intercection_num);
                while isnan(rayIntersection(occMap,poses(pos_num,:),problem_angle2,maxLidarRange))
                    problem_angle2= problem_angle2-(pi()/36);
                end
                border2 = rayIntersection(occMap,poses(pos_num,:),problem_angle2,maxLidarRange);
                border_Length = sqrt((border1(1,1) -border2(1,1))^2 + (border1(1,2)-border2(1,2) )^2);
                figure
                show(slamAlg);
                hold on
                plot(border1(:,1),border1(:,2),'*r') % Intersection points
                hold on
                plot(border2(:,1),border2(:,2),'*b') % Intersection points
            end
        end
    end
    
end

p = poses(length(poses),:);
intersectionPoints = rayIntersection(occMap,LastPosition,reserch_angles,maxLidarRange);
reserch_points =[];
for i = 1: length(intersectionPoints)
    reserch_points(i,1) =  0.5*(intersectionPoints(i,1) + LastPosition(1));
    reserch_points(i,2) =  0.5*(intersectionPoints(i,2) + LastPosition(2));
    
end

figure
show(slamAlg);
hold on
plot(reserch_points(:,1),reserch_points(:,2),'*r') % Intersection points



zeropoints = isnan(intersectionPoints);


RobotLength = 0.2 ;%[m]
RobotWidth = 0.2; %[m]
zeroangles = [];
k =1;
for i = 1:length(zeropoints)
    if zeropoints(i,1) == 1
        zeroangles(k,1) = llastpos_scan.angles(i,1);
        k=k+1;
    end
end

