close
clear
close all
clear all
clc

laser = rossubscriber('/scan');


maxLidarRange = 6;
mapResolution = 40; %70
map = lidarSLAM(mapResolution, maxLidarRange,500);
map.LoopClosureThreshold = 360;  
map.LoopClosureSearchRadius = 6;

last_pose_num = 1; % init - nr pozycji dla ktorej wyznaczony punkt zostal osiagniety
explo_points = []; % tablica na aktualne punkty ekspolracyjne

MIN_TARGET_DIST = 0.15; % minimalna odleglosc pozycji od punktu docelowego aby punkt zosta³ zaliczony
figure
show(map);
hold on

for i =1:5
 map = LidarAq(map, laser);
end

 show(map);
 disp('Wstêpne pomiary ukonczone')
 target_point_counter = 1;
 
while true 
    map = LidarAq(map, laser);
    explo_points = exploratory_points2(map,explo_points, last_pose_num );
    
    target_num = find(explo_points(:,3) == max(explo_points(:,3)));
    target_point =  explo_points(target_num,1:2) ;
    
    
    if isempty(target_point)
        disp('Mapowanie zakonczone');
     
        break;
    else
        while true
            [~,poses] = scansAndPoses(map);
            dist = norm(poses(end, 1:2) - target_point(end, 1:2) );
            if dist <= MIN_TARGET_DIST
                last_pose_num = length(poses(:, 1));
                explo_points(target_num, : ) = []; % usuniecie tego punktu z listy
                disp(["Punkt",num2str(target_point_counter) ,"osiagniety!"]);
                target_point_counter = target_point_counter +1;
                set(target_plot,'Visible','off');
                break;
            else
                show(map);
                hold on
                target_plot = plot(target_point(:,1),target_point(:,2),'ok');
                map = LidarAq(map, laser);
                
            end
        end  
    end 
end

   figure
   show(map);
   title('Ukonczona mapa');
