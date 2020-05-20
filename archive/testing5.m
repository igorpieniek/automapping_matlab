% proba zrobienia symulacji

clc
close all
clear all


% zrobienie "ukrytej" mapy
grayimage = rgb2gray(imread('mapa_paint4.png'));
bwimage = grayimage < 0.5;
resolution = 20;
hide_map = occupancyMap(bwimage, resolution);

% inicjacja nowej mapy
explo_map = occupancyMap( (ones(size(grayimage))).*0.5, resolution);


startPoint = [0.5 0.5 pi/2]; % punkt startowy
rangefinder = rangeSensor('HorizontalAngle', [-pi pi],'RangeNoise', 0.001);
numReadings = rangefinder.NumReadings;

last_pose_num  = 1; % nr ostatniej pozycji przy której osiagnieto punkt eksploracyjny
explo_points=[];    % tablica na punkty eksploracyjne
all_poses = [];     % tablica wszystkich kolejno osi¹ganych pozycji
all_poses(end+1,:) = startPoint; % dodanie pierwszego punktu

% pierwszy pomiar w punkcie startowym
[ranges, angles] = rangefinder(startPoint, hide_map);
insertRay(explo_map, startPoint, ranges, angles,  rangefinder.Range(end));


%% TEST
NumOfPoints = 1; % init ilosci punktów eksploracyjnych
ChangeNumOfPointsUP = false;
ChangeNumOfPointsDOWN = false;
fuckgoback_points = []; % miejsce na przechowywanie punktów eksploracyjnych od tego podwójego
%%
figure
% pêtla wyszukiwania kolejnych punktów i przemieszczania sie do nich
while true
    
    if  ChangeNumOfPointsUP
        fuckgoback_points(end+1,:) = startPoint(:,1:2);
        explo_points = exploratory_points2(explo_map, explo_points, last_pose_num, all_poses );
    elseif ChangeNumOfPointsDOWN
        if isempty(fuckgoback_points)
            ChangeNumOfPointsDOWN = false;
            continue;
        else
            target_point = fuckgoback_points(end,:);
        end

    else
        explo_points = exploratory_points2(explo_map, explo_points, last_pose_num, all_poses );
    end

    
    if ~isempty(explo_points)
        disp(['Explo_points DONE! NUMBER OF POINTS: ', num2str(length(explo_points(:,1)))]);
        if ~(length(explo_points(:,1)) == NumOfPoints)
            if length(explo_points(:,1)) > NumOfPoints % pojawi³¹ sie nowa ga³¹Ÿ
                ChangeNumOfPointsUP = true;
                ChangeNumOfPointsDOWN = false;
            elseif length(explo_points(:,1)) < NumOfPoints % punkty w jednej ga³êzi siê skonczy³y
                ChangeNumOfPointsDOWN = true;
                ChangeNumOfPointsUP = false;
                NumOfPoints = length(explo_points(:,1));
                continue;
            end
            NumOfPoints = length(explo_points(:,1));
        end
        
        if ~ChangeNumOfPointsDOWN
            % Wyznaczenie najlepszego punktu z punktów eksploracyjnych
            target_num = find(explo_points(:,3) == max(explo_points(:,3)));
            target_point =  explo_points(target_num,1:2) ;
        end
        
        hold on
        plot(target_point(1,1), target_point(1,2),'or')
        
        % Wyznaczenie wstêpnej trasy do zadanego punktu
        disp("Planner START!");
        vMap = validatorOccupancyMap;
        temp_map = occupancyMap(imbinarize(occupancyMatrix(explo_map),0.51), resolution);
        inflate(temp_map, 0.01);
        vMap.Map = temp_map;
        planner = plannerHybridAStar(vMap, 'MinTurningRadius', 0.2, 'MotionPrimitiveLength',0.2); % stworzenie obiektu planner
        
        % Przypisanie punktu pocz¹tkowego i koncowego wraz z wyznaczeniem k¹ta    
        entrance = startPoint;
        packagePickupLocation = [target_point Angle2Points(startPoint(1,1:2), target_point(1,1:2) )];
        
        poses = plannerProcess(planner, entrance, packagePickupLocation);
       
        disp("Planner DONE!");
     

        % Pocz¹tek przemieszczenia pojazdu do zadanego punktu
        disp("Navigation to point...");
        idx =1;
        tic
        while idx <= size(poses,1)
            ranges = [];
            angles = [];
            % Kolejne pomiary dla kolejnych pozycji i do³¹czanie ich do mapy
            [ranges, angles] = rangefinder(poses(idx,:), hide_map);
            insertRay(explo_map, poses(idx,:), ranges, angles, rangefinder.Range(end));


            all_poses(end+1,:) = poses(idx,:); % odczytanie ostatniej pozycji i dopsanie jej do tablicy wszystkich pozycji
            
            % Aktualizacja tworzonej mapy i trasy
            hold on
            show(explo_map, 'FastUpdate', true);
            
            if idx>2
                hold on
                plot(poses(idx-1 : idx ,1), poses(idx-1 :idx,2), '-r');
            end
            drawnow
            
            % Sprawdzenie czy na wyznaczonej trasie nie pojawi³a siê przeszkoda
            isRouteOccupied = any(checkOccupancy(explo_map, poses(:,1:2)));
            if isRouteOccupied && (toc > 0.5)
                % W przypadku pojawienia sie przeszkody - wyznaczenie nowej trasy
                temp_map = occupancyMap(imbinarize(occupancyMatrix(explo_map),0.51), resolution);
                inflate(temp_map, 0.01);
                planner.StateValidator.Map = temp_map;
                
                poses = plannerProcess(planner, poses(idx,:), packagePickupLocation);
                
                idx = 1;
                tic;
            else
                idx = idx + 1;  
            end
        end
        disp("Navigation to point... DONE!");
        
        if ~ChangeNumOfPointsDOWN
            explo_points(target_num,:) = []; % usuniêcie osi¹gnietego punktu eksploracyjnego
        else
            fuckgoback_points(end, :) = [];
        end
        last_pose_num  = length(all_poses(:,1));
        startPoint =  poses(end,:);
    else
        disp("MAPPING DONE");
        figure
        subplot(1,2,1)
        show(hide_map);
        title("Rzeczywista mapa")
        
        subplot(1,2,2)
        show(explo_map);
        title("Ukonczona  mapa po symulacji")
        break;
    end   
end