% proba zrobienia symulacji

clc
close all
clear all


% zrobienie "ukrytej" mapy
grayimage = rgb2gray(imread('mapa_paint5.png'));
bwimage = grayimage < 0.6;
resolution = 20;
hide_map = occupancyMap(bwimage, resolution);

% inicjacja nowej mapy
explo_map = occupancyMap( (ones(size(grayimage))).*0.5, resolution);


startPoint = [0.5 0.5 pi/2]; % punkt startowy
rangefinder = rangeSensor('HorizontalAngle', [-pi pi],'RangeNoise', 0.001,'Range' , [0 8]);
numReadings = rangefinder.NumReadings;

last_pose_num  = 1; % nr ostatniej pozycji przy której osiagnieto punkt eksploracyjny
explo_points=[];    % tablica na punkty eksploracyjne
all_poses = [];     % tablica wszystkich kolejno osi¹ganych pozycji
all_poses(end+1,:) = startPoint; % dodanie pierwszego punktu

% pierwszy pomiar w punkcie startowym
[ranges, angles] = rangefinder(startPoint, hide_map);
insertRay(explo_map, startPoint, ranges, angles,  rangefinder.Range(end));


%% TEST
parentNum = 0;
goback_flag= false;
child = [];
parentTochild_route = [];

 

badtarget_flag = false;

parentTochild_route = [0 startPoint(1,1:2)];

%%
figure
% pêtla wyszukiwania kolejnych punktów i przemieszczania sie do nich
while true
  %%  Czêœæ algortytmu odpowiadaj¹ca za relacje rodzic-dziecko 
    if goback_flag
        if parentTochild_route(end,1) == parentNum
            target_point = parentTochild_route(end-1,2:3);
            parentTochild_route(end, :) = [];
        else
            temp = find(child(:,1) == parentNum);
            if ~isempty(temp)
                sameparent_points = child(temp,:);
                % sprawdzenie obszaru - na wszelki wypadek zeby robot nie
                % jecha³ w miejsce juz poznane
                for k = 1: length(all_poses(:,1))
                    if ~isempty(sameparent_points)
                        delate_child_num = [];
                        for ex_num = 1: length(sameparent_points(:,1))
                            ex_dist = norm(all_poses(k,1:2)-  sameparent_points(ex_num,2:3));
                            if ex_dist < 0.75
                                delate_child_num(end+1) = ex_num;
                            end
                        end
                        sameparent_points(delate_child_num, :) = [];
                        temp(delate_child_num, :) = [];
                    end
                end
                 if isempty(sameparent_points)
                     parentNum = parentNum -1;
                     continue;
                 end
                
                points_withrating = [exploratory_points_rating(sameparent_points(:,2:3), explo_map, startPoint) temp];
                
                target_num = find(points_withrating(:,3) == max(points_withrating(:,3)));
                target_point =  points_withrating(target_num,1:2) ;
                child(points_withrating(target_num, 4), :) = [];
                
                goback_flag = false;
            else
                parentNum = parentNum -1;
                continue;
            end
        end
        
    else
        if parentNum == 0
            parentTochild_route(end+1,:) =[ 0 startPoint(1,1:2)];
        else
            parentTochild_route(end+1,:) =[ parentNum  target_point(1,1:2)];
        end
        
        explo_points = [];
        explo_points = exploratory_points2(explo_map, explo_points, last_pose_num, all_poses );
        
        if isempty(explo_points)
            if isempty(child)
                break;
            else
                goback_flag = true;
                continue;
            end
        else
            if length(explo_points(:,1)) > 1
                parentNum = parentNum +1;
                target_num = find(explo_points(:,3) == max(explo_points(:,3)));
                target_point =  explo_points(target_num,1:2) ;
                
                explo_points(target_num, :) = [];
                child = [child ; repmat(parentNum, length(explo_points(:,1)), 1) explo_points(:,1:2)];
                hold on
                plot(child(:,2), child(:,3),'om');
                
            else
                target_point =  explo_points(:,1:2) ;
            end
        end
    end
  %%  
    
    hold on
    plot(target_point(1,1), target_point(1,2),'or')
    
    % Wyznaczenie wstêpnej trasy do zadanego punktu
    disp("Planner START!");
    vMap = validatorOccupancyMap;
    temp_map = occupancyMap(imbinarize(occupancyMatrix(explo_map),0.51), resolution);
    inflate(temp_map, 0.01);
    vMap.Map = temp_map;
    planner = plannerHybridAStar(vMap, 'MinTurningRadius', 0.35, 'MotionPrimitiveLength',0.35); % stworzenie obiektu planner

    
    % Przypisanie punktu pocz¹tkowego i koncowego wraz z wyznaczeniem k¹ta
    entrance = startPoint;
    packagePickupLocation = [target_point Angle2Points(startPoint(1,1:2), target_point(1,1:2) )];
    
    last_pose_num  = length(all_poses(:,1));
    
    try
        poses = plannerProcess(planner, entrance, packagePickupLocation);
    catch er
        switch er.identifier
            case 'nav:navalgs:astar:OccupiedLocation'
                warning('Droga nie moze zostac wyznaczona! Proces zostanie przerwany');
                continue;
            case 'nav:navalgs:hybridastar:StartError'
                warning('Droga nie moze zostac wyznaczona! (planer A* error) Proces zostanie przerwany');
                continue;
            case 'nav:navalgs:hybridastar:GoalError'
                warning('Droga nie moze zostac wyznaczona! (planer A* error) Proces zostanie przerwany');
                 continue;
            otherwise
                rethrow(er);
        end
    end
    
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
            if goback_flag
                hold on
                plot(poses(idx-1 : idx ,1), poses(idx-1 :idx,2), '-b');
            else
                hold on
                plot(poses(idx-1 : idx ,1), poses(idx-1 :idx,2), '-r');
            end
        end
        drawnow
        temp_map = occupancyMap(imbinarize(occupancyMatrix(explo_map),0.51), resolution);
        inflate(temp_map, 0.01);
        

        % Sprawdzenie czy na wyznaczonej trasie nie pojawi³a siê przeszkoda
        isRouteOccupied = any(checkOccupancy(explo_map, poses(:,1:2)));
        if isRouteOccupied && (toc > 0.5)
            % W przypadku pojawienia sie przeszkody - wyznaczenie nowej trasy

            planner.StateValidator.Map = temp_map;
            % obs³uga b³êdów zwi¹zanych z replanowaniem trasy
            try
                poses = plannerProcess(planner, poses(idx,:), packagePickupLocation);
            catch er
                if er.identifier == 'nav:navalgs:astar:OccupiedLocation'
                    warning('Droga nie moze zostac wyznaczona! Proces zostanie przerwany');
                    break;
                elseif er.identifier ==   'nav:navalgs:hybridastar:StartError'
                    warning('Droga nie moze zostac wyznaczona! (planer A* error) Proces zostanie przerwany');
                    break;
                else
                    rethrow(er);
                end
            end
            
            idx = 1;
            tic;
        else
            idx = idx + 1;
        end
    end
    
    if badtarget_flag
        badtarget_flag = false;
        continue;
    end
    
    disp("Navigation to point... DONE!");
    

 
    startPoint =  poses(end,:);
    
    
end

disp("MAPPING DONE");
figure
subplot(1,2,1)
show(hide_map);
title("Rzeczywista mapa")

subplot(1,2,2)
show(explo_map);
title("Ukonczona  mapa po symulacji")