clc
close all
clear 

%%-------------  PARAMETRY POCZ¥TKOWE  -----------------------------------------

maxLidarRange = 8;               % [m]
MapResolution = 40;
MaxNumOfRetry = 3;              % Maksymalna liczba prób wyznaczenia œcie¿ki dla danego punktu poczatkowego i koncowego w przypadku wystapienia bledu

% Wybor rodzaju plannera
plannerType = 'RRT'; %do wyboru 'A*'(HybridA*) lub RRT(HybridRRT*)

% Parametry plannera A*
MinTurningRadius = 0.1;         % Minimalny promien zawracania
MotionPrimitiveLength = 0.1;    % Dlugosc "odcinkow" / "³uków" w grafie (?)
   % mo¿na dodac wiecej parametrow planera - te sa podstawowe
   
%Parametry plannera RRT*
validationDistance = 0.1;
maxIterations = 10000;
minTurningRadius = 0.01;
maxConnectionDistance = 1.5;

goalRadius = 0.3;
robotRadiusOrg = 0.2;
robotRadiusTemp = robotRadiusOrg;


% ROS Node init
node_automap = ros.Node('/matlab_automap');
pub_automap = ros.Publisher(node_automap, '/matlab_velocity', 'geometry_msgs/Vector3Stamped');
rosmsg = rosmessage('geometry_msgs/Vector3Stamped');
pathIndex = 0;

%%----------------- INICJALIZACJA --------------------------------------------------------------

% Inicjacja nowej mapy
explo_map = Lidar_Init();


% Pierwszy pomiar w punkcie startowym
Lidar_subscriber = rossubscriber('/scan');
explo_map = LidarAq(explo_map, Lidar_subscriber);
[explo_map_occ, realPoses] = buildMap_and_poses(explo_map, MapResolution, maxLidarRange);


last_pose_num  = 1; % nr ostatniej pozycji przy której osiagnieto punkt eksploracyjny
explo_points=[];    % tablica na punkty eksploracyjne


% Inicjalizacja zmiennych potrzebnych w g³ównej pêtli 
parentNum = 0;              % identyfikator rodzica danej galezi 
newParent_flag = false;     % flaga podnoszona przy odnalezieniu rozgalezienia
goback_flag = false;        % flaga podnoszona przy braku nowych punktow dla danej galezi - prowadzi do powrotu do punktu rozgalezienia
child = [];                 % macierz na punkty rozgalezien dla danego identyfikatora rozgalezienia (rodzica)
middle_Pt = [];             % macierz na "punkty srodkowe" - do okreslenia obszarow zablokowanych dla wyszukiwania punktow eksploracyjnych (sposob filtracji)
parentTochild_route = [];   % macierz na zapisywanie punktów po ktorych robot moze wrocic do rozgalezienia z ktorego wychodzi galaz na ktorej sie aktualnie znajduje
parentTochild_route = [0 realPoses(end,1:2)]; % dodanie pierwszego punktu powrotnego
RetryCounter = 0;           % licznik powtózen w przypadku bledu wyznaczania trasy


simulation_time = tic;      % pomiar czasu symulacji - zwracany na koniec wykonywania programu

% PLANNER RRT* INIT (TEST PLANNERA)


%-------------------- GLÓWNA PÊTLA ---------------------------------------------------------------
figure
while true
    %%--------------- Czêœæ algortytmu odpowiadaj¹ca za rozgalezienia -------------
    
    if goback_flag % powrot do rodzica
        
        % Usuniecie punktow - rodzicow bêd¹cych punktami powrotnymi jezeli nie posiadaja dzieci - innych galezi
        to_delete = [];
        for p = 1 :  length(parentTochild_route(:,1))
            semeparent_number = find(child(:,3) == parentTochild_route(p,1)); % zebranie galezi o tym samym identyfikatorze rodzica
            if isempty(semeparent_number)
                to_delete(1,end+1) = p;                                       % w przypadku braku galezi o danym identyfikatorze, zostaje zapisany nr identyfikatora
            end
        end
        to_delete = unique(to_delete);
        parentTochild_route(to_delete,:) = [];                                % usuniêcie punktow o zapisanym identyfikatorze

        
        if  parentTochild_route(end,1) == parentNum && length(parentTochild_route(:,1))>1 %jezeli operujemy caly czas na tym samym identyfikatorze rodzica
            target_point = parentTochild_route(end,2:3); % wyznaczenie aktualnego celu jako ostatneigu punktu z listy punktow powrotnych
            parentTochild_route(end, :) = [];
        else
            temp = find(child(:,3) == parentNum); % zebranie punktow galezi (dzieci) o tym samym identyfikatorze rodzica dla aktualnego identyfikatora
            if ~isempty(temp)
                
                sameparent_points = child(temp,:);                                                                     % tworzy tymaczasow¹ macierz dzieci posiadaj¹cych tych samych rodziców                
                points_withrating = [exploratory_points_rating(sameparent_points(:,1:2), explo_map_occ, realPoses(end, :), maxLidarRange) temp]; % macierz punktów w formacie [x y rate child_index]
                
                [target_point, ~, target_num] = best_point(points_withrating(:,1:2), points_withrating(:,3));          % wyznaczenie punktu target
                child(points_withrating(target_num, 4), :) = [];                                                       % usuniecie z listy dzieci punktu target
                
                goback_flag = false;                                                                                   % powrot do punktu - rodzica zostal zakonczony
            else
                parentNum = parentNum -1;                                                                              % jezeli nie ma galezi (dzieci) dla danego identyfikatora rodzica
                continue;
            end
        end
        
    else % flaga o powrocie do punktu rozgalezienia nie zostala podniesiona
        if parentNum == 0                                                   % na pocz¹tku gdy nie ma galezi nadpisywana jest pierwsza linijka
            parentTochild_route(end,:) =[ 0 realPoses(end,1:2)];
        elseif newParent_flag
            parentTochild_route(end+1,:) =[ parentNum  realPoses(end,1:2)]; % dopisywany jest kolejny rodzic, ale tylko przy zwiêkszeniu identyfikatora rodzica
            newParent_flag = false;
        end
        
        % Wyznaczenie punktów eksploracyjnych dla pozycji od last_pose_num do konca pozycji
        explo_points = [];
        disp("Exploratory points search START!");
        explo_points = exploratory_points2(explo_map_occ, explo_points, last_pose_num, realPoses, middle_Pt, maxLidarRange, 0.05 );
        disp("Exploratory points search DONE!");
        % weryfikacja dzieci wzglêdem osiagnietych pozycji
        if ~isempty(child) && ~isempty(middle_Pt)
            child = verify_PointsToPosses(child, middle_Pt);
        end
        
        % weryfikacja punktów wzgledem osiagnietych pozycji
        if ~isempty(explo_points) && ~isempty(middle_Pt)
            explo_points = verify_PointsToPosses(explo_points,middle_Pt);
        end
        
        if isempty(explo_points) %jezeli po tej operacji nie ma ani punktow-dzieci ani punktow eksploracyjnych mapowanie zostaje zakonczone
            if isempty(child)
                break;
            else
                goback_flag = true; % jezeli nie ma punktow eksploraycjnych ale sa punkty dzieci zostaje rozpoczeta sekwencja powrotna
                continue;
            end
        else
            if length(explo_points(:,1)) > 1
                
                parentNum = parentNum +1;
                newParent_flag = true;
                [target_point, explo_points, ~] = best_point(explo_points(:,1:2), explo_points(:,3));
                child = [child ; explo_points(:,1:2) repmat(parentNum, length(explo_points(:,1)), 1)];
                
            else
                target_point =  explo_points(1,1:2) ;
            end
        end
    end
    %---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    
    explo_map = LidarAq(explo_map, Lidar_subscriber);
    [explo_map_occ, realPoses] = buildMap_and_poses(explo_map, MapResolution, maxLidarRange);   
    
    % Przypisanie punktu pocz¹tkowego i koncowego wraz z wyznaczeniem k¹ta
   
    start_Location = [realPoses(end,1:2), realPoses(end,3)+pi/2] ;
    %start_Location = [realPoses(end,1:2), Angle2Points(realPoses(end,1:2), target_point(1,1:2))] ;
    stop_Location = [target_point Angle2Points(realPoses(end,1:2), target_point(1,1:2) )];
    
    last_pose_num  = length(realPoses(:,1));
    
    disp("Map Processing START!")
    % oszukanie zajetosci przez binaryzacjê aktualnej mapy i kolejn¹ erozjê
    % i dylatacje w celu poprawy wygl¹du mapy oraz umozliwienia jazdy w
    % nieznane
    binMap = imbinarize(occupancyMatrix(explo_map_occ),0.5);
    se = strel('cube',4);
    se2 = strel('cube',8);
    binMap  = imerode(binMap ,se);
    binMap  = imdilate(binMap ,se2);   
%     temp_map = occupancyMap(imbinarize(occupancyMatrix(explo_map_occ),0.51), MapResolution);
    temp_map = occupancyMap(binMap , MapResolution); %
    temp_map.LocalOriginInWorld = explo_map_occ.LocalOriginInWorld;
%     inflate(temp_map, robotRadiusTemp);
    disp("Map Processing DONE!")

    % Wyznaczenie najkrótszej œcie¿ki
    disp("Planner START!");  
    
    % PLANNER A*
    if plannerType == 'HA*'
        vMap = validatorOccupancyMap;
        vMap.Map = temp_map;
        planner = plannerHybridAStar(vMap, 'MinTurningRadius', MinTurningRadius, 'MotionPrimitiveLength',MotionPrimitiveLength); % stworzenie obiektu planner
        
        try
            plannerPoses = plannerProcess(planner, start_Location, stop_Location);
        catch er
            switch er.identifier
                case 'nav:navalgs:astar:OccupiedLocation'
                    warning('Droga nie moze zostac wyznaczona! Powod : OccupiedLocation');
                    if robotRadiusTemp <=0.06
                        break
                    end
                    robotRadiusTemp = robotRadiusTemp - 0.03;
                    continue;
                case 'nav:navalgs:hybridastar:StartError'
                    warning('Droga nie moze zostac wyznaczona! (planer A* error)  : StartError');
                    if robotRadiusTemp <=0.06
                        break
                    end
                    robotRadiusTemp = robotRadiusTemp - 0.03;
                    continue;
                case 'nav:navalgs:hybridastar:GoalError'
                    warning('Droga nie moze zostac wyznaczona! (planer A* error)  : GoalError');
                    if robotRadiusTemp <=0.06
                        break
                    end
                    robotRadiusTemp = robotRadiusTemp - 0.03;
                    continue;
                otherwise
                    rethrow(er);
            end
        end
        % gdy œciezka zostanie wyznaczona:
        robotRadiusTemp = robotRadiusOrg;
    % PLANNER RRT*    
    elseif plannerType == 'RRT' 
        plannerStatus = true;
        plannerFirstIt = true;
        while true
            vehDim = vehicleDimensions(0.38, 0.25, 0.2,'FrontOverhang',0.04,'RearOverhang',0.3, 'Wheelbase', 0.005);

            ccConfig = inflationCollisionChecker(vehDim, 'InflationRadius', robotRadiusTemp, 'NumCircles',1);
            tic
            costmap = vehicleCostmap(temp_map,'CollisionChecker',ccConfig );%TO JAKOS D£UGO CHODZI
            toc

            planner = pathPlannerRRT(costmap);
            planner.MaxIterations = maxIterations;
            planner.ConnectionDistance = maxConnectionDistance;
            planner.MinTurningRadius = minTurningRadius;
            planner.GoalTolerance = [0.2, 0.2, 360];
            planner.ConnectionMethod = 'Dubins';


            if plannerFirstIt
                stop_Location = changePointToClosest(temp_map, costmap, stop_Location);
                plannerFirstIt = false;
                if isempty(stop_Location)
                    disp('Cel zostal yznaczony mocno poza mapa')
                    break
                end
            end


            if exist('plannerPosesObj', 'var')
                clear plannerPosesObj
            end
        
            try
            [plannerPosesObj,tree] = plan(planner,[start_Location(1:2), rad2deg(start_Location(3))], ...
                                           [stop_Location(1:2), rad2deg(stop_Location(3))]);        
            catch er
                warning(['RRT* nie moze wyznaczyc trasy, powod : ', er.identifier, er.message]);
                if robotRadiusTemp <=0.06
                    plannerStatus= false;
                    break
                end
                if ~exist('plannerPosesObj', 'var')
                    robotRadiusTemp = robotRadiusTemp - 0.02;
                    continue;   
                else
                    disp('error sie pojawi³ ale mamy obiekt')
                end

            end
            if plannerPosesObj.Length == 0
                disp('Planner return length=0 path!')
                robotRadiusTemp = robotRadiusTemp - 0.02;
                continue
            end
            break
        end
        
        if ~plannerStatus
            continue
        end
        clear('plannerStatus')
        clear('plannerFirstIt')
        
        robotRadiusTemp = robotRadiusOrg;        
        [refPoses,refDirections]  = interpolate(plannerPosesObj);

%         hold on
%         plot(planner)
%         legend('hide')
        approxSeparation = 0.09; % meters
        numSmoothPoses = round(plannerPosesObj.Length / approxSeparation);
        try
            [plannerPoses,~] = smoothPathSpline(refPoses,refDirections,numSmoothPoses);
        catch er
            warning(['Problem ze smoothPathSpline : ', er.identifier]);
            plannerPoses = refPoses;
        end
        plannerPoses = [plannerPoses(:,1:2) deg2rad(plannerPoses(:,3))];
        plannerPoses(end,3) = plannerPoses(end-1,3);
        plannerPoses = [start_Location; plannerPoses];

end
    
    % Pocz¹tek przemieszczenia pojazdu do zadanego punktu
    disp("Navigation to point...");
    hold on
    show(explo_map_occ);
    hold on 
    plot(plannerPoses(:,1), plannerPoses(:,2), '.r');
    hold on
    plot(refPoses(:,1), refPoses(:,2), '*r');
    hold on
    plot(stop_Location(:,1), stop_Location(:,2), 'sb')
    if ~isempty(child) 
        if exist('child_plot', 'var')
            delete(child_plot);
        end
        hold on
        child_plot = plot(child(:,1),child(:,2), '.b');
    end
    
    
    sendPath([],pub_automap, rosmsg, pathIndex) % przesy³anie danych
    sendPath(plannerPoses ,pub_automap, rosmsg, pathIndex) % przesy³anie danych
 
    distanceToGoal = norm(realPoses(end,1:2) - plannerPoses(end, 1:2));
    
    lastDistanceToGoal = distanceToGoal;
    RetryCounter = 0;

    while( distanceToGoal > goalRadius )
        

        % Akwizycja danych z lidaru i przypisanie do mapy oraz aktualizacja pozycji
        explo_map = LidarAq(explo_map, Lidar_subscriber);
        [explo_map_occ, realPoses] = buildMap_and_poses(explo_map, MapResolution, maxLidarRange);
        
        
        
        
        % Aktualizacja odleg³oœci od koñca wyznaczonej œcie¿ki
        distanceToGoal = norm(realPoses(end,1:2) - plannerPoses(end, 1:2));
        
        disp('NEXT MEASURMENT')
        disp(num2str(distanceToGoal))
        
        if distanceToGoal < lastDistanceToGoal + 0.02 && distanceToGoal > lastDistanceToGoal - 0.02
            RetryCounter = RetryCounter+1;
            if RetryCounter >= MaxNumOfRetry
                break
            end
        else 
            RetryCounter = 0;
        end
        
        lastDistanceToGoal = distanceToGoal;
        
        % Wyznaczenie okregow filtrujacych
        middle_Pt(end+1,:) = middle_points2(explo_map_occ,realPoses(end,:));
         
        %RetryCounter = 0; 
    
        
    end
    sendPath([],pub_automap, rosmsg, pathIndex) % stop motors
    disp("Navigation to point... DONE!");
    
    startPoint =  plannerPoses(end,:); % dodanie jako kolejnej pozycji startowej ostatniej osi¹gniêtej pozycji - aktulanej pozycji robota
    
end
toc(simulation_time) % zatrzymanie timera odpowiadzalnego za pomiar czasu symulacji

show(explo_map_occ);
%%
disp("MAPPING DONE");
figure
show(explo_map_occ);
title("Ukonczona  mapa po symulacji")