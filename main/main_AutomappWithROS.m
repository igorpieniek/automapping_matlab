clc
close all
clear 

%%-------------  PARAMETRY POCZ¥TKOWE  -----------------------------------------

maxLidarRange = 8;               % [m]
MapResolution = 40;
MaxNumOfRetry = 3;              % Maksymalna liczba prób wyznaczenia œcie¿ki dla danego punktu poczatkowego i koncowego w przypadku wystapienia bledu

 
%Parametry plannera RRT*
robotRadiusConfig.original = 0.3;
robotRadiusConfig.step = 0.02;
robotRadiusConfig.min = 0.06;
plannerConfig.maxIterations = 10000;
plannerConfig.ConnectionDistance = 1;
plannerConfig.minTurningRadius = 0.01;
plannerConfig.goalTolerance = [0.2, 0.2, 360];
pathPointsDistance = 0.04;


goalRadius = 0.3;


vehDim = vehicleDimensions(0.38, 0.25, 0.2,'FrontOverhang',0.04,'RearOverhang',0.3, 'Wheelbase', 0.005);
ccConfigOrg = inflationCollisionChecker(vehDim, 'InflationRadius', robotRadiusConfig.original , 'NumCircles',1);

% ROS Node init
node_automap = ros.Node('/matlab_automap');
pub_automap = ros.Publisher(node_automap, '/matlab_velocity', 'geometry_msgs/Vector3Stamped');
rosmsg = rosmessage('geometry_msgs/Vector3Stamped');
pathIndex = 0;

%%----------------- INICJALIZACJA --------------------------------------------------------------

% Inicjacja nowej mapy
exploMap = Lidar_Init();


% Pierwszy pomiar w punkcie startowym
Lidar_subscriber = rossubscriber('/scan');
exploMap = LidarAq(exploMap, Lidar_subscriber);
[exploMapOcc, realPoses] = buildMap_and_poses(exploMap, MapResolution, maxLidarRange);


lastPoseNum  = 1; % nr ostatniej pozycji przy której osiagnieto punkt eksploracyjny
allPlannerPoses = [];
exploratoryInflateRatio = 0.05; % wspó³czynnik funkcji inflate potrzebny przy przetwarzaniu aktualnej mapy w g³ównej funkcji wyszukuj¹cej obszary
                                % do eksploracji - exploratory_points2


% Inicjalizacja zmiennych potrzebnych w g³ównej pêtli 
parentNum = 0;              % identyfikator rodzica danej galezi 
newParentFlag = false;     % flaga podnoszona przy odnalezieniu rozgalezienia
gobackFlag = false;        % flaga podnoszona przy braku nowych punktow dla danej galezi - prowadzi do powrotu do punktu rozgalezienia
exploPoints = [];                 % macierz na punkty rozgalezien dla danego identyfikatora rozgalezienia (rodzica)
middlePoints = [];             % macierz na "punkty srodkowe" - do okreslenia obszarow zablokowanych dla wyszukiwania punktow eksploracyjnych (sposob filtracji)
parentTochildRoute = [];   % macierz na zapisywanie punktów po ktorych robot moze wrocic do rozgalezienia z ktorego wychodzi galaz na ktorej sie aktualnie znajduje
parentTochildRoute = [0 realPoses(end,1:2)]; % dodanie pierwszego punktu powrotnego
RetryCounter = 0;           % licznik powtózen w przypadku bledu wyznaczania trasy

DFS = DFSalgorithm;

simulation_time = tic;      % pomiar czasu symulacji - zwracany na koniec wykonywania programu



%-------------------- GLÓWNA PÊTLA ---------------------------------------------------------------
figure
figAxis = [-5 3 -5 4];
while true
    %%--------------- Czêœæ algortytmu odpowiadaj¹ca za rozgalezienia -------------
    
    if gobackFlag % powrot do rodzica
            [parentTochildRoute,...
            exploPoints,...
            parentNum,...
            target_point,...
            gobackFlag,...
            continueStatus ] = DFS.goBack(parentTochildRoute,...
                                          exploPoints,...
                                          parentNum,...
                                          exploMapOcc,...
                                          realPoses,...
                                          maxLidarRange );
        if continueStatus
            continue
        end      
        
    else % flaga o powrocie do punktu rozgalezienia nie zostala podniesiona
        [parentTochildRoute,...
            exploPoints,...
            parentNum,...
            newParentFlag,...
            target_point,...
            gobackFlag,...
            breakStatus  ] = DFS.goDeep(parentTochildRoute, ...
                                        exploPoints,...
                                        parentNum,...
                                        newParentFlag,...
                                        realPoses,...
                                        exploMapOcc,...
                                        lastPoseNum ,...
                                        middlePoints,...
                                        maxLidarRange,...
                                        exploratoryInflateRatio);
        if gobackFlag
            continue
        elseif breakStatus
            break
        end                
    end
    %---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    
    exploMap = LidarAq(exploMap, Lidar_subscriber);
    [exploMapOcc, realPoses] = buildMap_and_poses(exploMap, MapResolution, maxLidarRange); 
    if exist('lidarPlot', 'var')
        delete(lidarPlot);
    end
    hold on
    lidarPlot = plot(realPoses(:,1), realPoses(:,2),'-k.','DisplayName','LIDAR');
    % Przypisanie punktu pocz¹tkowego i koncowego wraz z wyznaczeniem k¹ta
   
    start_Location = [realPoses(end,1:2), realPoses(end,3)+pi/2] ;

    stop_Location = [target_point Angle2Points(realPoses(end,1:2), target_point(1,1:2) )];
    
    lastPoseNum  = length(realPoses(:,1));
    

    
    disp("Map Processing START!")
    % oszukanie zajetosci przez binaryzacjê aktualnej mapy i kolejn¹ erozjê
    % i dylatacje w celu poprawy wygl¹du mapy oraz umozliwienia jazdy w
    % nieznane
    temp_map = mapConversion(exploMapOcc, MapResolution);
    disp("Map Processing DONE!")

    % Wyznaczenie najkrótszej œcie¿ki
    disp("Planner START!");  

    [path,differentRadiusFlag] = configuredRRTplanner(start_Location,...
                                                      stop_Location,...
                                                      vehDim,...
                                                      robotRadiusConfig,...
                                                      temp_map,...
                                                      plannerConfig,...
                                                      pathPointsDistance);
    if isempty(path)
        continue;
    end
    
    
    
    disp("Navigation to point...");
    
    hold on
    show(exploMapOcc,'FastUpdate', true);
    axis(figAxis)
    hold on
    if exist('plannerPlot', 'var')
        delete(plannerPlot);
    end
    plannerPlot = plot(path(:,1), path (:,2), '-r.','DisplayName','Œcie¿ka');
    
    hold on
    if exist('targetPlot', 'var')
        delete(targetPlot);
    end
    targetPlot = plot(stop_Location(:,1), stop_Location(:,2), 'sb','DisplayName','Aktualny cel');
    
    if ~isempty(exploPoints) 
        if exist('child_plot', 'var')
            delete(child_plot);
        end
        hold on
        child_plot = plot(exploPoints(:,1),exploPoints(:,2), '.', 'Color', '#EDB120','DisplayName','Punkty eksploracyjne');
    end
    legend()
    
    % Wysy³anie œcie¿ki 
    sendPath([],pub_automap, rosmsg, pathIndex) % przesy³anie danych
    sendPath(path ,pub_automap, rosmsg, pathIndex) % przesy³anie danych
 
    distanceToGoal = norm(realPoses(end,1:2) - path(end, 1:2));
    
    lastDistanceToGoal = distanceToGoal;
    RetryCounter = 0;

    while( distanceToGoal > goalRadius )
        

        % Akwizycja danych z lidaru i przypisanie do mapy oraz aktualizacja pozycji
        exploMap = LidarAq(exploMap, Lidar_subscriber);
        [exploMapOcc, realPoses] = buildMap_and_poses(exploMap, MapResolution, maxLidarRange);
        
        % Wyznaczenie okregow filtrujacych
        middlePoints(end+1,:) = middle_points2(exploMapOcc,realPoses(end,:), middlePoints);
        
        hold on
        show(exploMapOcc,'FastUpdate', true);
        axis(figAxis)
        hold on
        if exist('lidarPlot', 'var')
            delete(lidarPlot);
        end
        lidarPlot = plot(realPoses(:,1), realPoses(:,2),'-k.','DisplayName','LIDAR');
        
        drawnow
       
        
        % Aktualizacja odleg³oœci od koñca wyznaczonej œcie¿ki
        distanceToGoal = norm(realPoses(end,1:2) - path(end, 1:2));
        
        disp('NEXT MEASURMENT')
        disp(num2str(distanceToGoal))
                    
        temp_map = mapConversion(exploMapOcc, MapResolution);
        costmap = vehicleCostmap(temp_map,'CollisionChecker',ccConfigOrg );
        
        
        if differentRadiusFlag && checkFree(costmap, [realPoses(end,1:2) rad2deg(realPoses(end,3))])
            warning('Vehicle left occupied area!')
            differentRadiusFlag = false;
            break
        end
        
        % cyklicznie sprawdza czy œcie¿ka wraz z kolejnymi elementami na
        % mapie jest okej, jak nie to ucina j¹
        occupated = checkOccupied(costmap, [path(:,1:2) rad2deg(path(:,3))] );
        if any(occupated) 
            if any(occupated(end-2:end))
                warning(['EXECUTE PART: last positions in blocked area !!!:',num2str(find(occupated==1)'), 'length:', num2str(length(occupated)) ]);
                break
            else
                warning(['EXEC PART : poses in occupated area!! Poses number:',num2str(find(occupated==1)') ]);
                
            end
            
        end
        
        if checkOccupied(costmap, [realPoses(end,1:2) rad2deg(realPoses(end,3))])
            disp("ROUTE OCCUPIED ")
        end
        
        % Sprawdzenie czy robot stan¹³ w miejscu  - powodu pozornego dotarcia do celu lub b³êdu
        if distanceToGoal < lastDistanceToGoal + 0.02 && distanceToGoal > lastDistanceToGoal - 0.02
            RetryCounter = RetryCounter+1;
            if RetryCounter >= MaxNumOfRetry
                break
            end
        else 
            RetryCounter = 0;
        end
        
        lastDistanceToGoal = distanceToGoal;
         
    end
    sendPath([],pub_automap, rosmsg, pathIndex) % stop motors
    disp("Navigation to point... DONE!");
    
    allPlannerPoses =  [allPlannerPoses; path];
end
toc(simulation_time) % zatrzymanie timera odpowiadzalnego za pomiar czasu symulacji


%%
disp("MAPPING DONE");
figure
show(exploMapOcc);
title("Ukonczona  mapa po symulacji")