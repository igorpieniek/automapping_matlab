clc
close all
clear 

%%-------------  PARAMETRY POCZ¥TKOWE  -----------------------------------------

maxLidarRange = 8;        % maksymalny zasiêg pomiarowy Lidaru [m]
MapResolution = 40;       % rozdzielczoœæ mapy
MaxNumOfRetry = 3;        % Maksymalna liczba prób wyznaczenia œcie¿ki dla danego punktu poczatkowego i koncowego w przypadku wystapienia bledu

scanAngleOffset = -pi/2;  % offset zwi¹zany z obrotem odczytanego skanu, tak aby 
                          % pocz¹tkowa orientacja pojazdu i skanu by³y identyczne

%Parametry plannera RRT*
robotRadiusConfig.original = 0.3;
robotRadiusConfig.step = 0.02;
robotRadiusConfig.min = 0.06;
plannerConfig.maxIterations = 10000;
plannerConfig.ConnectionDistance = 1;
plannerConfig.minTurningRadius = 0.01;
plannerConfig.goalTolerance = [0.2, 0.2, 360];
pathPointsDistance = 0.04;

%Parametry dziêki którym weryfikowana jest poprawnoœæ œledzenia œcie¿ki
goalRadius = 0.3; %odleg³oœc od wybranego celu, poniezej której zostanie uznane osi¹gniecie celu
isStandingMargin = 0.02; %margines miêdzy kolejnymiu pozycjami. Jezeli poprzednia aproksymowana pozycja
                         %w porównaniu z aktualn¹ bêdzie +- ta wartoœæ - rozpatrywane jest jako b³¹d wykonywania œcie¿ki 

exploratoryInflateRatio = 0.05; % wspó³czynnik funkcji inflate potrzebny przy przetwarzaniu aktualnej mapy w g³ównej funkcji wyszukuj¹cej obszary
                                % do eksploracji - exploratory_points2
                                
%Definicja wymiarów robota oraz obiektu odpowiadajacego za sprawdzanie marginesu podczas wyznaczania œcie¿ki                         
vehDim = vehicleDimensions(0.38, 0.25, 0.2,'FrontOverhang',0.04,'RearOverhang',0.3, 'Wheelbase', 0.005);
ccConfigOrg = inflationCollisionChecker(vehDim, 'InflationRadius', robotRadiusConfig.original , 'NumCircles',1);

% ROS Node init
node_automap = ros.Node('/matlab_automap');
pub_automap = ros.Publisher(node_automap, '/matlab_velocity', 'geometry_msgs/Vector3Stamped');
rosmsg = rosmessage('geometry_msgs/Vector3Stamped');

%%----------------- INICJALIZACJA --------------------------------------------------------------

% Inicjacja nowej mapy
exploMap = Lidar_Init(maxLidarRange, MapResolution);

% Pierwszy pomiar w punkcie startowym
Lidar_subscriber = rossubscriber('/scan');
exploMap = LidarAq(exploMap, Lidar_subscriber, scanAngleOffset);
[exploMapOcc, allPoses] = buildMap_and_poses(exploMap, MapResolution, maxLidarRange);

% Inicjacja obiektu klasy DFSalgorithm - zbudowanej na potrzeby skryptu                                
DFS = DFSalgorithm(allPoses(end,1:2), maxLidarRange, exploratoryInflateRatio);        

% Wartoœci poczatkowe, inicjacja potrzebnych zmiennych
middlePoints = [];         % macierz na "punkty srodkowe" - do okreslenia obszarow zablokowanych dla wyszukiwania punktow eksploracyjnych (sposob filtracji)
RetryCounter = 0;          % licznik powtózen w przypadku bledu wyznaczania trasy
lastPoseNum  = 1;          % nr ostatniej pozycji przy której osiagnieto punkt eksploracyjny
allPlannerPoses = [];      % macierz wszystkich punktów œciezki

% Rozpoczêcie pomiaru czasu wykonywania
processTime = tic;      

%-------------------- GLÓWNA PÊTLA ---------------------------------------------------------------
figure
figAxis = [-5 3 -5 4];
while true
    %%--------------- Czêœæ algortytmu odpowiadaj¹ca za rozgalezienia - algorytm DFS-------------
     DFSoutput = DFS.process(exploMapOcc, allPoses(lastPoseNum:end, :), middlePoints);
     if isempty(DFSoutput.target)
         break;
     end
    %---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    % Nowy pomiar
    exploMap = LidarAq(exploMap, Lidar_subscriber, scanAngleOffset);
    [exploMapOcc, realPoses] = buildMap_and_poses(exploMap, MapResolution, maxLidarRange);
    
    % Aktualizacja
    if exist('lidarPlot', 'var')
        delete(lidarPlot);
    end
    hold on
    lidarPlot = plot(realPoses(:,1), realPoses(:,2),'-k.','DisplayName','LIDAR');
    
    % Okreœlenie celu oraz punktu startowego
    start_Location = [realPoses(end,1:2), realPoses(end,3)+pi/2] ;
    stop_Location = [DFSoutput.target Angle2Points(realPoses(end,1:2), DFSoutput.target(1,1:2) )];
    
    % Zapisanie numeru ostatniej zapsianej pozycji
    lastPoseNum  = length(realPoses(:,1));
    
    % Konwersja aktualnej mapy
    temp_map = mapConversion(exploMapOcc);

    % Wyznaczenie najkrótszej œcie¿ki - RRT*
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
    
    % Wyœwietlenie wyników dzia³ania plannera   
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
    
    if ~isempty(DFSoutput.exploPoints) 
        if exist('child_plot', 'var')
            delete(child_plot);
        end
        hold on
        child_plot = plot(DFSoutput.exploPoints(:,1),DFSoutput.exploPoints(:,2), '.', 'Color', '#EDB120','DisplayName','Punkty eksploracyjne');
    end
    legend()
    
    % Wysy³anie œcie¿ki do sieci ROS (do kontrolera œcie¿ki na Raspberry Pi)
    sendPath([],pub_automap, rosmsg) % przesy³anie danych
    sendPath(path ,pub_automap, rosmsg) % przesy³anie danych
    
    % Rozpoczêcie procesu jazdy oraz weryfikacji poprawnoœci przemieszczania siê
    disp("Navigation to point...");
    distanceToGoal = norm(realPoses(end,1:2) - path(end, 1:2));
    lastDistanceToGoal = distanceToGoal;
    RetryCounter = 0;
    while( distanceToGoal > goalRadius )
        
        % Akwizycja danych z lidaru i przypisanie do mapy oraz aktualizacja pozycji      
        exploMap = LidarAq(exploMap, Lidar_subscriber, scanAngleOffset);
        [exploMapOcc, realPoses] = buildMap_and_poses(exploMap, MapResolution, maxLidarRange);
        
        % Wyznaczenie okregow filtrujacych
        middlePoints(end+1,:) = middle_points2(exploMapOcc,realPoses(end,:), middlePoints);
        
        % Aktualizacja wyœwietlanej mapy zajêtoœci
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
        disp('NEXT MEASURMENT')       
        distanceToGoal = norm(realPoses(end,1:2) - path(end, 1:2));
        disp(num2str(distanceToGoal))
        
        % Konwersja mapy zajêtoœci
        temp_map = mapConversion(exploMapOcc);       
        costmap = vehicleCostmap(temp_map,'CollisionChecker',ccConfigOrg );
               
        if differentRadiusFlag && checkFree(costmap, [realPoses(end,1:2) rad2deg(realPoses(end,3))])
            warning('Vehicle left occupied area!')
            differentRadiusFlag = false;
            break
        end
        
        % Cykliczna weryfikacja czy œcie¿ka jest w niezajêtej przestrzeni -
        % jak jej koniec jest to zatrzymuje proces
        occupated = checkOccupied(costmap, [path(:,1:2) rad2deg(path(:,3))] );
        if any(occupated) 
            if any(occupated(end-2:end))
                warning(['EXECUTE PART: last positions in blocked area !!!: ',num2str(find(occupated==1)'), 'length:', num2str(length(occupated)) ]);
                break
            else
                warning(['EXECUTE PART : poses in occupated area!! Poses number: ',num2str(find(occupated==1)') ]);
                
            end           
        end
        
        if checkOccupied(costmap, [realPoses(end,1:2) rad2deg(realPoses(end,3))])
            disp("ROUTE OCCUPIED ")
        end
        
        % Sprawdzenie czy robot stan¹³ w miejscu  - powodu pozornego dotarcia do celu lub b³êdu
        if distanceToGoal < lastDistanceToGoal + isStandingMargin && distanceToGoal > lastDistanceToGoal - isStandingMargin
            RetryCounter = RetryCounter + 1;
            if RetryCounter >= MaxNumOfRetry
                break
            end
        else 
            RetryCounter = 0;
        end
        
        lastDistanceToGoal = distanceToGoal;      
    end
    sendPath([],pub_automap, rosmsg) % Stop pracy kontrolera
    disp("Navigation to point... DONE!");
    allPlannerPoses =  [allPlannerPoses; path];
end
disp("MAPPING DONE");
toc(processTime) % zatrzymanie timera i wyœwietlenie czasu procesu autonomicznego mapowania


%% Wyœwietlenie koñcowych wyników
disp("MAPPING DONE");
figure
show(exploMapOcc);
title("Ukonczona  mapa po symulacji")