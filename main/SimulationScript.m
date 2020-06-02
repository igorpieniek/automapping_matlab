% V3 - dzia³aj¹ca, poprawiony powrót do rodzica

clc
close all
clear all

%%-------------  PARAMETRY POCZ¥TKOWE  -----------------------------------------


RealMap = 'C:\Users\Igor\Dysk Google\Studia\IN¯YNIERKA\maps\sym_part1.png';     % Wczytywana rzeczywista mapa 
MapResolution = 34;              % Rozdzielczoœæ mapy - iloœæ pikseli przypadaj¹ca na metr  
startPoint = [1.3 2.5 pi/2];      % Punkt startowy robota oraz jego pocz¹tkowe po³o¿enie k¹towe [x y rad]

maxLidarRange = 12;               % [m]
AngleRangeBoundaries = [-pi pi]; % Maksymalny zakres katowy (dla innego zakresu ni¿ 360 stopni mo¿e nie funkcjonowaæ poprawnie)
RangeNoise = 0.001;              % Szum przy okreœlaniu zasiêgów

% Wybor rodzaju plannera
plannerType = "RRT*"; %do wyboru 'A*'(HybridA*) lub RRT*(HybridRRT*)

% Parametry plannera A*
MinTurningRadius = 0.3;         % Minimalny promien zawracania
MotionPrimitiveLength = 0.3;    % Dlugosc "odcinkow" / "³uków" w grafie (?)
   % mo¿na dodac wiecej parametrow planera - te sa podstawowe
   
   %Parametry plannera RRT*
validationDistance = 0.5;
maxIterations = 10000;
minTurningRadius = 0.001;
maxConnectionDistance = 0.5;

robotSize = 0.2;
robotRadiusOrg = 0.3;
robotRadiusTemp = robotRadiusOrg; 

vehDim = vehicleDimensions(0.38, 0.25, 0.2,'FrontOverhang',0.04,'RearOverhang',0.3, 'Wheelbase', 0.005);
ccConfigOrg = inflationCollisionChecker(vehDim, 'InflationRadius', robotRadiusOrg, 'NumCircles',1);



% Wyniki
show_child_points = true;       % Aktualne przedstawienie wszystkich punktów rozgalezien
show_robot_path = true;         % Wyswietlanie przebytej sciezki przez robota
show_current_target = false;     % Wyswietlanie aktualego punktu uznanego jako cel do ktorego zostaje wyznaczona sciezka

%%----------------- INICJALIZACJA --------------------------------------------------------------
% Inicjalizacja ukrytej rzeczywistej mapy mapy

grayimage = rgb2gray(imread(RealMap));
bwimage = grayimage < 128;
hide_map = occupancyMap(bwimage, MapResolution);
show(hide_map)

% Inicjacja nowej mapy
explo_map = occupancyMap( (ones(size(grayimage))).*0.5, MapResolution);

% Inicjalizacja wirtualnego lidaru oraz jego parametrów
rangefinder = rangeSensor('HorizontalAngle', AngleRangeBoundaries,'RangeNoise', RangeNoise,'Range' , [0 maxLidarRange]);
numReadings = rangefinder.NumReadings;

last_pose_num  = 1; % nr ostatniej pozycji przy której osiagnieto punkt eksploracyjny
explo_points=[];    % tablica na punkty eksploracyjne
all_poses = [];     % tablica wszystkich kolejno osi¹ganych pozycji
all_poses(end+1,:) = startPoint; % dodanie pierwszego punktu

% Pierwszy pomiar w punkcie startowym
[ranges, angles] = rangefinder(startPoint, hide_map);
insertRay(explo_map,  startPoint, ranges, angles,  rangefinder.Range(end));


% Inicjalizacja zmiennych potrzebnych w g³ównej pêtli 
parentNum = 0;              % identyfikator rodzica danej galezi 
newParent_flag = false;     % flaga podnoszona przy odnalezieniu rozgalezienia
goback_flag = false;        % flaga podnoszona przy braku nowych punktow dla danej galezi - prowadzi do powrotu do punktu rozgalezienia
child = [];                 % macierz na punkty rozgalezien dla danego identyfikatora rozgalezienia (rodzica)
middle_Pt = [];             % macierz na "punkty srodkowe" - do okreslenia obszarow zablokowanych dla wyszukiwania punktow eksploracyjnych (sposob filtracji)
parentTochild_route = [];   % macierz na zapisywanie punktów po ktorych robot moze wrocic do rozgalezienia z ktorego wychodzi galaz na ktorej sie aktualnie znajduje
parentTochild_route = [0 startPoint(1,1:2)]; % dodanie pierwszego punktu powrotnego
RetryCounter = 0;           % licznik powtózen w przypadku bledu wyznaczania trasy

exploratoryInflateRatio = 0.05; % wspó³czynnik funkcji inflate potrzebny przy przetwarzaniu aktualnej mapy w g³ównej funkcji wyszukuj¹cej obszary
                                % do eksploracji - exploratory_points2

DFS = DFSalgorithm;         % za³¹czenie funkcji algorytmu DFS zbudowanego na potrzeby skryptu
viz = HelperUtils;          % zalaczenie narzedzi do wyœwietlania robota (byc moze do wyrzucenia - teraz korzysta tylko ze znacznika robota)

simulation_time = tic;      % pomiar czasu symulacji - zwracany na koniec wykonywania programu

%%%%%%do legendy%%%%%%%%%%%
backPlotFirst = true;
headPlotFirst = true;
exploPlotFirst = true;
targetPlotFirst = true;

%-------------------- GLÓWNA PÊTLA SYMULACJI---------------------------------------------------------------
figure  
while true
    %%--------------- Czêœæ algortytmu odpowiadaj¹ca za rozgalezienia -------------
    
    if goback_flag % powrot do rodzica
        [parentTochild_route,...
            child, parentNum,...
            target_point,...
            goback_flag,...
            continueStatus ] = DFS.goBack(parentTochild_route,...
                                          child,...
                                          parentNum,...
                                          explo_map,...
                                          all_poses,...
                                          maxLidarRange );
        if continueStatus
            continue
        end

    else % flaga o powrocie do punktu rozgalezienia nie zostala podniesiona
        [parentTochild_route,...
            child,...
            parentNum,...
            newParent_flag,...
            target_point,...
            goback_flag,...
            continueStatus,...
            breakStatus  ] = DFS.goDeep(parentTochild_route, ...
                                        child,...
                                        parentNum,...
                                        newParent_flag,...
                                        all_poses,...
                                        explo_map,...
                                        last_pose_num ,...
                                        middle_Pt,...
                                        maxLidarRange,...
                                        exploratoryInflateRatio);
        if continueStatus
            continue
        elseif breakStatus
            break
        end        
    end
    %---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    % Aktualizacja wyswietlania wynikow
    if ~isempty(child) && show_child_points
        if exist('child_plot', 'var')
            delete(child_plot);

        end
        hold on
        child_plot = plot(child(:,1),child(:,2), '.b','DisplayName','Punkty eksploracyjne');
        legend()
        
    end
    if ~isempty(target_point) && show_current_target
        if exist('target_plot', 'var')
            delete(target_plot);
        end
        hold on
        target_plot = plot(target_point(1,1), target_point(1,2),'or','HandleVisibility','off');
        
        %poradzenie sobie z jedenorazow¹ legend¹
        if targetPlotFirst
            targetPlotFirst = false;
            plot(target_point(1,1), target_point(1,2),'or','DisplayName','Aktualny cel')
            legend()
        end
    end
    
    % Przypisanie punktu pocz¹tkowego i koncowego wraz z wyznaczeniem k¹ta
    start_Location = startPoint;
    stop_Location = [target_point Angle2Points(startPoint(1,1:2), target_point(1,1:2) )];
    
    last_pose_num  = length(all_poses(:,1));
    
    % Inicjalizacja planera
    disp("Planner START!");
    binMap = imbinarize(occupancyMatrix(explo_map),0.5);
    se = strel('cube',4);
    se2 = strel('cube',8);
    binMap  = imerode(binMap ,se);
    binMap  = imdilate(binMap ,se2);   
    temp_map = occupancyMap(binMap , MapResolution); %
    temp_map.LocalOriginInWorld = explo_map.LocalOriginInWorld;
    %inflate(temp_map, robotSize);
    
    % PLANNER hybrid A*
    if plannerType == "HA* "
        vMap = validatorOccupancyMap;
        vMap.Map = temp_map;
        planner = plannerHybridAStar(vMap, 'MinTurningRadius', MinTurningRadius, 'MotionPrimitiveLength',MotionPrimitiveLength); % stworzenie obiektu planner
        
        try
            poses = plannerProcess(planner, start_Location, stop_Location);
        catch er
            switch er.identifier
                case 'nav:navalgs:astar:OccupiedLocation'
                    warning('Droga nie moze zostac wyznaczona! Powod : OccupiedLocation');
                    continue;
                case 'nav:navalgs:hybridastar:StartError'
                    warning('Droga nie moze zostac wyznaczona! (planer A* error)  : StartError');
                    continue;
                case 'nav:navalgs:hybridastar:GoalError'
                    warning('Droga nie moze zostac wyznaczona! (planer A* error)  : GoalError');
                    continue;
                otherwise
                    rethrow(er);
            end
        end
    % PLANNER RRT*    
    elseif plannerType == "RRT*" 
        plannerStatus = true;
        plannerFirstIt = true;
        while true
            ccConfig = inflationCollisionChecker(vehDim, 'InflationRadius', robotRadiusTemp, 'NumCircles',1);
            costmap = vehicleCostmap(temp_map,'CollisionChecker',ccConfig );

            planner = pathPlannerRRT(costmap, 'MaxIterations',maxIterations,'ConnectionDistance',maxConnectionDistance, ...
                                    'MinTurningRadius',minTurningRadius,'GoalTolerance', [0.2, 0.2, 360], 'ConnectionMethod', 'Dubins');

            if plannerFirstIt
                costmapOrg = copy(costmap);
%                 stop_Location = changePointToClosest(temp_map, costmap, stop_Location);
                plannerFirstIt = false;
                if isempty(stop_Location)
                    disp('Cel zostal yznaczony mocno poza mapa')
                    plannerStatus = false;
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
                warning('Planner return length=0 path!')
                if robotRadiusTemp <=0.06
                    plannerStatus= false;
                    break
                else
                    robotRadiusTemp = robotRadiusTemp - 0.02; %%%%%%%%%%%%%%%%%%
                end
                start_Location(3) = start_Location(3) +pi/2; 
                continue
            end
            
            if ~checkPathValidity(plannerPosesObj,costmap)
                disp('path NOT VALID!!!!!')
            end
            
            break
        end
        
        diffrentRadiusFlag = false;
        if robotRadiusTemp ~= robotRadiusOrg
             diffrentRadiusFlag = true; %flag to infrorm if the radius (margin) changed
        end
        robotRadiusTemp = robotRadiusOrg; 
        
        if ~plannerStatus 
            continue
        end
        
        clear('plannerStatus')
        clear('plannerFirstIt')
 
        lengths = 0 : 0.25 : plannerPosesObj.Length;
        [refPoses,refDirections]  = interpolate(plannerPosesObj,lengths);
        refPoses2  = interpolate(plannerPosesObj);
%         hold on
%         plot(planner)
%         legend('hide')

        poses = refPoses;
        poses = [poses(:,1:2) deg2rad(poses(:,3))];
        poses(end,3) = poses(end-1,3);
        %poses = [start_Location; poses];  
        
        disp(num2str(checkPathValidity(plannerPosesObj,costmap)));
        occupated = checkOccupied(costmapOrg, [poses(:,1:2) rad2deg(poses(:,3))] );
        if any(occupated)
%             hold on 
%             plot(poses(find(occupated==1), 1),poses(find(occupated==1), 2), 'og')
            if any(occupated(end-2:end))
                 warning(['last positions in blocked area!!!:',num2str(find(occupated==1)'), 'length:', num2str(length(occupated)) ]); 
                 occupatedIndexes = find(occupated==1);
                 poses = poses(1:occupatedIndexes(1)-1, :);
            else
                warning(['poses in occupated area!! Poses number:',num2str(find(occupated==1)') ]); 
            end
            
        end
     
    else
        error("Nieodpowiednia nazwa plannera- tylko RRT* lub A*");
    end
    
    
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
        
        % Zapisanie aktualnej pozycji robota
        all_poses(end+1,:) = poses(idx,:); % odczytanie ostatniej pozycji i dopisanie jej do tablicy wszystkich pozycji
        %middle_Pt(end+1,:) = middle_points(explo_map, angles, ranges, all_poses(end,:), middle_Pt);
        middle_Pt(end+1,:) = middle_points2(explo_map, all_poses(end,:), middle_Pt);

        
        % Aktualizacja tworzonej mapy 
        hold on
        show(explo_map, 'FastUpdate', true);
        
        % Aktualiacja znacznika robota
        hold on
        if exist('robot_plot', 'var')
            delete(robot_plot);
        end
        robot_plot = viz.plotRobot(gca, all_poses(end,:), 0.5);
     
        % Aktualizacja przejechanej sciezki (o ile potrzebna)
        if idx>1 && show_robot_path
            if goback_flag
                hold on
                if backPlotFirst
                    backPlotFirst = false;
                    plot(poses(idx-1 : idx ,1), poses(idx-1 :idx,2), '-b', 'DisplayName', 'Œcie¿ka powrotna');
                    legend()
                else
                    plot(poses(idx-1 : idx ,1), poses(idx-1 :idx,2), '-b','HandleVisibility','off'); 
                end
            else
                hold on
                if headPlotFirst
                    headPlotFirst = false;
                    plot(poses(idx-1 : idx ,1), poses(idx-1 :idx,2), '-r', 'DisplayName', 'Œcie¿ka');
                    legend()
                else
                    plot(poses(idx-1 : idx ,1), poses(idx-1 :idx,2), '-r','HandleVisibility','off' ); 
                end
            end
        end
        drawnow
      
        % Sprawdzenie czy na wyznaczonej trasie nie pojawi³a siê przeszkoda

        binMap = imbinarize(occupancyMatrix(explo_map),0.5);
        temp_map = occupancyMap(binMap , MapResolution); %
        temp_map.LocalOriginInWorld = explo_map.LocalOriginInWorld;
        costmap = vehicleCostmap(temp_map,'CollisionChecker',ccConfigOrg );
        
        idx = idx + 1;
        
        if diffrentRadiusFlag && checkFree(costmap, [all_poses(end,1:2) rad2deg(all_poses(end,3))])
            warning('Vehicle left occupied area!')

            diffrentRadiusFlag = false;
            break
        end
        
        if checkOccupied(costmap, [all_poses(end,1:2) rad2deg(all_poses(end,3))])
            disp("ROUTE OCCUPIED ")
        end
    end
    
    disp("Navigation to point... DONE!");
    startPoint =  all_poses(end,:); % dodanie jako kolejnej pozycji startowej ostatniej osi¹gniêtej pozycji - aktulanej pozycji robota
    
end
toc(simulation_time) % zatrzymanie timera odpowiadzalnego za pomiar czasu symulacji

%%
disp("MAPPING DONE");
figure
subplot(1,2,1)
show(hide_map);
title("Rzeczywista mapa")

subplot(1,2,2)
show(explo_map);
title("Ukonczona  mapa po symulacji")