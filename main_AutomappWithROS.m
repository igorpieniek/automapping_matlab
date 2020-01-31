clc
close all
clear all

%%-------------  PARAMETRY POCZ�TKOWE  -----------------------------------------

maxLidarRange = 8;               % [m]
MapResolution = 50;
MaxNumOfRetry = 10;              % Maksymalna liczba pr�b wyznaczenia �cie�ki dla danego punktu poczatkowego i koncowego w przypadku wystapienia bledu

% Parametry plannera A*
MinTurningRadius = 0.1;         % Minimalny promien zawracania
MotionPrimitiveLength = 0.1;    % Dlugosc "odcinkow" / "�uk�w" w grafie (?)
   % mo�na dodac wiecej parametrow planera - te sa podstawowe

% ROS Node init
node_automap = ros.Node('/matlab_automap');
pub_automap = ros.Publisher(node_automap, '/matlab_velocity', 'std_msgs/Float32MultiArray');
rosmsg = rosmessage('std_msgs/Float32MultiArray');
rosmsg.Data = [0 0];
send(pub_automap, rosmsg);
%%----------------- INICJALIZACJA --------------------------------------------------------------

% Inicjacja nowej mapy
explo_map = Lidar_Init();


% Pierwszy pomiar w punkcie startowym
Lidar_subscriber = rossubscriber('/scan');
explo_map = LidarAq(explo_map, Lidar_subscriber);
[explo_map_occ, realPoses] = buildMap_and_poses(explo_map, MapResolution, maxLidarRange);



last_pose_num  = 1; % nr ostatniej pozycji przy kt�rej osiagnieto punkt eksploracyjny
explo_points=[];    % tablica na punkty eksploracyjne


% Inicjalizacja zmiennych potrzebnych w g��wnej p�tli 
parentNum = 0;              % identyfikator rodzica danej galezi 
newParent_flag = false;     % flaga podnoszona przy odnalezieniu rozgalezienia
goback_flag = false;        % flaga podnoszona przy braku nowych punktow dla danej galezi - prowadzi do powrotu do punktu rozgalezienia
child = [];                 % macierz na punkty rozgalezien dla danego identyfikatora rozgalezienia (rodzica)
middle_Pt = [];             % macierz na "punkty srodkowe" - do okreslenia obszarow zablokowanych dla wyszukiwania punktow eksploracyjnych (sposob filtracji)
parentTochild_route = [];   % macierz na zapisywanie punkt�w po ktorych robot moze wrocic do rozgalezienia z ktorego wychodzi galaz na ktorej sie aktualnie znajduje
parentTochild_route = [0 realPoses(end,1:2)]; % dodanie pierwszego punktu powrotnego
RetryCounter = 0;           % licznik powt�zen w przypadku bledu wyznaczania trasy


simulation_time = tic;      % pomiar czasu symulacji - zwracany na koniec wykonywania programu

% PLANNER RRT* INIT (TEST PLANNERA)


%-------------------- GL�WNA P�TLA ---------------------------------------------------------------

while true
    %%--------------- Cz�� algortytmu odpowiadaj�ca za rozgalezienia -------------
    
    if goback_flag % powrot do rodzica
        
        % Usuniecie punktow - rodzicow b�d�cych punktami powrotnymi jezeli nie posiadaja dzieci - innych galezi
        to_delete = [];
        for p = 1 :  length(parentTochild_route(:,1))
            semeparent_number = find(child(:,3) == parentTochild_route(p,1)); % zebranie galezi o tym samym identyfikatorze rodzica
            if isempty(semeparent_number)
                to_delete(1,end+1) = p;                                       % w przypadku braku galezi o danym identyfikatorze, zostaje zapisany nr identyfikatora
            end
        end
        to_delete = unique(to_delete);
        parentTochild_route(to_delete,:) = [];                                % usuni�cie punktow o zapisanym identyfikatorze

        
        if  parentTochild_route(end,1) == parentNum && length(parentTochild_route(:,1))>1 %jezeli operujemy caly czas na tym samym identyfikatorze rodzica
            target_point = parentTochild_route(end,2:3); % wyznaczenie aktualnego celu jako ostatneigu punktu z listy punktow powrotnych
            parentTochild_route(end, :) = [];
        else
            temp = find(child(:,3) == parentNum); % zebranie punktow galezi (dzieci) o tym samym identyfikatorze rodzica dla aktualnego identyfikatora
            if ~isempty(temp)
                
                sameparent_points = child(temp,:);                                                                     % tworzy tymaczasow� macierz dzieci posiadaj�cych tych samych rodzic�w                
                points_withrating = [exploratory_points_rating(sameparent_points(:,1:2), explo_map_occ, realPoses(end, :), maxLidarRange) temp]; % macierz punkt�w w formacie [x y rate child_index]
                
                [target_point, ~, target_num] = best_point(points_withrating(:,1:2), points_withrating(:,3));          % wyznaczenie punktu target
                child(points_withrating(target_num, 4), :) = [];                                                       % usuniecie z listy dzieci punktu target
                
                goback_flag = false;                                                                                   % powrot do punktu - rodzica zostal zakonczony
            else
                parentNum = parentNum -1;                                                                              % jezeli nie ma galezi (dzieci) dla danego identyfikatora rodzica
                continue;
            end
        end
        
    else % flaga o powrocie do punktu rozgalezienia nie zostala podniesiona
        if parentNum == 0                                                   % na pocz�tku gdy nie ma galezi nadpisywana jest pierwsza linijka
            parentTochild_route(end,:) =[ 0 realPoses(end,1:2)];
        elseif newParent_flag
            parentTochild_route(end+1,:) =[ parentNum  realPoses(end,1:2)]; % dopisywany jest kolejny rodzic, ale tylko przy zwi�kszeniu identyfikatora rodzica
            newParent_flag = false;
        end
        
        % Wyznaczenie punkt�w eksploracyjnych dla pozycji od last_pose_num do konca pozycji
        explo_points = [];
        disp("Exploratory points search START!");
        explo_points = exploratory_points2(explo_map_occ, explo_points, last_pose_num, realPoses, middle_Pt, maxLidarRange );
        disp("Exploratory points search DONE!");
        % weryfikacja dzieci wzgl�dem osiagnietych pozycji
        if ~isempty(child) && ~isempty(middle_Pt)
            child = verify_PointsToPosses(child, middle_Pt);
        end
        
        % weryfikacja punkt�w wzgledem osiagnietych pozycji
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
    % Inicjalizacja planera
    disp("Planner START!");
    ss = stateSpaceSE2;
    vMap = validatorOccupancyMap(ss);

    
    temp_map = occupancyMap(imbinarize(occupancyMatrix(explo_map_occ),0.51), MapResolution); % oszukanie zajetosci przez binaryzacj� aktualnej mapy
    temp_map.LocalOriginInWorld = explo_map_occ.LocalOriginInWorld;
    inflate(temp_map, 0.01);                                                             % powi�kszenie zaj�tych obszarow
    vMap.Map = temp_map;
    vMap.ValidationDistance = 0.01;
 
    ss.StateBounds = [temp_map.XWorldLimits; temp_map.YWorldLimits; [-pi pi]];
    %     planner = plannerHybridAStar(vMap, 'MinTurningRadius', MinTurningRadius, 'MotionPrimitiveLength',MotionPrimitiveLength); % stworzenie obiektu planner
    planner = plannerRRTStar(ss,vMap);
    planner.ContinueAfterGoalReached = true;
    
    planner.MaxIterations = 2500;
%     planner.MaxConnectionDistance = 0.1;
    
    % Przypisanie punktu pocz�tkowego i koncowego wraz z wyznaczeniem k�ta
    start_Location = realPoses(end,:);
    stop_Location = [target_point Angle2Points(realPoses(end,1:2), target_point(1,1:2) )];
    
    last_pose_num  = length(realPoses(:,1));
    
    % Proba wyznaczenia trasy i obsulga b��d�w
    try
%         plannerPoses = plannerProcess(planner, start_Location, stop_Location);
        [plannerPoses, ~] = plan(planner,start_Location,stop_Location);
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
    
    disp("Planner DONE!");
    
    
    % Pocz�tek przemieszczenia pojazdu do zadanego punktu
    disp("Navigation to point...");
    
    goalRadius = 0.2;
    distanceToGoal = norm(realPoses(end,1:2) - plannerPoses.States(end, 1:2));
    controller = controllerPurePursuit("Waypoints",plannerPoses.States(:,1:2),"DesiredLinearVelocity",0.5,"MaxAngularVelocity", pi/4, "LookaheadDistance", 0.3);
    while( distanceToGoal > goalRadius )
        
        %sprawdzenie �cie�ki, czy zosta�a poprawnie 
        navTimeProcess = tic;
        isRouteOccupied = any(checkOccupancy(explo_map_occ, plannerPoses.States(:,1:2)));
        if isRouteOccupied 
            %   MOTOR STOP
            rosmsg.Data = [0 0];
            send(pub_automap, rosmsg);
            
            % Stworzenie tymaczowej mapy dla planera przez binaryzacj� aktualnej - oszukanie zaj�tosci obszaru
            temp_map = occupancyMap(imbinarize(occupancyMatrix(explo_map_occ),0.51), MapResolution);
            temp_map.LocalOriginInWorld = explo_map_occ.LocalOriginInWorld;
            inflate(temp_map, 0.01);
            planner.StateValidator.Map = temp_map;
            
            % Replanowanie �ciezki wraz obs�uga b��d�w 
            try
%                 plannerPoses = plannerProcess(planner, plannerPoses(idx,:), stop_Location);
                 [plannerPoses, ~] = plan(planner,start_Location,stop_Location);
            catch er
                switch er.identifier
                    case'nav:navalgs:astar:OccupiedLocation'
                        warning('Droga nie moze zostac wyznaczona! Proces zostanie przerwany');
                        RetryCounter = RetryCounter +1 ;
                        continue;
                    case 'nav:navalgs:hybridastar:StartError'
                        warning('Droga nie moze zostac wyznaczona! (planer A* error) Proces zostanie przerwany');
                        RetryCounter = RetryCounter +1 ;
                        continue;
                    otherwise
                        rethrow(er);
                end
            end
        end
        % Akwizycja danych z lidaru i przypisanie do mapy oraz aktualizacja pozycji
        explo_map = LidarAq(explo_map, Lidar_subscriber);
        [explo_map_occ, realPoses] = buildMap_and_poses(explo_map, MapResolution, maxLidarRange);
        
        % Wyznaczenie pr�dkosci
        [v, omega] = controller(realPoses(end,:));
        
        % WYSLIJ AKTUALNE PREDKOSCI
        rosmsg.Data = [v omega];
        send(pub_automap, rosmsg);
        
        
        % Aktualizacja odleg�o�ci od ko�ca wyznaczonej �cie�ki
        distanceToGoal = norm(realPoses(end,1:2) - plannerPoses.States(end, 1:2));
        
        % Wyznaczenie okregow filtrujacych
        middle_Pt(end+1,:) = middle_points2(explo_map_occ,realPoses(end,:));
         
        RetryCounter = 0; 
    
        
    end
    disp("Navigation to point... DONE!");
    
    startPoint =  plannerPoses.States(end,:); % dodanie jako kolejnej pozycji startowej ostatniej osi�gni�tej pozycji - aktulanej pozycji robota
    
end
toc(simulation_time) % zatrzymanie timera odpowiadzalnego za pomiar czasu symulacji

show(explo_map_occ);
%%
disp("MAPPING DONE");
figure
show(explo_map_occ);
title("Ukonczona  mapa po symulacji")