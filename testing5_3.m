% V3 - dzia³aj¹ca, poprawiony powrót do rodzica

clc
close all
clear all

%%-------------  PARAMETRY POCZ¥TKOWE  -----------------------------------------


RealMap = 'mapa_paint5.png';     % Wczytywana rzeczywista mapa 
MapResolution = 20;              % Rozdzielczoœæ mapy - iloœæ pikseli przypadaj¹ca na metr  
startPoint = [1 0.5 pi/2];      % Punkt startowy robota oraz jego pocz¹tkowe po³o¿enie k¹towe [x y rad]

maxLidarRange = 8;               % [m]
AngleRangeBoundaries = [-pi pi]; % Maksymalny zakres katowy (dla innego zakresu ni¿ 360 stopni mo¿e nie funkcjonowaæ poprawnie)
RangeNoise = 0.001;              % Szum przy okreœlaniu zasiêgów

MaxNumOfRetry = 10;              % Maksymalna liczba prób wyznaczenia œcie¿ki dla danego punktu poczatkowego i koncowego w przypadku wystapienia bledu

% Wybor rodzaju plannera
plannerType = "RRT*"; %do wyboru 'A*'(HybridA*) lub RRT*(HybridRRT*)

% Parametry plannera A*
MinTurningRadius = 0.1;         % Minimalny promien zawracania
MotionPrimitiveLength = 0.1;    % Dlugosc "odcinkow" / "³uków" w grafie (?)
   % mo¿na dodac wiecej parametrow planera - te sa podstawowe
   
   %Parametry plannera RRT*
validationDistance = 0.2;
maxIterations = 2500;
maxConnectionDistance = 0.1;
goalRadius = 0.1;


% Wyniki
show_child_points = true;       % Aktualne przedstawienie wszystkich punktów rozgalezien
show_robot_path = true;         % Wyswietlanie przebytej sciezki przez robota
show_current_target = true;     % Wyswietlanie aktualego punktu uznanego jako cel do ktorego zostaje wyznaczona sciezka

%%----------------- INICJALIZACJA --------------------------------------------------------------
% Inicjalizacja ukrytej rzeczywistej mapy mapy

grayimage = rgb2gray(imread(RealMap));
bwimage = grayimage < 128;
hide_map = occupancyMap(bwimage, MapResolution);

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

viz = HelperUtils;          % zalaczenie narzedzi do wyœwietlania robota (byc moze do wyrzucenia - teraz korzysta tylko ze znacznika robota)

simulation_time = tic;      % pomiar czasu symulacji - zwracany na koniec wykonywania programu


%-------------------- GLÓWNA PÊTLA SYMULACJI---------------------------------------------------------------
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
                points_withrating = [exploratory_points_rating(sameparent_points(:,1:2), explo_map, startPoint, maxLidarRange) temp]; % macierz punktów w formacie [x y rate child_index]
                
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
            parentTochild_route(end,:) =[ 0 all_poses(end,1:2)];
        elseif newParent_flag
            parentTochild_route(end+1,:) =[ parentNum  all_poses(end,1:2)]; % dopisywany jest kolejny rodzic, ale tylko przy zwiêkszeniu identyfikatora rodzica
            newParent_flag = false;
        end
        
        % Wyznaczenie punktów eksploracyjnych dla pozycji od last_pose_num do konca pozycji
        explo_points = [];
        explo_points = exploratory_points2(explo_map, explo_points, last_pose_num, all_poses, middle_Pt, maxLidarRange );
        
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
    % Aktualizacja wyswietlania wynikow
    if ~isempty(child) && show_child_points
        if exist('child_plot', 'var')
            delete(child_plot);

        end
        hold on
        child_plot = plot(child(:,1),child(:,2), '.b');
    end
    if ~isempty(target_point) && show_current_target
        if exist('target_plot', 'var')
            delete(target_plot);
        end
        hold on
        target_plot = plot(target_point(1,1), target_point(1,2),'or');
    end
    
    % Przypisanie punktu pocz¹tkowego i koncowego wraz z wyznaczeniem k¹ta
    start_Location = startPoint;
    stop_Location = [target_point Angle2Points(startPoint(1,1:2), target_point(1,1:2) )];
    
    last_pose_num  = length(all_poses(:,1));
    
    % Inicjalizacja planera
    disp("Planner START!");
    temp_map = occupancyMap(imbinarize(occupancyMatrix(explo_map),0.51), MapResolution); % oszukanie zajetosci przez binaryzacjê aktualnej mapy
    temp_map.LocalOriginInWorld = explo_map.LocalOriginInWorld;
    inflate(temp_map, 0.01);
    
    % PLANNER A*
    if plannerType == "A*"
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
        ss = stateSpaceSE2;
        
        vMap = validatorOccupancyMap(ss);
        vMap.Map = temp_map;
        vMap.ValidationDistance = validationDistance;
        
        ss.StateBounds = [temp_map.XWorldLimits; temp_map.YWorldLimits; [-pi pi]];
        
        planner = plannerRRTStar(ss,vMap);
        planner.ContinueAfterGoalReached = true;
        planner.MaxIterations = maxIterations;
        planner.MaxConnectionDistance = maxConnectionDistance;
        
        try
        [plannerPosesObj, ~] = plan(planner,start_Location,stop_Location);
        poses = plannerPosesObj.States;
        catch er
            warning(["RRT* nie moze wyznaczyc trasy, powod : ", er.identifier]);
            continue;
            
            %dodac obsluge bledu jesli bedzie wystepowac
            
        end
    else
        error("Nieodpowiednia nazwa plannera- tylko RRT* lub A*");
    end
    
    
    % Pocz¹tek przemieszczenia pojazdu do zadanego punktu
    disp("Navigation to point...");
    idx =1;
    tic
    while idx <= size(poses,1) && RetryCounter <= MaxNumOfRetry
        ranges = [];
        angles = [];
        
        % Kolejne pomiary dla kolejnych pozycji i do³¹czanie ich do mapy
        [ranges, angles] = rangefinder(poses(idx,:), hide_map);
        insertRay(explo_map, poses(idx,:), ranges, angles, rangefinder.Range(end));
        
        % Zapisanie aktualnej pozycji robota
        all_poses(end+1,:) = poses(idx,:); % odczytanie ostatniej pozycji i dopisanie jej do tablicy wszystkich pozycji
        middle_Pt(end+1,:) = middle_points(explo_map, angles, ranges, all_poses(end,:));
        
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
                plot(poses(idx-1 : idx ,1), poses(idx-1 :idx,2), '-b');
            else
                hold on
                plot(poses(idx-1 : idx ,1), poses(idx-1 :idx,2), '-r');
            end
        end
        drawnow
        
        % Sprawdzenie czy na wyznaczonej trasie nie pojawi³a siê przeszkoda
        isRouteOccupied = any(checkOccupancy(explo_map, poses(:,1:2)));
        if isRouteOccupied && (toc > 0.5)
            
            % Stworzenie tymaczowej mapy dla planera przez binaryzacjê aktualnej - oszukanie zajêtosci obszaru
            temp_map = occupancyMap(imbinarize(occupancyMatrix(explo_map),0.51), MapResolution);
            inflate(temp_map, 0.01);
            planner.StateValidator.Map = temp_map;
            
            % Replanowanie œciezki wraz obs³uga b³êdów
            % Stworzenie tymaczowej mapy dla planera przez binaryzacjê aktualnej - oszukanie zajêtosci obszaru
            temp_map = occupancyMap(imbinarize(occupancyMatrix(explo_map),0.51), MapResolution);
            temp_map.LocalOriginInWorld = explo_map.LocalOriginInWorld;
            inflate(temp_map, 0.01);
            planner.StateValidator.Map = temp_map;
            
            % Replanowanie œciezki wraz obs³uga b³êdów
            
            if plannerType == "A*"
                try
                    poses = plannerProcess(planner, poses(idx,:), stop_Location);
                catch er
                    switch er.identifier
                        case'nav:navalgs:astar:OccupiedLocation'
                            warning('Droga nie moze zostac wyznaczona! OccupiedLocation');
                            RetryCounter = RetryCounter +1 ;
                            continue;
                        case 'nav:navalgs:hybridastar:StartError'
                            warning('Droga nie moze zostac wyznaczona! StartError');
                            RetryCounter = RetryCounter +1 ;
                            continue;
                        otherwise
                            rethrow(er);
                    end
                end
                
            elseif  plannerType == "RRT*"
                try
                    [plannerPosesObj, ~] = plan(planner,start_Location,stop_Location);
                    poses = plannerPosesObj.States;
                catch er
                    warning(["RRT* nie moze wyznaczyc trasy, powod : ", er.identifier]);
                    continue;
                    
                    %dodac obsluge bledu jesli bedzie wystepowac
                end
            end
            idx = 1;
            tic;
        else
            idx = idx + 1;
        end
    end
    
    RetryCounter = 0; 
    
    disp("Navigation to point... DONE!");
    
    startPoint =  poses(end,:); % dodanie jako kolejnej pozycji startowej ostatniej osi¹gniêtej pozycji - aktulanej pozycji robota
    
end
toc(simulation_time) % zatrzymanie timera odpowiadzalnego za pomiar czasu symulacji

show(explo_map);
%%
disp("MAPPING DONE");
figure
subplot(1,2,1)
show(hide_map);
title("Rzeczywista mapa")

subplot(1,2,2)
show(explo_map);
title("Ukonczona  mapa po symulacji")