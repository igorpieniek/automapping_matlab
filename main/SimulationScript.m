clc
close all
clear 

%%-------------  PARAMETRY POCZ¥TKOWE  -----------------------------------------

RealMap = 'C:\Users\Igor\Dysk Google\Studia\IN¯YNIERKA\maps\sym_part1.png';     % Wczytywana rzeczywista mapa 
MapResolution = 34;              % Rozdzielczoœæ mapy - iloœæ pikseli przypadaj¹ca na metr  
startPoint = [1.3 2.5 pi/2];      % Punkt startowy robota oraz jego pocz¹tkowe po³o¿enie k¹towe [x y rad]

maxLidarRange = 12;               % [m]
AngleRangeBoundaries = [-pi pi]; % Maksymalny zakres katowy (dla innego zakresu ni¿ 360 stopni mo¿e nie funkcjonowaæ poprawnie)
RangeNoise = 0.001;              % Szum przy okreœlaniu zasiêgów


%Parametry plannera RRT*
robotRadiusConfig.original = 0.3;
robotRadiusConfig.step = 0.02;
robotRadiusConfig.min = 0.06;
plannerConfig.maxIterations = 10000;
plannerConfig.ConnectionDistance = 0.5;
plannerConfig.minTurningRadius = 0.001;
plannerConfig.goalTolerance = [0.2, 0.2, 360];
pathPointsDistance = 0.25;

% wymiary pojazdu
vehDim = vehicleDimensions(0.38, 0.25, 0.2,'FrontOverhang',0.04,'RearOverhang',0.3, 'Wheelbase', 0.005);
ccConfigOrg = inflationCollisionChecker(vehDim, 'InflationRadius', robotRadiusConfig.original, 'NumCircles',1);

% Wyniki
show_childPoints = true;       % Aktualne przedstawienie wszystkich punktów rozgalezien
show_robotPath = true;         % Wyswietlanie przebytej sciezki przez robota
show_target = false;     % Wyswietlanie aktualego punktu uznanego jako cel do ktorego zostaje wyznaczona sciezka

%%----------------- INICJALIZACJA --------------------------------------------------------------
% Inicjalizacja ukrytej rzeczywistej mapy mapy
grayimage = rgb2gray(imread(RealMap));
bwimage = grayimage < 128;
hideMap = occupancyMap(bwimage, MapResolution);
show(hideMap)

% Inicjacja nowej mapy
exploMap = occupancyMap( (ones(size(grayimage))).*0.5, MapResolution);

% Inicjalizacja wirtualnego lidaru oraz jego parametrów
rangefinder = rangeSensor('HorizontalAngle', AngleRangeBoundaries,'RangeNoise', RangeNoise,'Range' , [0 maxLidarRange]);
numReadings = rangefinder.NumReadings;

lastPoseNum  = 1; % nr ostatniej pozycji przy której osiagnieto punkt eksploracyjny
allPoses = [];     % tablica wszystkich kolejno osi¹ganych pozycji
allPoses( end + 1,:) = startPoint; % dodanie pierwszego punktu

% Pierwszy pomiar w punkcie startowym
[ranges, angles] = rangefinder(startPoint, hideMap);
insertRay(exploMap,  startPoint, ranges, angles,  rangefinder.Range(end));


% Inicjalizacja zmiennych potrzebnych w g³ównej pêtli 
parentNum = 0;             % identyfikator rodzica danej galezi 
newParentFlag = false;     % flaga podnoszona przy odnalezieniu rozgalezienia
gobackFlag = false;        % flaga podnoszona przy braku nowych punktow dla danej galezi - prowadzi do powrotu do punktu rozgalezienia
exploPoints = [];                 % macierz na punkty rozgalezien dla danego identyfikatora rozgalezienia (rodzica)
middlePoints = [];             % macierz na "punkty srodkowe" - do okreslenia obszarow zablokowanych dla wyszukiwania punktow eksploracyjnych (sposob filtracji)
parentTochildRoute = [];   % macierz na zapisywanie punktów po ktorych robot moze wrocic do rozgalezienia z ktorego wychodzi galaz na ktorej sie aktualnie znajduje
parentTochildRoute = [0 startPoint(1,1:2)]; % dodanie pierwszego punktu powrotnego

exploratoryInflateRatio = 0.05; % wspó³czynnik funkcji inflate potrzebny przy przetwarzaniu aktualnej mapy w g³ównej funkcji wyszukuj¹cej obszary
                                % do eksploracji - exploratory_points2

DFS = DFSalgorithm;         % za³¹czenie funkcji algorytmu DFS zbudowanego na potrzeby skryptu
viz = HelperUtils;          % zalaczenie narzedzi do wyœwietlania robota (byc moze do wyrzucenia - teraz korzysta tylko ze znacznika robota)

% Obejœcie legedy na wykresie - dziêki temu przy wyœwietlaniu legendy informacja pojawia sie tylko raz
backPlotFirst = true;
headPlotFirst = true;
targetPlotFirst = true;

% Rozpoczêcie pomiaru czasu wykonywania
simulation_time = tic;      
%-------------------- GLÓWNA PÊTLA SYMULACJI---------------------------------------------------------------
figure  
while true
    %%--------------- Czêœæ algortytmu odpowiadaj¹ca za rozgalezienia - algorytm DFS -------------
    
    if gobackFlag % flaga o powrocie do punktu rozgalezienia zostala podniesiona
        [parentTochildRoute,...
            exploPoints,...
            parentNum,...
            target_point,...
            gobackFlag,...
            continueStatus ] = DFS.goBack(parentTochildRoute,...
                                          exploPoints,...
                                          parentNum,...
                                          exploMap,...
                                          allPoses,...
                                          maxLidarRange );
        if continueStatus
            continue
        end

    else 
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
                                        allPoses,...
                                        exploMap,...
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
    % Aktualizacja wyswietlania wynikow
    if ~isempty(exploPoints) && show_childPoints
        if exist('child_plot', 'var')
            delete(child_plot);
        end
        hold on
        child_plot = plot(exploPoints(:,1),exploPoints(:,2), '.b','DisplayName','Punkty eksploracyjne');
        legend()
        
    end
    if ~isempty(target_point) && show_target
        if exist('target_plot', 'var')
            delete(target_plot);
        end
        hold on
        target_plot = plot(target_point(1,1), target_point(1,2),'or','HandleVisibility','off');
        if targetPlotFirst
            targetPlotFirst = false;
            plot(target_point(1,1), target_point(1,2),'or','DisplayName','Aktualny cel')
            legend()
        end
    end
    
    % Przypisanie punktu pocz¹tkowego i koncowego wraz z wyznaczeniem k¹ta
    start_Location = startPoint;
    stop_Location = [target_point Angle2Points(startPoint(1,1:2), target_point(1,1:2) )];
    
    % Zapisanie numeru ostatniej zapsianej pozycji
    lastPoseNum  = length(allPoses(:,1));
    
    % Konwersja aktualnej mapy
    temp_map = mapConversion(exploMap, MapResolution);
    

    % PLANNER RRT*
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

    % Pocz¹tek przemieszczenia pojazdu do zadanego punktu
    disp("Navigation to point...");
    idx =1;
    while idx <= size(path,1)
        ranges = [];
        angles = [];
        
        % Kolejne pomiary dla kolejnych pozycji i do³¹czanie ich do mapy
        [ranges, angles] = rangefinder(path(idx,:), hideMap);
        insertRay(exploMap, path(idx,:), ranges, angles, rangefinder.Range(end));
        
        % Zapisanie aktualnej pozycji robota
        allPoses(end+1,:) = path(idx,:); % odczytanie ostatniej pozycji i dopisanie jej do tablicy wszystkich pozycji
        
        % Dodanie kolejnych okrêgów filtruj¹cych
        middlePoints(end+1,:) = middle_points2(exploMap, allPoses(end,:), middlePoints);

        
        % Aktualizacja tworzonej mapy 
        hold on
        show(exploMap, 'FastUpdate', true);
        
        % Aktualiacja znacznika robota
        hold on
        if exist('robot_plot', 'var')
            delete(robot_plot);
        end
        robot_plot = viz.plotRobot(gca, allPoses(end,:), 0.5);
     
        % Aktualizacja przejechanej sciezki (o ile potrzebna)
        if idx > 1 && show_robotPath
            if gobackFlag
                hold on
                if backPlotFirst
                    backPlotFirst = false;
                    plot(path(idx-1 : idx ,1), path(idx-1 :idx,2), '-b', 'DisplayName', 'Œcie¿ka powrotna');
                    legend()
                else
                    plot(path(idx-1 : idx ,1), path(idx-1 :idx,2), '-b','HandleVisibility','off'); 
                end
            else
                hold on
                if headPlotFirst
                    headPlotFirst = false;
                    plot(path(idx-1 : idx ,1), path(idx-1 :idx,2), '-r', 'DisplayName', 'Œcie¿ka');
                    legend()
                else
                    plot(path(idx-1 : idx ,1), path(idx-1 :idx,2), '-r','HandleVisibility','off' ); 
                end
            end
        end
        drawnow
        
        idx = idx + 1;
        
        % Konwersja aktualnej wersji mapy - do weryfikacji
        temp_map = mapConversion(exploMap,MapResolution);
        
        % Uworzenie aktualnej mapy kosztów - wzglêdem orginalnej wielkosci marginesu
        costmap = vehicleCostmap(temp_map,'CollisionChecker',ccConfigOrg );    
        
        % Sprawdzenie czy na wyznaczonej trasie nie pojawi³a siê przeszkoda
        if differentRadiusFlag && checkFree(costmap, [allPoses(end,1:2) rad2deg(allPoses(end,3))])
            warning('Vehicle left occupied area!')
            differentRadiusFlag = false;
            break
        end
        
        % Informacja o przebiegu œcie¿ki przez oszar zajêty wed³ug aktualnej mapy kosztów
        if checkOccupied(costmap, [allPoses(end,1:2) rad2deg(allPoses(end,3))])
            disp("ROUTE OCCUPIED ")
        end
    end
    
    disp("Navigation to point... DONE!");
    startPoint =  allPoses(end,:); % dodanie jako kolejnej pozycji startowej ostatniej osi¹gniêtej pozycji - aktulanej pozycji robota
    
end
disp("MAPPING DONE");
toc(simulation_time) % zatrzymanie timera i wyœwietlenie czasu symulacji

%% Wyœwietlenie wyników

figure
subplot(1,2,1)
show(hideMap);
title("Rzeczywista mapa")
subplot(1,2,2)
show(exploMap);
title("Ukonczona  mapa po symulacji")