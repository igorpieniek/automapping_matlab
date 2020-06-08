clc
close all
clear 

%%-------------  PARAMETRY POCZ�TKOWE  -----------------------------------------

RealMap = 'C:\Users\Igor\Dysk Google\Studia\IN�YNIERKA\maps\mapa_paint5.png';     % Wczytywana rzeczywista mapa 
MapResolution = 20;              % Rozdzielczo�� mapy - ilo�� pikseli przypadaj�ca na metr  
startPoint = [0.7 1 pi/2];      % Punkt startowy robota oraz jego pocz�tkowe po�o�enie k�towe [x y rad]

maxLidarRange = 12;               % [m]
AngleRangeBoundaries = [-pi pi]; % Maksymalny zakres katowy (dla innego zakresu ni� 360 stopni mo�e nie funkcjonowa� poprawnie)
RangeNoise = 0.001;              % Szum przy okre�laniu zasi�g�w


%Parametry plannera RRT*
robotRadiusConfig.original = 0.25;
robotRadiusConfig.step = 0.02;
robotRadiusConfig.min = 0.06;
plannerConfig.maxIterations = 10000;
plannerConfig.ConnectionDistance = 0.5;
plannerConfig.minTurningRadius = 0.001;
plannerConfig.goalTolerance = [0.2, 0.2, 360];
pathPointsDistance = 0.4;

% wymiary pojazdu
vehDim = vehicleDimensions(0.38, 0.25, 0.2,'FrontOverhang',0.04,'RearOverhang',0.3, 'Wheelbase', 0.005);
ccConfigOrg = inflationCollisionChecker(vehDim, 'InflationRadius', robotRadiusConfig.original, 'NumCircles',1);

% Wyniki
show_childPoints = true;       % Aktualne przedstawienie wszystkich punkt�w rozgalezien
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

% Inicjalizacja wirtualnego lidaru oraz jego parametr�w
rangefinder = rangeSensor('HorizontalAngle', AngleRangeBoundaries,'RangeNoise', RangeNoise,'Range' , [0 maxLidarRange]);
numReadings = rangefinder.NumReadings;

lastPoseNum  = 1; % nr ostatniej pozycji przy kt�rej osiagnieto punkt eksploracyjny
allPoses = [];     % tablica wszystkich kolejno osi�ganych pozycji
allPoses( end + 1,:) = startPoint; % dodanie pierwszego punktu

% Pierwszy pomiar w punkcie startowym
[ranges, angles] = rangefinder(startPoint, hideMap);
insertRay(exploMap,  startPoint, ranges, angles,  rangefinder.Range(end));


middlePoints = [];             % macierz na "punkty srodkowe" - do okreslenia obszarow zablokowanych dla wyszukiwania punktow eksploracyjnych (sposob filtracji)


exploratoryInflateRatio = 0.05; % wsp�czynnik funkcji inflate potrzebny przy przetwarzaniu aktualnej mapy w g��wnej funkcji wyszukuj�cej obszary
                                % do eksploracji - exploratory_points2

DFS = DFSalgorithm(startPoint(1,1:2), maxLidarRange, exploratoryInflateRatio);         % za��czenie funkcji algorytmu DFS zbudowanego na potrzeby skryptu
viz = HelperUtils;          % zalaczenie narzedzi do wy�wietlania robota (byc moze do wyrzucenia - teraz korzysta tylko ze znacznika robota)

% Obej�cie legedy na wykresie - dzi�ki temu przy wy�wietlaniu legendy informacja pojawia sie tylko raz
backPlotFirst = true;
headPlotFirst = true;
targetPlotFirst = true;

% Rozpocz�cie pomiaru czasu wykonywania
simulation_time = tic;      
%-------------------- GL�WNA P�TLA SYMULACJI---------------------------------------------------------------
figure  
while true
    %%--------------- Cz�� algortytmu odpowiadaj�ca za rozgalezienia - algorytm DFS -------------
    
     DFSoutput = DFS.process(exploMap, allPoses(lastPoseNum:end, :), middlePoints);
     if isempty(DFSoutput.target)
         break;
     end
     exploPoints = DFSoutput.exploPoints;
     target_point = DFSoutput.target;
     gobackFlag = DFSoutput.goBackFlag;
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
    
    % Okre�lenie aktualnego celu oraz punktu startowego
    start_Location = startPoint;
    stop_Location = [target_point Angle2Points(startPoint(1,1:2), target_point(1,1:2) )];
    
    % Zapisanie numeru ostatniej zapsianej pozycji
    lastPoseNum  = length(allPoses(:,1));
    
    % Konwersja aktualnej mapy
    temp_map = mapConversion(exploMap);
    

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

    % Pocz�tek przemieszczenia pojazdu do zadanego punktu
    disp("Navigation to point...");
    idx =1;
    while idx <= size(path,1)
        ranges = [];
        angles = [];
        
        % Kolejne pomiary dla kolejnych pozycji i do��czanie ich do mapy
        [ranges, angles] = rangefinder(path(idx,:), hideMap);
        insertRay(exploMap, path(idx,:), ranges, angles, rangefinder.Range(end));
        
        % Zapisanie aktualnej pozycji robota
        allPoses(end+1,:) = path(idx,:); % odczytanie ostatniej pozycji i dopisanie jej do tablicy wszystkich pozycji
        
        % Dodanie kolejnych okr�g�w filtruj�cych
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
                    plot(path(idx-1 : idx ,1), path(idx-1 :idx,2), '-b', 'DisplayName', '�cie�ka powrotna');
                    legend()
                else
                    plot(path(idx-1 : idx ,1), path(idx-1 :idx,2), '-b','HandleVisibility','off'); 
                end
            else
                hold on
                if headPlotFirst
                    headPlotFirst = false;
                    plot(path(idx-1 : idx ,1), path(idx-1 :idx,2), '-r', 'DisplayName', '�cie�ka');
                    legend()
                else
                    plot(path(idx-1 : idx ,1), path(idx-1 :idx,2), '-r','HandleVisibility','off' ); 
                end
            end
        end
        drawnow
        
        idx = idx + 1;
        
        % Konwersja aktualnej wersji mapy - do weryfikacji
        temp_map = mapConversion(exploMap);
        
        % Uworzenie aktualnej mapy koszt�w - wzgl�dem orginalnej wielkosci marginesu
        costmap = vehicleCostmap(temp_map,'CollisionChecker',ccConfigOrg );    
        
        % Sprawdzenie czy na wyznaczonej trasie nie pojawi�a si� przeszkoda
        if differentRadiusFlag && checkFree(costmap, [allPoses(end,1:2) rad2deg(allPoses(end,3))])
            warning('Vehicle left occupied area!')
            differentRadiusFlag = false;
            break
        end
        
        % Informacja o przebiegu �cie�ki przez oszar zaj�ty wed�ug aktualnej mapy koszt�w
        if checkOccupied(costmap, [allPoses(end,1:2) rad2deg(allPoses(end,3))])
            disp("ROUTE OCCUPIED ")
        end
    end
    
    disp("Navigation to point... DONE!");
    startPoint =  allPoses(end,:); % dodanie jako kolejnej pozycji startowej ostatniej osi�gni�tej pozycji - aktulanej pozycji robota
    
end
disp("MAPPING DONE");
toc(simulation_time) % zatrzymanie timera i wy�wietlenie czasu symulacji

%% Wy�wietlenie wynik�w

figure
subplot(1,2,1)
show(hideMap);
title("Rzeczywista mapa")
subplot(1,2,2)
show(exploMap);
title("Ukonczona  mapa po symulacji")