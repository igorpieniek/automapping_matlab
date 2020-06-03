function [path,...
          differentRadiusFlag] = configuredRRTplanner(start,...
                                                      stop,...
                                                      vehicleDim,...
                                                      robotRadiusConfig,...
                                                      map,...
                                                      plannerConfig,...
                                                      pathPointsDistance)
% Skonfigurowana funkcja planująca ściezke przy wykorzystaniu plannera
% RRT* powstała na potrzeby algorytmu autonomicznego mapowania. Wewnątrz 
% funkcji znajduje się zoorganizowana obsługa błędów plannera. W przypadku
% wystapienia błędu plannera, gdy ściezka nie jest możliwa do
% wyznacznia, zostaje obniżony margines błędu, aż do wartosci minimalnej
% wg struktury robotRadiusConfig. W przypadku kiedy dla minimum nie
% zostanie odnaleziona sciezka, funkcja kończy swoją prace i zwraca pustą tablice
% ścieżki, dzieki której może poinformowac o błędzie lub o potrzebie
% przerwania aktualnej iteracji pętli głównej algorytmu autonomicznego mapowania
% INPUT:
% - start - pozycja startowa [x, y, fi] fi w radianach
% - stop - pozycja startowa [x, y, fi] fi w radianach
% - vehicleDim - obiekt klasy vehicleDimensions
% - robotRadiusConfig - struktura zawierająca takie pola jak : 'original',
% 'step' oraz 'min' - świadcząca o orginalnym marginesie robota, kroku o
% który ma zostać obnizany w przypadku niepowodzenia plannera, oraz wartośc
% minimalna, przy której nieznalezienie trasy zakonczy działanie funkcji
% zwracając true przy continueFlag. Więcej o tym marginesie w pomocy klasy inflationCollisionChecker 
% - map - obiekt occupancyMap - najlepiej przekonwertowana przez funkcje
% mapConversion
% - plannerConfig - struktura zawierająca ustawienia plannera RRT:
%       * maxIteration
%       * ConnectionDistance
%       * minTurningRadius
%       * goalTolerance
%       więcej ustawień dostępne w pomocy klasy pathplannerRRT. W celu zwiększenia ilosci 
%       potrzebnych ustawień nalezy modyfikowac te funkcje
% - pathPointsDistance - odległości między punktami na ścieżce
% 
% OUTPUT:
% - path - ścieżka w formacie macierzy [x, y, fi] (fi w radianach)
% - differentRadiusFlag - flaga wystawiana w przypadku, gdy ścieżka
%   została wyznaczona po obniżeniu marginesu (robotRadiusConfig)

%% INIT
    plannerStatus = true;
    plannerFirstIt = true;
    differentRadiusFlag = false;
    
    path = [];
    robotRadiusTemp = robotRadiusConfig.original;
%%    % Główna pętla plannera 
%      dopóki nie zostanie wyznaczona ściezka, lub  margines nie zostanie zminiejszony do minimum 
    while true
        ccConfig = inflationCollisionChecker(vehicleDim,...
                                             'InflationRadius', robotRadiusTemp,...
                                             'NumCircles',1);
                                         
        costmap = vehicleCostmap(map,'CollisionChecker',ccConfig );

        planner = pathPlannerRRT(costmap,...
                                 'MaxIterations',plannerConfig.maxIterations,...
                                 'ConnectionDistance',plannerConfig.ConnectionDistance, ...
                                 'MinTurningRadius',plannerConfig.minTurningRadius,...
                                 'GoalTolerance', plannerConfig.goalTolerance,...
                                 'ConnectionMethod', 'Dubins');

        if plannerFirstIt
            costmapOrg = copy(costmap);
%           stop_Location = changePointToClosest(map, costmap, stop);
            plannerFirstIt = false;
            if isempty(stop)
                disp('Cel zostal wyznaczony poza mapa')
                plannerStatus = false;
                break
            end
        end

        try
        [plannerPosesObj,~] = plan(planner,[start(1:2), rad2deg(start(3))], ...
                                       [stop(1:2), rad2deg(stop(3))]);        
        catch er
            warning(['RRT* nie moze wyznaczyc trasy, powod : ', er.identifier, er.message]);
            if robotRadiusTemp <= robotRadiusConfig.min
                plannerStatus = false;
                break
            end
            if ~exist('plannerPosesObj', 'var')
                robotRadiusTemp = robotRadiusTemp - robotRadiusConfig.step;
                continue;   
            else
                disp('error sie pojawił ale mamy obiekt')
            end

        end
        
        if plannerPosesObj.Length == 0
            warning('Planner return length = 0 path!')
            if robotRadiusTemp <= robotRadiusConfig.min
                plannerStatus = false;
                break
            else
                robotRadiusTemp = robotRadiusTemp - robotRadiusConfig.step; 
            end
            start(3) = start(3) + pi/2; %sztuczna zmiana orientacji (próba wyznaczenia ścieżki za wszelką cene)
            continue
        end

        if ~checkPathValidity(plannerPosesObj,costmap)
            disp('path NOT VALID!!!!!')
        end

        break
    end
 %% INTERPOLACJA   
    if ~plannerStatus 
        return
    end
    
    if robotRadiusTemp ~= robotRadiusConfig.original
        differentRadiusFlag = true; 
    end

    lengths = 0 : pathPointsDistance : plannerPosesObj.Length;
    [path, ~]  = interpolate(plannerPosesObj, lengths);

    path = [path(:,1:2) deg2rad(path(:,3))]; % konwersja deg -> rad
    path(end,3) = path(end-1,3);             % wymiana kąta ostatniego punktu ścieżki

%% UCINANIE ŚCIEŻKI W PRZYPADKU ZAKOŃCZENIA W PUNKCIE NIEDOZWOLONYM

    occupated = checkOccupied(costmapOrg, [path(:,1:2) rad2deg(path(:,3))] );
    if any(occupated)
%             hold on 
%             plot(poses(find(occupated==1), 1),poses(find(occupated==1), 2), 'og')
        if any(occupated(end-2:end))
             warning(['Last positions in blocked area!!!: ',num2str(find(occupated==1)'), ' length: ', num2str(length(occupated)) ]); 
             occupatedIndexes = find(occupated==1);
             path = path(1:occupatedIndexes(1)-1, :);
        else
            warning(['Poses in occupated area!! Poses number:',num2str(find(occupated==1)') ]); 
        end

    end

