function explo_points_out = exploratory_points2(explo_map, allPoses, maxLidarRange, robotMargin)
% % OPIS FUNKCJI
% Funkcja zajmuje siê wyznaczeniem punktów eksploracyjnych, czyli takich
% do których nale¿y przemieœciæ lidar, aby uzupe³niæ mapê i koñcowo,
% doprowadziæ do jej "zamkniêcia". Dodatkowo oprócz zwrócenia punktów
% eksploracyjnych w formacie [x y] jako trzeci parametr zwracana jest waga
% danego punktu, przez funkcjê exploratory_points_rating(). Punkt z
% najwy¿sz¹ wag¹ powinien zostaæ rozwa¿ony jako naj³atwiejszy do
% przemieszczenia.
% 
% % Technicznie:
% Dla ka¿dej pozycji sprawdzane jest czy istnieje punkt przeciecia z map¹
% dla 'NumOfRays' liczby promieni dla pocz¹tkowej d³ugoœci 'ray_length' (pocz¹tkowo 3m)
% Je¿eli wszystkie trafiaja na przeszkodê promien zawê¿a siê a¿ do
% osi¹gniêcia przerwy w postaci kilku promieni które w nic nie trafiaj¹ na
% danym promieniu i tam zostaje wyznaczony punkt eksploracyjny.
% 
% INPUT:
%    - map - occupancyMap
%    - occ_poses - wszystkie pozycje w ktorych przebywal robot
%    - maxLidarRange - maksymalny zakres lidaru w metrach
%    - robotMargin - wartoœc o któr¹ zostanie powiêkszone przeszkoday na
%      mapie (do funkcji inflate)
% OUTPUT
%    - explo_points_out - macierz punktów eksploracyjnych i ich wag w formacie [ x y rate]   


% Obs³uga b³êdów argumentów funkcji
if ~isa(explo_map, 'occupancyMap')  
    error("map - musi to byæ objekt occupancyMap");
elseif (nargin < 3)
    error("Do funkcji nale¿y przekazac 3 argumenty patrz -> help ")
end

% mapResolution = 50;      % rozdzielczoœæ mapy 
NumOfRays = 72; %64         % liczba promieni przypadaj¹ca na ka¿d¹ pozycjê

rayLength = 2.5;          % promieñ sprawdzanego okrêgu pod wzglêdem przeszkód
minGap = 0.5;          % odlegosc miedzy ostatnio odnalezionymi punktami stycznosci z map¹
rayLength_step = 0.3;   % krok z jakim odejmowana jest d³ugoœæ promienia w przypadku braku punktow eksploracyjnych

   
omap = copy(explo_map);

inflate(omap, robotMargin);

MaxAngle = 2*pi;
MinAngle = 0;
angleResolution = MaxAngle / NumOfRays;
alfa = (MinAngle:(angleResolution) :(MaxAngle-angleResolution)); %zakres k¹towy promieni
interPoints = [];

minNaNLength = 1;        % minimalna ilosc punktow nan aby zasz³o wyznaczanie punktu eksploracyjnego
exploPoints = [];         % macierz na punty eksploracyjne

inter_Pt_mean_temp = [];   % macierz na tymczasowe œrednie dla poszczególnych pozycji
inter_Pt_mean = 0;         % INIT -œrednia d³ugosci promieni dla ostatnich pozycji MINIMUM WYSZUKIWANIA PROMIENIA
firstItFlag = true;       % Flaga pierwszej iteracji
tooShortRay = false;  % flaga wystawiana gdy nie ma 'dziur' na wyszukiwanym promieniu

%% Wyznaczenie punktów eksploracyjncch
while rayLength > inter_Pt_mean && rayLength <= (maxLidarRange - rayLength_step)
    for i = 1 : length(allPoses(:,1))
        
        interPoints = rayIntersection(omap,[allPoses(i,1:2) 0], alfa , rayLength);   % wyznaczenie punktów przeciecia
        for j = 1: length(interPoints(:,1))
            interPoints(j,3)  = norm(interPoints(j,1:2) - allPoses(i,1:2));              % wyznaczenie odleglosci miedzy badana pozycja, a wyznaczonymi punktami
        end
        
        nonan_numbers = find(isnan(interPoints(:,1)) == 0);                        % wyznaczneie numerów promieni, które trafiaja w przeszkode
        if isempty(nonan_numbers)
            warning("nie mozna wyznaczyc punktow eksploracyjnych, poniewaz nie ma nic w promieniu poszukiwan! ");
            tooShortRay = true;
            break;
        end
        if firstItFlag
            inter_Pt_mean_temp(end+1, :) =  median(interPoints(nonan_numbers ,3)) ;  % obliczanie œredniej d³ugoœci promieni dla danej pozycji (TYLKO DLA PIERWSZEJ ITERACJI G£ÓWNEJ PÊTLI)
        end
        
        if ~((length(nonan_numbers)) == NumOfRays)                              % je¿eli wszystkie punkty trafiaj¹ to nie wyznaczymy punktów eksploracyjnych
            for k = 1:length( nonan_numbers)-1                                  % petla sprawdzajaca odleglosci miedzy punktami miedzy którymi promien na nic nie natrafi³
                if (nonan_numbers(k+1) - nonan_numbers(k)) > minNaNLength     
                    dist = norm(interPoints(nonan_numbers(k),1:2) - interPoints(nonan_numbers(k+1),1:2));
                    if dist > minGap
                        % wpisanie do tablicy punktow ekspolracyjnych
                        exploPoints(end+1,:) = mean([interPoints(nonan_numbers(k),1:2); interPoints(nonan_numbers(k+1),1:2)]);
                    end
                end
            end
            
            % przypadek gdy promien nie trafia na samym pocz¹tku lub koncu listy
            LastToFirst_nan_length = (nonan_numbers(1,1) - 1) + (NumOfRays - nonan_numbers(end,1)) + 1;
            if  LastToFirst_nan_length > minNaNLength
                dist = norm(interPoints(nonan_numbers(1),1:2) - interPoints(nonan_numbers(end),1:2));
                if dist > minGap
                    % wpisanie do tablicy punktow ekspolracyjnych
                    exploPoints(end+1,:) = mean([interPoints(nonan_numbers(1),1:2); interPoints(nonan_numbers(end),1:2)]);
                end
            end
        end
    end
    if tooShortRay && rayLength <= (maxLidarRange - rayLength_step)
        tooShortRay = false;
        rayLength = rayLength + rayLength_step;
        continue;
    end

    if firstItFlag
        inter_Pt_mean = median(inter_Pt_mean_temp); % obliczenie œredniej ze œrednich d³ugoœci promieni wyznaczonych dla kazdej pozycji (TYLKO W PIERWSZEJ ITERACJI)
        firstItFlag  = false;
    end
    
    if ~isempty(exploPoints)
        break;
    else
        rayLength = rayLength - rayLength_step;
        continue;
    end
    
end 

explo_points_out = exploratory_points_rating(exploPoints, omap, allPoses(end,:), maxLidarRange);




