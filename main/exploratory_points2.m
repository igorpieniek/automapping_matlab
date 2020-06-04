function explo_points_out = exploratory_points2(explo_map, explo_points_all, last_pos_num, occ_poses, middle_area_points, maxLidarRange, robotMargin)
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
%    - explo_points_all - ostatnio wyznaczone punkty eksploracyjne (jeœli istniej¹)   
%    - last_pos_num - numer pozycji przy której zosta³ osi¹gniêty ostatni
%                     punkt eksploracyjny (o ile by³ wyznaczony i osi¹gniêty )
%    - occ_poses - wszystkie pozycje w ktorych przebywal robot
%    - middle_area_points - punkty w formacie [x y radius] okreslaja
%                           obszary w ktorych wyznaczone punkty nie powinny
%                           sie znalezc (na ten moment funkcja wy³aczona)
%    - maxLidarRange - maksymalny zakres lidaru w metrach
%
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

ray_length = 2;          % promieñ sprawdzanego okrêgu pod wzglêdem przeszkód
MIN_DIST = 0.15;          % odlegosc miedzy ostatnio odnalezionymi punktami stycznosci z map¹
ray_length_step = 0.3;   % krok z jakim odejmowana jest d³ugoœæ promienia w przypadku braku punktow eksploracyjnych
size_of_checkarea = 0.2; % minimalna odleg³oœæ punktu eksploracyjnego od przeszkody

PROJECT_RESULTS = false; % wyswietlanie wyników ? (do testowania)

% if isa(explo_map, 'lidarSLAM')
%     [scans,poses] = scansAndPoses(explo_map);                         
%     omap = buildMap(scans,poses,explo_map.MapResolution, maxLidarRange); %stworzenie mapy o danej rozdzielczoœci
% elseif isa(explo_map, 'occupancyMap')
%     
    omap = copy(explo_map);
    poses = occ_poses; % to musz¹ byc wszystkie pozycje!
% end

inflate(omap, robotMargin);

% rangefinder = rangeSensor('HorizontalAngle', [-pi pi],'RangeNoise', 0.001,'Range' , [0 8]);
% [ranges, ~] = rangefinder(poses(end,:), omap);


MaxAngle = 2*pi;
MinAngle = 0;
angleResolution = 2*pi / NumOfRays;
alfa = (MinAngle:(angleResolution) :(MaxAngle-angleResolution)); %zakres k¹towy promieni
inter_Pt = [];


min_nan_length = 1;        % minimalna ilosc punktow nan aby zasz³o wyznaczanie punktu eksploracyjnego
explo_points = [];         % macierz na punty eksploracyjne

inter_Pt_mean_temp = [];   % macierz na tymczasowe œrednie dla poszczególnych pozycji
inter_Pt_mean = 0;         % INIT -œrednia d³ugosci promieni dla ostatnich pozycji MINIMUM WYSZUKIWANIA PROMIENIA
firstIt_flag = true;       % Flaga pierwszej iteracji
short_ray_length = false;  % flaga wystawiana gdy nie ma 'dziur' na wyszukiwanym promieniu

%% Wyznaczenie punktów eksploracyjncch
while ray_length > inter_Pt_mean && ray_length <= (maxLidarRange - ray_length_step)
    for i = last_pos_num : length(poses(:,1))
        
        inter_Pt = rayIntersection(omap,[poses(i,1:2) 0], alfa , ray_length);   % wyznaczenie punktów przeciecia
        for j = 1: length(inter_Pt(:,1))
            inter_Pt(j,3)  = norm(inter_Pt(j,1:2) - poses(i,1:2));              % wyznaczenie odleglosci miedzy badana pozycja, a wyznaczonymi punktami
        end
        
        nonan_numbers = find(isnan(inter_Pt(:,1)) == 0);                        % wyznaczneie numerów promieni, które trafiaja w przeszkode
        if isempty(nonan_numbers)
            warning("nie mozna wyznaczyc punktow eksploracyjnych, poniewaz nie ma nic w promieniu poszukiwan! ");
            short_ray_length = true;
            break;
        end
        if firstIt_flag
            inter_Pt_mean_temp(end+1, :) =  median(inter_Pt(nonan_numbers ,3)) ;  % obliczanie œredniej d³ugoœci promieni dla danej pozycji (TYLKO DLA PIERWSZEJ ITERACJI G£ÓWNEJ PÊTLI)
        end
        
        if ~((length(nonan_numbers)) == NumOfRays)                              % je¿eli wszystkie punkty trafiaj¹ to nie wyznaczymy punktów eksploracyjnych
            for k = 1:length( nonan_numbers)-1                                  % petla sprawdzajaca odleglosci miedzy punktami miedzy którymi promien na nic nie natrafi³
                if (nonan_numbers(k+1) - nonan_numbers(k)) > min_nan_length     
                    dist = norm(inter_Pt(nonan_numbers(k),1:2) - inter_Pt(nonan_numbers(k+1),1:2));
                    if dist > MIN_DIST
                        % wpisanie do tablicy punktow ekspolracyjnych
                        explo_points(end+1,:) = mean([inter_Pt(nonan_numbers(k),1:2); inter_Pt(nonan_numbers(k+1),1:2)]);
                    end
                end
            end
            
            % przypadek gdy promien nie trafia na samym pocz¹tku lub koncu listy
            LastToFirst_nan_length = (nonan_numbers(1,1) - 1) + (NumOfRays - nonan_numbers(end,1)) + 1;
            if  LastToFirst_nan_length > min_nan_length
                dist = norm(inter_Pt(nonan_numbers(1),1:2) - inter_Pt(nonan_numbers(end),1:2));
                if dist > MIN_DIST
                    % wpisanie do tablicy punktow ekspolracyjnych
                    explo_points(end+1,:) = mean([inter_Pt(nonan_numbers(1),1:2); inter_Pt(nonan_numbers(end),1:2)]);
                end
            end
        end
    end
    if short_ray_length && ray_length <= (maxLidarRange - ray_length_step)
        short_ray_length = false;
        ray_length = ray_length + ray_length_step;
        continue;
    end
%----------------------------------------------------------   
    if PROJECT_RESULTS && ~isempty(explo_points)         
        figure                                              
        show(omap)
        hold on
        plot(explo_points(:,1),explo_points(:,2),'*r')
        title('Wszystkie punkty eksploracyjne')
    end
    %----------------------------------------------------------
    if firstIt_flag
        inter_Pt_mean = median(inter_Pt_mean_temp); % obliczenie œredniej ze œrednich d³ugoœci promieni wyznaczonych dla kazdej pozycji (TYLKO W PIERWSZEJ ITERACJI)
        firstIt_flag  = false;
    end
    
    %% 1 Filtracja punktów : usuniecie punktów które lez¹ za blisko osiagnietych pozycji oraz powtarzajacych sie punktow
    %
        if ~isempty(explo_points)
            break;
        else
            ray_length = ray_length - ray_length_step;
            continue;
        end
    
    %----------------------------------------------------------
    if PROJECT_RESULTS && ~isempty(explo_points)
        figure
        show(omap)
        hold on
        plot(explo_points(:,1),explo_points(:,2),'*r')
        title('Bez punktów lezacych za blisko osiagnietych pozycji')
    end
    %----------------------------------------------------------

    %% WYŒWIETLENIE WYNIKÓW ORAZ PRZERWANIE PROCESU W PRZYPADKU WYSTEPOWANIA PUNKTÓW NA TYM ETAPIE
    if ~isempty(explo_points)
        %----------------------------------------------------------
        if PROJECT_RESULTS
            figure
            show(omap);
            hold on
            plot(explo_points(:,1),explo_points(:,2),'*r')
            hold on
            plot(poses(end,1),poses(end,2),'ok')
            title('Po K-means + najlepszy punkt')
        end
        %----------------------------------------------------------
        
        break;
    end
end % koniec

explo_points_out = exploratory_points_rating(explo_points, omap, poses(end,:), maxLidarRange);




