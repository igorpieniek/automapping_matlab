function explo_points_rating = exploratory_points_rating(exploPoints, map, currentPose, maxLidarRange)
% Funkcja oceniaj�ca punkty eksploracyjne w celu okre�lenia punktu do 
% kt�rego bedzie naj�atwiej dotrze� robotowi mobilnemu. Zwraca tablice
% punkt�w ekspolracyjnych z do��czon� w trzeciej kolumnie ocenie danego punktu.
% Ocena ka�dego punktu jest iloczynem poszczeg�lnych ocen:
%     - ocena "obudowania" punktu eksploracyjnego przez map� (ocenia w 
%       jakim stopniu punkt jest wewn�trz obrysu do mapy)
%     - ocena odleg�o�ci (w lini prostej) ostatniej pozycji od punktu
%       eksploracyjnego - najmniejsza odleglo�� ma najwy�sz� ocen�
%     - ocena opieraj�ca si� na sprawdzeniu czy na prostej ��cz�cej
%       ostatni� pozycj� i punkt eksploracyjny zosta�a wykryta przeszkoda
%                                                                              
% INPUT:
%   - explo_point - tablica punkt�w eksploracyjnych zawieraj�ca pary
%                wsp�rz�dnych [x y]
%   - map        - objekt occupancyMap
%   - last_pose  - ostatnia zapisana pozycja (aktualna pozycja obiektu) 
%   - maxLidarRange - maksymalny zakres Lidaru
%
% OUTPUT:
%   - explo_points_rating - tablica punkt�w eksploracyjnych zawieraj�ca pary
%                wsp�rz�dnych oraz ocene danego punktu w formacie [x y rate]

if isempty(exploPoints)
    explo_points_rating = [];
    return
end

if nargin < 3
    error("Za ma�o argument�w");
elseif ~isa(map, 'occupancyMap')
    error("map - musi byc obiektem occupancyMap")
end

angleResolution = pi/16;
angles = (0: (angleResolution): (2*pi)); %k�ty dla kt�rych sprawdza si� zderzenia z obrysem mapy

% Tymczasowe tablice pomocnicze do obliczenia danych wsp�czynnik�w
enclosedRate_temp = [];
distRate_temp = [];
interRate_temp = [];

for k = 1: length(exploPoints(:,1))
    % Sprawdzenie obudowania punkt�w explo_point(k,1:2);  
    intsectionPts = rayIntersection(map,[exploPoints(k,1:2) 0], angles, maxLidarRange);
    enclosedRate_temp(end+1,1) = sum( isnan( intsectionPts(:,1) ) ); %sumowanie ilo�ci promieni kt�re nie trafi�y na przeszkod� na mapie
    
    % Sprawdzenie odleglosci od ostatniej pozycji
    distRate_temp(end+1,:) = norm( currentPose(1,1:2) - exploPoints(k,1:2));
    
    % Sprawdzenie czy mi�dzy ostatni� pozycj�, a punktem eksploracyjnym istnieje przeszkoda 
    alfa = Angle2Points( currentPose(1,1:2), exploPoints(k,1:2) ); 
    interPoints = rayIntersection( map,[ currentPose(1,1:2) 0], alfa , maxLidarRange ); % znalezienie punktu zderzenia z przeszkoda (o ile istnieje) na k�cie na kt�rym jest punkt eksploracyjny
    %alfa jest liczone wzgl�dem k�ta wpisanego w drugim argumencie, w 3
    %kolumnie

    if isnan(interPoints) 
        % punkt w nic nie trafia - jest "czysta" droga do punktu
        % eksploracyjnego
        interRate_temp(end+1,1) = true;
    else
        %sprawdzenie czy odleg�o�� ostatniego po�o�enia od punktu zderzenia wi�ksza r�wna od
        %odleg�o�ci od punktu eksploracyjnego
        dist_inter = norm(currentPose(1,1:2) - interPoints );
        if dist_inter >= distRate_temp(end)
           %punkt jest okej, mamy "czyste przebicie" do punktu
           %eksploracyjnego
            interRate_temp(end+1,1) = true;    
        else
            %nie mamy dojscia bezposredniego - na drodze jest przeszkoda
            interRate_temp(end+1,1) = false;
        end
    end
end

    % OBLICZENIE OCENY DANEGO PUNKTU
    
    % enclosed rate: 1 - enclosed_rate/length(angles)
    % dist rate : 1 - dist_rate / max_dist_rate
    % inter rate: false -> * 0.75, true -> *1.25
    % nastepnie wymnozenie wszystkiego
    enclosed_rate = 1;% - (enclosed_rate_temp./length(angles)); %OBUDOWANIE
    dist_rate = 1 - ( distRate_temp ./max(distRate_temp));  %DYSTANS MIEDZY OSTATNIM POLOZENIEM A PUNKTEM
    inter_rate = 1;%(inter_rate_temp +1) * 0.6; %CZY LINIA WYZNACZONA OD OSTATNIEGO PO�OZENIA DO PUNKTU EKSPLORACYNEGO NA COS WPADA CZY NIE
    
    rate = [];
    rate =   enclosed_rate .* dist_rate .*  inter_rate;
    
    explo_points_rating(:,1:2) = exploPoints;    
    explo_points_rating(:,3) = rate;


    


