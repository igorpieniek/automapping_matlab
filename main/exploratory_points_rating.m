function explo_points_rating = exploratory_points_rating(explo_point, map, last_pose, maxLidarRange)
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



if nargin < 3
    error("Za ma�o argument�w");
elseif ~isa(map, 'occupancyMap')
    error("map - musi byc obiektem occupancyMap")
end

angles = (0: (pi/16): (2*pi)); %k�ty dla kt�rych sprawdza si� zderzenia z obrysem mapy

% Tymczasowe tablice pomocnicze do obliczenia danych wsp�czynnik�w
enclosed_rate_temp = [];
dist_rate_temp = [];
inter_rate_temp = [];

for k = 1: length(explo_point(:,1))
    % Sprawdzenie obudowania punkt�w explo_point(k,1:2);  
    intsectionPts = rayIntersection(map,[explo_point(k,1:2) 0],angles, maxLidarRange);
    enclosed_rate_temp(end+1,1) = sum(isnan(intsectionPts(:,1))); %sumowanie ilo�ci promieni kt�re nie trafi�y na przeszkod� na mapie
    
    % Sprawdzenie odleglosci od ostatniej pozycji
    dist_rate_temp(end+1,:) = norm(last_pose(1,1:2) - explo_point(k,1:2));
    
    % Sprawdzenie czy mi�dzy ostatni� pozycj�, a punktem eksploracyjnym istnieje przeszkoda 
    if (explo_point(k,1) - last_pose(1,1)) == 0 % wsp. a czyli tg alfa nie mozliwy do policzenia bo x2-x1 = 0 (z funkcji liniowej)
        % rozwi�zanie problemu k�t�w 90 i -90 stopni
        if (explo_point(k,2) > last_pose(1,2))
            alfa = pi/2;
        elseif (explo_point(k,2) == last_pose(1,2))
            % zablokowac punkt - to sa te same punkty, malo prawdopodobne
            inter_rate_temp(end+1,1) = false;
            continue; 
        else
            alfa = -pi/2;
        end
    else
        alfa = atan((explo_point(k,2) - last_pose(1,2)) /(explo_point(k,1) - last_pose(1,1))) ; %f. linowa gdzie a = tg alfa = y2-y1/x2-x1
        if alfa < 0 
            alfa = pi + alfa; %problem arctan
        end
    end
%     alfa = Angle2Points(last_pose(1,1:2),explo_point(k,1:2)); 
    inter_Pt = rayIntersection(map,[last_pose(1,1:2) 0], alfa , 10); % znalezienie punktu zderzenia z przeszkoda (o ile istnieje) na k�cie na kt�rym jest punkt eksploracyjny
    %alfa jest liczone wzgl�dem k�ta wpisanego w drugim argumencie, w 3
    %kolumnie

    if isnan(inter_Pt) 
        % punkt w nic nie trafia - jest "czysta" droga do punktu
        % eksploracyjnego
        inter_rate_temp(end+1,1) = true;
    else
        %sprawdzenie czy odleg�o�� ostatniego po�o�enia od punktu zderzenia wi�ksza r�wna od
        %odleg�o�ci od punktu eksploracyjnego
        dist_inter = norm(last_pose(1,1:2) - inter_Pt );
        if dist_inter >= dist_rate_temp(end)
           %punkt jest okej, mamy "czyste przebicie" do punktu
           %eksploracyjnego
            inter_rate_temp(end+1,1) = true;    
        else
            %nie mamy dojscia bezposredniego - na drodze jest przeszkoda
            inter_rate_temp(end+1,1) = false;
        end
    end
end

    % OBLICZENIE OCENY DANEGO PUNKTU
    
    % enclosed rate: 1 - enclosed_rate/length(angles)
    % dist rate : 1 - dist_rate / max_dist_rate
    % inter rate: false -> * 0.75, true -> *1.25
    % nastepnie wymnozenie wszystkiego
    enclosed_rate = 1;% - (enclosed_rate_temp./length(angles)); %OBUDOWANIE
    dist_rate = 1 - ( dist_rate_temp ./max(dist_rate_temp));  %DYSTANS MIEDZY OSTATNIM POLOZENIEM A PUNKTEM
    inter_rate = 1;%(inter_rate_temp +1) * 0.6; %CZY LINIA WYZNACZONA OD OSTATNIEGO PO�OZENIA DO PUNKTU EKSPLORACYNEGO NA COS WPADA CZY NIE
    
    rate = [];
    rate =   enclosed_rate .* dist_rate .*  inter_rate;
    
    explo_points_rating(:,1:2) = explo_point;    
    explo_points_rating(:,3) = rate;


    


