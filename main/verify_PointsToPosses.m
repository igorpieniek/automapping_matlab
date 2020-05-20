function points_out = verify_PointsToPosses(ver_points, middle_area_points)
% Funkcja filtruj¹ca punkty znajdujace sie za blisko osiagnietych pozycji.
% Ma za zadanie zablokowac lub usuwac punkty  znajdujace sie w obszarze
% mniejszym niz d³ugosc promienia (charakterystycznego dla danego punktu)
% od ka¿dego z punktow middle_area_points.
% Punkty middle_area_points sa wyznaczone przez funkcjê middle_points()
%
% INPUT:
%   - ver_points - punkty do weryfikacji w formacie [x y]
%   - middle_area_points - punkty wytworzone z middle_points() w formacie [x y radius] 
% OUTPUT:
%   - points_out - przefiltrowane punkty 

if ~(nargin == 2)
    error('Nieprawidlowa ilosc argumentow');
elseif ~isempty(ver_points) && length(ver_points(1,:))<2
    error('Punkty przeznaczone do filtracji musza byc macierza w formacie [x y]')
elseif ~isempty(middle_area_points) && ~length(middle_area_points(1,:))== 3
    error('Punkty filtrujace musza byc macierza w formacie [x y radius]')    
end

if ~isempty(ver_points) && ~isempty(middle_area_points)
    temp_del_num = [];                          % macierz na usuwane indeksy 
    for j = 1: length(middle_area_points(:,1))  
        for i = 1: length(ver_points(:,1))
            dist = norm(ver_points(i,1:2) - middle_area_points(j,1:2)); % obliczenie odleglosci danego punktu od danego punktu œrodkowego obszaru
            if dist <= middle_area_points(j,3)                          % gdy odleglosc jest mniejsza od danego promienia punkt zostaje usuniety
                temp_del_num(1, end+1) = i;                             % zapisanie indeksu
            end
        end
    end
    temp_del_num = unique(temp_del_num); % usuniecie powtorzen
    ver_points(temp_del_num, :) = [];    % usuniecie punktow
    points_out = ver_points;
else
    points_out = [];
end