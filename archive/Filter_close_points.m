function filtered_explo_points = Filter_close_points(...
                                 explo_points, ...   % tablica punktow ekploracyjnych [x y]
                                 Map,...             % mapa
                                 MIN_dist,...        % minimalna odleglosc miedzy punktami aby zaszlo filtorwanie
                                 size_of_checkarea)  % rozmiar sprawdzanego obszaru wokol punktu - czy jest za blisko sciany 
% Funkcja zwraca przefiltrowane punty eksploracyjne. Filtracja wystêpuje 
% ze wzglêdu na odleglosci miedzy punktami. Jezeli odleglosc miedzy punktami 
% jest mniejsza od minimalnego, z dwoch punktow wyliczany jest punkt sredni,
% a nastepnie punkty z ktorych zostal wyliczony nowy punkt zostaja usuniete 
% o ile nowy punkt nie lezy za blisko wykrytego na mapie obiektu.
%
% INPUT:
%   explo_points      - tablica punktów eksploracyjnych zawieraj¹ca pary
%                       wspó³rzêdnych [x y]
%   Map               - objekt occupancyMap
%   MIN_dist          - minimalna odleglosc miedzy punktami aby zaszlo filtorwanie 
%   size_of_checkarea - rozmiar sprawdzanego obszaru wokol punktu - czy jest za blisko sciany
%
% OUTPUT:
%   filtered_explo_points - przefiltrowana tablica punktów eksploracyjnych zawieraj¹ca pary
%                           wspó³rzêdnych [x y]

if nargin == 4  && MIN_dist >0
  if ~isempty(explo_points) || length(explo_points(:,1)) > 1
    filtered_num = [];
    explo_points = unique(explo_points, 'rows'); % usuniecie takich samych punktow i ustawienie ich kolejnosci
    new_explo_points = [];
    for explo_num = 1: (length(explo_points(:,1))-1)
        for k = (explo_num+1) : length(explo_points(:,1)) 
            explo_points_distance = norm(explo_points(explo_num, 1:2) - explo_points(k, 1:2));
            if explo_points_distance < MIN_dist
                avr_point = [mean([explo_points(explo_num,1:2); explo_points(k,1:2)]) explo_points(explo_num,1:2) explo_points(k,1:2) ];

                if CheckArea_xy(avr_point(1:2), Map,  size_of_checkarea)
                    new_explo_points(end+1,:)=avr_point;               
                    filtered_num(1, end+1) = explo_num; %zapamietanie numer pierwszego rodzica
                    filtered_num(1, end+1) = k; % zapamietanie numeru punktu drugiego rodzica
                end
            end
        end
    end
    if ~isempty(filtered_num)
        filtered_num = unique(filtered_num); %usuniecie powtarzaj¹cych siê numerow rodzicow punktow
        explo_points(filtered_num, :) = []; % usuniecie 'rodzicow' nowych punktow
        explo_points = [explo_points; new_explo_points] ; %dopisanie nowych punktow
        
    end

  end
  filtered_explo_points = explo_points;
  
  
elseif nargin ==3
    size_of_checkarea = 0.3;
    filtered_explo_points = Filter_close_points(...
                                 explo_points, ...   
                                 Map,...       
                                 MIN_dist,... 
                                 size_of_checkarea);
end