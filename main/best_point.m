function [target, points_out, del_index]  = best_point(points, rate)
% Funkcja zwracaj¹ca punkt posiadaj¹cy najwiêksz¹ ocenê, punkty
% eksploracyjne z wy³¹czeniem punktu z najwieksza ocena oraz indeks
% usuwanego puntu
% INPUT :
%   - points - punkty w formacie [x y]
%   - rate - ocena punktów 
% OUTPUT:
%   - target - punkt [x y] wybrany jako najlepszy
%   - points_out - punkty wejsciowe z usuniêtym punktem target
%   - del_index - indeks najlepszego punktu wybranego wg kryteriow
%                 przyjetych w funkcji exploratory_points_rating


if ~nargin==2
    error('Niepoprawna liczba argumentow');
elseif ~nargout ==3
    error('Niepoprawna liczba argumentow wyjsciowych');
end
if ~isempty(points)
    if ~length(points(1,:))==2
        error('Punkty musz¹ byc w formacie [x y]')
    elseif ~( length(points(:,1))== length(rate(:,1)) )
        error('Ilosc punktow musi odpowiadac ilosci ocen');
    end
    
    del_index = find(rate== max(rate));                    % wyznaczenie indeksu maksymalnej oceny
    target =  unique(points(del_index(1,1),1:2), 'rows') ; % wyzanczene punktu target (unique zeby usunac te same punkty z te sama ocena - do unikniecia 2 targetow)
    
    points(del_index, :) = [];                             % usuniecie z listy punktow punku target
    
    points_out = points;                                   % macierz wyjsciowa
else
    target = [];
    points_out = [];
    del_index = [];
end