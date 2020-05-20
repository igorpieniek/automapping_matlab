function out = middle_points(map, angles, ranges, poses)
% Funkcja ma za zadanie wyznaczyæ promien najmnijeszego okrêgu jaki mo¿na
% wpisaæ w obszar na mapie wg danego obszaru. 
% Funkcja zwracaj¹ca macierz w postaci[x y radius]

% check czy zakres to 360 stopni - jak nie ma to ten pomys³ nie przejdzie
if(mod(length(ranges),2) == 0)
    onehalf = ranges(1 : length(ranges(:,1))/2, :); % ustalenie po³ówek k¹tów
    secondhalf =  ranges((length(ranges(:,1))/2)+1 : end, :);
    d = onehalf + secondhalf; % dodanie przeciwnych promieni tworz¹c macierz œrednic
    [min_d , index] = min(d); % wyznaczneie minimalnej œrednicy oraz jej indeksu wzglêdem macierzy
    angle = [angles(index) angles(index + length(secondhalf))]; % wziêcie tych k¹tów z angles
    interPoints = rayIntersection(map, poses(end, :), angle, 2*min_d); % wyznaczenie punktów przeiciêæ
    out= [mean(interPoints) (min_d/2)]; % format [x y r]
      
end