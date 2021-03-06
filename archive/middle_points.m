function out = middle_points(map, angles, ranges, poses, midPoints)
% Funkcja ma za zadanie wyznaczy� promien najmnijeszego okr�gu jaki mo�na
% wpisa� w obszar na mapie wg danego obszaru. 
% Funkcja zwracaj�ca macierz w postaci[x y radius]

% check czy zakres to 360 stopni - jak nie ma to ten pomys� nie przejdzie
if(mod(length(ranges),2) == 0)
    onehalf = ranges(1 : length(ranges(:,1))/2, :); % ustalenie po��wek k�t�w
    secondhalf =  ranges((length(ranges(:,1))/2)+1 : end, :);
    d = onehalf + secondhalf; % dodanie przeciwnych promieni tworz�c macierz �rednic
    [min_d , index] = min(d); % wyznaczneie minimalnej �rednicy oraz jej indeksu wzgl�dem macierzy
    angle = [angles(index) angles(index + length(secondhalf))]; % wzi�cie tych k�t�w z angles
    interPoints = rayIntersection(map, poses(end, :), angle, 2*min_d); % wyznaczenie punkt�w przeici��
    out= [mean(interPoints) 0.95.*(min_d/2)]; % format [x y r]
    
    if ~isempty(midPoints)
        if length( midPoints( :,1) ) < 2
            avrg = mean(midPoints(:,3));
        else
            avrg = mean(midPoints(end-1 : end,3));
        end

        if out(1,3) >= avrg + 0.25
            out(1,3) = avrg;
%             hold on
%             viscircles(out(:,1:2), out(:,3),'Color', 'b' );
        end
    end
      
end