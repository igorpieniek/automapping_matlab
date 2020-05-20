function explo_points_output = Filter_blocked_points(explo_points, blocked_points, min_AreaSize)
% Funkcja sprawdzaj¹ca obszary zablokowane. Je¿eli jeden z punktów
% eksploracyjnych znajduje siê w obszarze zablokowanym zostaje on usuniêty
% z listy punktow eksploracyjnych.
%
%INPUT:
%   expl_point     - tablica punktów eksploracyjnych zawieraj¹ca pary
%                    wspó³rzêdnych [x y]
%   blocked_points - tablica punktów które powinny zostaæ wy³¹czone, wraz z
%                    obszarem je okalaj¹cym
%   min_AreaSize   - wielkoœæ sprawdzanego obszaru woko³ punktu
%                    zablokowanego w metrach
%OUTPUT:
%   explo_points_output - tablica punktów eksploracyjnych po usuniêciu
%                         punktów zablokowanych

if nargin == 3
    if isempty(blocked_points)
        explo_points_output = explo_points;
       return;
    end
    if min_AreaSize < 0
        error("Wielkosæ obszaru musi byc dodatnia")
    end
    
    num_pointsToErase = [];
    for bnum = 1: length(blocked_points(:,1))
        for explonum = 1: length(explo_points(:,1))
            dist = norm( blocked_points(bnum,1:2) - explo_points(explonum,1:2) );
            if dist < min_AreaSize
                num_pointsToErase(end+1) = explonum;
            end
        end
    end
    num_pointsToErase = unique(num_pointsToErase);
    explo_points( num_pointsToErase,:) = [];
    explo_points_output = explo_points;
    
   
else
    error("Za ma³o argumentów!");
end