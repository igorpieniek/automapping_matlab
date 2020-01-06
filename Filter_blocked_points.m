function explo_points_output = Filter_blocked_points(explo_points, blocked_points, min_AreaSize)
% Funkcja sprawdzaj�ca obszary zablokowane. Je�eli jeden z punkt�w
% eksploracyjnych znajduje si� w obszarze zablokowanym zostaje on usuni�ty
% z listy punktow eksploracyjnych.
%
%INPUT:
%   expl_point     - tablica punkt�w eksploracyjnych zawieraj�ca pary
%                    wsp�rz�dnych [x y]
%   blocked_points - tablica punkt�w kt�re powinny zosta� wy��czone, wraz z
%                    obszarem je okalaj�cym
%   min_AreaSize   - wielko�� sprawdzanego obszaru woko� punktu
%                    zablokowanego w metrach
%OUTPUT:
%   explo_points_output - tablica punkt�w eksploracyjnych po usuni�ciu
%                         punkt�w zablokowanych

if nargin == 3
    if isempty(blocked_points)
        explo_points_output = explo_points;
       return;
    end
    if min_AreaSize < 0
        error("Wielkos� obszaru musi byc dodatnia")
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
    error("Za ma�o argument�w!");
end