function point_out = changePointToClosest(occMap,costmap ,point)
% Funkcja służąca do przesunięcia wybranego punktu eksploracyjnego w obszar
% uznany przez mapę kosztów za dostępny. Jeżeli punkt uznany za cel
% znajduje się w punkcie okupowanym, to sprawdzany jest najbliższy obszar
% wokół niego. Zostaje zwrócony najbliższy uznany za punkt w przestrzeni
% wolnej. W przypadku gdy w najblizsyzm obszarze nie zostanie odnaleziony
% taki punkt zwracana jest pusta tablica
% INPUT
% - occMap - obiekt mapy occupancyMap
% - costmap - obiekt klasy vehicleCostmap
% - point - punkt będacy celem
% OUTPUT
% - point_out - nowy punkt lub ten sam wprowadzony cel

cellStep = occMap.XLocalLimits(2) / (occMap.GridSize(1));
maxRadius = 5 * cellStep;
angleRes = deg2rad(20);

if checkOccupied(costmap, [point(1,1:2) rad2deg(point(1,3))])
    for i = cellStep : cellStep : maxRadius
        for angle = 0: angleRes: 2*pi
            x = point(1,1) + i*cos(angle);
            y = point(1,2) + i*sin(angle);
            try
                if checkFree(costmap, [x,y, rad2deg(point(1,3))])
%                     hold on
%                     plot(x, y, '.b')
                    disp(["Found free point:", num2str([x,y])," on dist ", num2str(i)," and angle ",num2str(rad2deg(angle))]);
                    point_out=[x,y,point(1,3)];
                    return
                else
%                     hold on
%                     plot(x, y, '.m')
                end
            catch er
                warning(['CheckFree in changePoint To closest function have problem ', er.identifier, er.message]);
            end
        end
    end
   point_out=[];
   return
   
else 
    point_out = point;
    return 
end