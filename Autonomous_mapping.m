% G��wny plik autonomicznego mapowania terenu

map = Lidar_Init;
blocked_points = [];
for i =1:2
    map = LidarAq(map); %kilka wstp�pnych pomair�w
end

while true
    map = LidarAq(map);
    explo_points = exploratory_points(map, blocked_points);
    if isempty(explo_points)
        disp ("Mapowanie zakonczone!");
        disp(" ");
        con = input('Czy mapa jest w porz�dku ?[Y/N]');
        if      con == 'Y' || con == 'y'
               disp ("Algorytm Mapowania konczy prace!");
               break;
        elseif  con == 'N' || con == 'n'
            disp ("Algorytm Mapowania kontynuuje prace");
             continue;
        else
            error("Nie wybrano zadnej odpowiedzi")
        end
    else
       % wyb�r punktu ze wzgl�du na najwy�sz� ocene
       target_num = find(explo_points(:,3) == max(explo_points(:,3)));
       target_point =  explo_points(target_num,1:2);
       %wyznaczenie ostatniej pozycji
       [~, poses] = scansAndPoses(map);
       last_pose = poses(end,:);
       
       % wyznaczenie �ciezki + jazda
        
        
    end
end