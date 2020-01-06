fclc
close all
clear
load lidardata_2.mat map3
load lidardata_2.mat map2
load lidardata_3.mat maps
load lidardata_3.mat maps2
load lidardata_3.mat maps3
% load lidardata_4.mat map1
% load lidardata_4.mat map2
% load lidardata_4.mat map3
% load lidardata_4.mat map4
% load lidardata_4.mat map5
% load lidardata_4.mat map6




%--------DANE POCZATKOWE--------------------------
maxLidarRange = 6;
mapResolution = 40;

Hough_threshold = 0.2;
max_last_points = 4;
NHood_divide = 20;


margin = 5 ;% odleg³oœci miedzy liniami aby mozna by³a uznac dana odeleglosc za podejrzana w px
size_of_checkarea = 0.3;  % promien testowanego obszaru wzgledem testowanego punktu - sprawdzenie czy punkt nie lezy w przeszkodzie
max_lines_consider = 1; %nr aktualnej lini branej pod uwage (w inicie nr 1)

current_map = map2;

%---------------------------------------------------
[scansSLAM,poses] = scansAndPoses(current_map);
omap = buildMap(scansSLAM,poses,mapResolution,maxLidarRange);
inflate(omap, 0.01);
% figure
% show(omap)

%%

BinaryMap = imbinarize(occupancyMatrix(omap, 'ternary'));
BinaryMap = custom_sum_filter(BinaryMap,3,2); % customowy filtr sumujacy
[H,theta,rho] = hough(BinaryMap); 

 
P = houghpeaks(H,30,'threshold',ceil(Hough_threshold*max(H(:))), 'NHoodSize', 2*floor(size(H)/NHood_divide)+1 ,'Theta', [-90:5:85]); % znalezienie punktów przeciecia linia na Houghie
% imshow(H,[],'XData',theta,'YData',rho,'InitialMagnification','fit');
% xlabel('\theta'), ylabel('\rho');
% axis on, axis normal, hold on;
% plot(theta(P(:,2)),rho(P(:,1)),'s','color','white');
lines = houghlines(BinaryMap,theta,rho,P);%,'FillGap',3,'MinLength',8); %zaznaczenie lini na obrazie po prewicie
% figure
% imshow(BinaryMap)
% hold on
% 
% for k = 1:length(lines)
%     xy = [lines(k).point1; lines(k).point2];
%     plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
% end

%% Wyznaczenie podejrzanych punktow próba 1

search_point = lines(1).point1;%przypisanie pierwszej lini z listy do inicjacji poszukiwan


explo_points = []; %tablica na punkty exploracyjne
disp('Szukanie punktow eksploracyjnych...')
for q = 1:2
    if q == 2
        search_point = lines(1).point2;
    end
    to2_num = 1; %inicjalizacja naliczania lini uprzednio wykorzystaqnych
    used = []; %numery u¿ytych lini
    for line_num = 1 : length(lines)

        used(to2_num) = max_lines_consider;
        if(to2_num ==max_last_points)
            to2_num= 1;
        else
            to2_num = to2_num + 1 ;
        end

        MIN_dist = 1000;  
        %----------------------------------------POSZUKIWANIA NAJBLIZSZEGO PUNKTU-------------------------------------------------    
        for search_num = 1 : length(lines) 
            % do znalezienia najblizej lezacej lini
            % wyjsciowe bedzie numer nowej lini new_line_num
            % wyjœciowe bedzie dist = odleg³osc punktu od najblizszej linii
            %------------SPRAWDZENIE CZY LINIA BY£A U¯YTA---------------------
            used_flag = false;
            if ~isempty(find(used == search_num))
                used_flag = true;
                continue;
            end
            %-----------------------------------------------------------------
            if used_flag == false
                % Wyznaczenie najkrótszej odleg³osci punktu od lini
                dist = point_to_line_dist(search_point, lines(search_num).point1, lines(search_num).point2 ); 

                % Wyznacznie punktu przeciecia lini i prostej prostopad³ej przechodz¹cej przez punkt referencyjny
                P_inter = F2_inter_point(search_point, lines(search_num).point1, lines(search_num).point2 );

                % Sprawdzenie czy dany punkt znajduje sie na odciunku
                inrange = insegment_check(P_inter, lines(search_num).point1, lines(search_num).point2);

                nextPt_flag = false; % flaga do informaowania który kolejny punkt lini ma s³uzyc do przeszukiwania (false - p2 , true - p1)

                Pts_dist1 = norm(P_inter - lines(search_num).point1); %odleglosc P_przeciecia -> P1
                Pts_dist2 = norm(P_inter - lines(search_num).point2); %odleglosc P_przeciecia -> P2
                if ~inrange % jezeli nie jest w zakresie to miery odleglosc do najblizeszego punktu odcinka
                         if(Pts_dist1 > Pts_dist2)
                             dist = norm(search_point - lines(search_num).point2); %odleglosc P_ref -> P2
                             nextPt_flag = true;
                             temp_susp_point = [mean([search_point; lines(search_num).point2]), search_point, lines(search_num).point2 ]; %punkt œrodkowy P_ref -> P2
                                                                                                                                          % dwa ostatnie argumenty dodane dla wizualizacji 
                         else
                             dist = norm(search_point - lines(search_num).point1); %odleglosc P_ref -> P1
                             nextPt_flag = false;
                             temp_susp_point = [mean([search_point; lines(search_num).point1]),search_point, lines(search_num).point1 ]; %punkt œrodkowy P_ref -> P1
                         end
                else  % Punkt lezy na odcinku
                         dist = norm(search_point - P_inter);
                         temp_susp_point = [mean([search_point;P_inter]),search_point, P_inter ];        %punkt œrodkowy P_ref -> P_przeciecia
                         if(Pts_dist1 > Pts_dist2)
                             nextPt_flag = true ;                    
                         else
                             nextPt_flag = false;
                         end

                end
                %-----------PRZYPISYWANIE DO MIN-------------------------------
                if(MIN_dist > dist) %&& (dist > margin) % szukanie najblizszego odcinka
                    MIN_dist = dist;
                    new_line = lines(search_num); % zapamietanie danych TEJ lini
                    max_lines_consider = search_num; % numer tej lini z tablicy lini
                    last_lin_num_test = line_num;
                    new_susp_point = temp_susp_point; % tymczasowy podejrzany punkt wychodzacy z pêtli     
                    if nextPt_flag
                        nextPt = new_line.point1;                                                            
                    else
                        nextPt = new_line.point2;
                    end
                end
                %-----------KONIEC PRZYPISYWANIA DO MIN------------------------
            end     
        end
        %----------------------------------------KONIEC POSZUKIWAN NAJBLIZSZEGO PUNKTU---------------------------------------------------------------------- 

    %------------DODANIE PUNKTU PODEJRZANEGO----------------------------------------    
        if MIN_dist > margin
            explo_points(end+1,:) = new_susp_point; 
           
        end
    %--------------------------------------------------------------------------------    


    %----------ODBLOKOWANIE WSZYSTKICH PUNKTOW PRZY 2 OSTATNICH LINIACH--------------    
        if length(used) == (length(lines) - 1) % jezeli tablica uzytych sie prawie zape³ni³a
            % odblokuj wszystkie linie oprocz ostatniej i przedostatniej
            NextToLast_line = used(length(used));
            used(:) = [];
            used(end+1) = NextToLast_line;
        end
    %--------------------------------------------------------------------------------    

        search_point = nextPt; % przypisanie do punktu - do kolejnej iteracji
    end
end

if isempty(explo_points)
    disp("BRAK PUNKTOW EKSPLORACYJNYCH!")
else
    disp('Filtracja punktów eksploracyjnych...')
    %% Przejœcie z punktów we wspó³rzêdnych obrazu binarnego na wspó³rzêdne na mapie
    explo_points_xy = BinaryToMeters(explo_points, omap, BinaryMap );
    
    figure
    show(omap)
    hold on
    plot(explo_points_xy(:,1),explo_points_xy(:,2),'*r') % Intersection points
    %% Filtrowanie punktow lezacych za blisko przeszkód
    area_check = find(CheckArea_xy(explo_points_xy, omap, size_of_checkarea) ==  0); 
    explo_points_xy(area_check, :) = [];
    

    %% Filtrowanie punktow exploracyjnych le¿acych za blisko siebie
    explo_points_xy = Filter_close_points(explo_points_xy, omap, 0.5, size_of_checkarea);

    %% Sprawdzenie zablokowanych obszarów
    blocked_points = []; %potem do przerzucenia do argumentu funkcji
    % mozna wyrzucuc parametr wielkosci obszaru na pocz¹tek (ewentualnie)
    explo_points_xy = Filter_blocked_points(explo_points_xy, blocked_points, 0.1);

    %% Ocena punktów eksploracyjnych
    explo_points_xy = exploratory_points_rating(explo_points_xy, omap, poses(end,:)); 

    target_num = find(explo_points_xy(:,3) == max(explo_points_xy(:,3)));
    target_point =  [explo_points_xy(target_num,1:2) , 0];


    plot(target_point(:,1),target_point(:,2),'ob') % best point
    %----------------wyswietlanie wynikow--------------------------------------
    % if ~isempty(explo_points)
    %     figure
    %     imshow(BinaryMap);
    %     hold on
    %     plot(explo_points(:,1),explo_points(:,2),'*r') % Intersection points
    %     hold on
    %     plot(explo_points(:,3),explo_points(:,4),'oy') % one point
    %     hold on
    %     plot(explo_points(:,5),explo_points(:,6),'oy') % Intersection points
    % 
    %     for k = 1:length(explo_points(:,1))
    %         xyz = [explo_points(k,3),explo_points(k,4)  ; explo_points(k,5), explo_points(k,6), ];
    %         plot(xyz(:,1),xyz(:,2),'LineWidth',1,'Color','yellow');
    %     end
    % else
    %     disp("Mapowanie zakonczone");
    % end



    % %% testowo: odleglosc pozycji od punktow
    %  distance = [];
    %  explo_num = [];
    % for i = 1: length(poses(:,1))
    %     for j = 1: length(explo_points_xy(:,1))
    %         dist = [i norm(poses(i,1:2) - explo_points_xy(j,1:2))]; 
    %         distance(end+1,:) = dist;
    %         if  dist < 1
    %             explo_num(end+1) = j;
    %         end
    %     end
    % end
    % if ~isempty(explo_num)
    %     explo_num = unique(explo_num);
    %     explo_points_xy(explo_num, :) = [];
    %     figure
    %     show(omap)
    %     hold on
    %     plot(explo_points_xy(:,1),explo_points_xy(:,2),'*r') % Intersection points
    % else
    %     disp("Nie ma punktow eksploracyjnych blisko trasy")
    % end
    %% test - oszukanie occupancy map


    aa_new_points = [];
    aa_new_points_xy = [];

     map_height = abs(omap.YWorldLimits(2) - omap.YWorldLimits(1));
     map_width = abs(omap.XWorldLimits(2) - omap.XWorldLimits(1));
     MToBin_rate = (length(BinaryMap(:,1))) / map_height;

     aa_new_map = binaryOccupancyMap(map_width ,map_height, MToBin_rate);
     aa_new_map.LocalOriginInWorld = omap.LocalOriginInWorld;
     setOccupancy(aa_new_map, BinaryMap)
%     figure
%     show(aa_new_map)

    % Wyznacznie trasy 
    disp('Wyznaczanie trasy...')
    estMap = aa_new_map;
    vMap = validatorOccupancyMap;
    vMap.Map = estMap;
    planner = plannerHybridAStar(vMap, 'MotionPrimitiveLength', 0.5,...
                                   'MinTurningRadius', 0.32 );

    % wyznaczenie najlepszego punktu                           
    target_num = find(explo_points_xy(:,3) == max(explo_points_xy(:,3)));
    target_point =  [explo_points_xy(target_num,1:2) , 0];
    %wyznaczenie ostatniej pozycji
    [~, poses] = scansAndPoses(current_map);
    last_pose = poses(end,:);

    route = plan(planner, last_pose, target_point);
    route = route.States;

    % Get poses from the route.
    rsConn = reedsSheppConnection('MinTurningRadius', planner.MinTurningRadius);
    startPoses = route(1:end-1,:);
    endPoses = route(2:end,:);

    rsPathSegs = connect(rsConn, startPoses, endPoses);
%     poses = [];
%     for i = 1:numel(rsPathSegs)
%         lengths = 0:0.1:rsPathSegs{i}.Length;
%         [pose, ~] = interpolate(rsPathSegs{i}, lengths);
%         poses = [poses; pose];
%     end

    figure
    show(planner)
    title('Initial Route to Package')
end







