function explo_points = exploratory_points(map, blocked_points)

%--------DANE POCZATKOWE--------------------------
maxLidarRange = 6;
mapResolution = 70;

Hough_threshold = 0.2;
max_last_points = 4;
NHood_divide = 20;


margin = 5 ;%[px] odleg³oœci miedzy liniami aby mozna by³a uznac dana odeleglosc za podejrzana w px
size_of_checkarea = 0.3;  %[m] promien testowanego obszaru wzgledem testowanego punktu - sprawdzenie czy punkt nie lezy w przeszkodzie
min_dist_between_exPoints = 0.5; %[m] minimalna odleg³oœæ miedzy punktami eksploracyjnymi aby mozna by³o uznac ze leza za blisko siebie
size_of_blockedArea = 0.1; %[m] promieñ obszaru wokó³ zablokowanego obszaru


%---------------------------------------------------
[scansSLAM,poses] = scansAndPoses(map);
omap = buildMap(scansSLAM,poses,mapResolution,maxLidarRange);
inflate(omap, 0.01);


%% Binaryzacja i transformata Hough'a

BinaryMap = imbinarize(occupancyMatrix(omap, 'ternary'));
BinaryMap = custom_sum_filter(BinaryMap,3,2); % customowy filtr sumujacy
[H,theta,rho] = hough(BinaryMap); 

 
P = houghpeaks(H,30,'threshold',ceil(Hough_threshold*max(H(:))), 'NHoodSize', 2*floor(size(H)/NHood_divide)+1 ,'Theta', [-90:5:85]); % znalezienie punktów przeciecia linia na Houghie
lines = houghlines(BinaryMap,theta,rho,P);%,'FillGap',3,'MinLength',8); %zaznaczenie lini na obrazie po prewicie

%% Wyznaczenie podejrzanych punktow próba 1

search_point = lines(1).point1;%przypisanie pierwszej lini z listy do inicjacji poszukiwan
max_lines_consider = 1; %nr aktualnej lini branej pod uwage przy kolejnych iteracjach (w inicie nr 1)
explo_points = []; %tablica na punkty exploracyjne

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


%% Przejœcie z punktów we wspó³rzêdnych obrazu binarnego na wspó³rzêdne na mapie
explo_points_xy = BinaryToMeters(explo_points, omap, BinaryMap );
%% Filtrowanie punktow lezacych za blisko przeszkód
area_check = find(CheckArea_xy(explo_points_xy, omap, size_of_checkarea) ==  0); 
explo_points_xy(area_check, :) = [];

%% Filtrowanie punktow exploracyjnych le¿acych za blisko siebie
explo_points_xy = Filter_close_points(explo_points_xy, omap, min_dist_between_exPoints, size_of_checkarea);

%% Sprawdzenie zablokowanych obszarów
explo_points_xy = Filter_blocked_points(explo_points_xy, blocked_points, size_of_blockedArea);

%% Ocena punktów eksploracyjnych
explo_points_xy = exploratory_points_rating(explo_points_xy, omap, poses(end,:)); 


