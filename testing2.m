clc
close all
clear

% load lidardata_2.mat map3
% load lidardata_2.mat map2
% load lidardata_3.mat maps
% load lidardata_3.mat maps2
% load lidardata_3.mat maps3
load lidardata_4.mat map1
load lidardata_4.mat map2
load lidardata_4.mat map3
load lidardata_4.mat map4
load lidardata_4.mat map5
load lidardata_4.mat map6


current_map = map4;
tic
maxLidarRange = 6;
mapResolution = 50;
NumOfRays = 64; % 
MaxAngle = 2*pi;
MinAngle = 0;
ray_length = 3;   % promieñ sprawdzanego okrêgu pod wzglêdem przeszkód
ClearPosition_radius = 0.7; % (w takim promieniu w okó³ pozycji zaden pnkt eksploracyny nie powinien istniec)
size_of_checkarea = 0.5; % minimalna odleg³oœæ punktu eksploracyjnego od przeszkody

PROJECT_RESULTS = true;

[scans,poses] = scansAndPoses(current_map);
omap = buildMap(scans,poses,mapResolution,maxLidarRange);
inflate(omap, 0.01);

angleResolution = 2*pi / NumOfRays;
alfa = (MinAngle:(angleResolution) :(MaxAngle-angleResolution));
inter_Pt = [];

MIN_DIST = 0.6; %  odlegosc miedzy ostatnio odnalezionymi punktami stycznosci z map¹
min_nan_length = 1;        %minimalna ilosc punktow nan aby zasz³o wyznaczanie punktu eksploracyjnego
explo_points = []; % macierz na punty eksploracyjne
test_mean = [];




    %% Wyznaczenie punktów eksploracyjncch
    for i = 1: length(poses(:,1))
        
        inter_Pt = rayIntersection(omap,[poses(i,1:2) 0], alfa , ray_length);   % wyznaczenie punktów przeciecia
        for j = 1: length(inter_Pt(:,1))
            inter_Pt(j,3)  = norm(inter_Pt(j,1:2) - poses(i,1:2));              % wyznaczenie odleglosci miedzy badana pozycja, a wyznacoznymi punktami
        end
        
        nonan_numbers = find(isnan(inter_Pt(:,1)) == 0);                        % wyznaczneie numerów promieni które trafiaja w przeszkode
        
        test_mean(end+1, :) =  mean(inter_Pt(nonan_numbers ,3)) ;
        
        if ~((length(nonan_numbers)) ==NumOfRays)
            for k = 1:length( nonan_numbers)-1                                  %  petla sprawdzajaca odleglosci miedzy punktami miedzy którymi promien na nic nie natrafi³
                if (nonan_numbers(k+1) - nonan_numbers(k)) > min_nan_length
                    dist = norm(inter_Pt(nonan_numbers(k),1:2) - inter_Pt(nonan_numbers(k+1),1:2));
                    if dist > MIN_DIST
                        % wpisanie do tablicy punktow ekspolracyjnych
                        explo_points(end+1,:) = mean([inter_Pt(nonan_numbers(k),1:2); inter_Pt(nonan_numbers(k+1),1:2)]);
                    end
                end
            end
            
            % przypadek gdy promien nie trafia na samym pocz¹tku lub koncu
            % listy
            LastToFirst_nan_length = (nonan_numbers(1,1) - 1) + (NumOfRays - nonan_numbers(end,1)) + 1;
            if  LastToFirst_nan_length > min_nan_length
                dist = norm(inter_Pt(nonan_numbers(1),1:2) - inter_Pt(nonan_numbers(end),1:2));
                if dist > MIN_DIST
                    % wpisanie do tablicy punktow ekspolracyjnych
                    explo_points(end+1,:) = mean([inter_Pt(nonan_numbers(1),1:2); inter_Pt(nonan_numbers(end),1:2)]);
                end
            end
        end
    end
    if PROJECT_RESULTS
        figure
        show(omap)
        hold on
        plot(explo_points(:,1),explo_points(:,2),'*r')
        hold on
        viscircles(poses(end,1:2), test_mean(end));
        hold on
        viscircles(poses(end,1:2), ray_length);
        title('Wszystkie punkty eksploracyjne')
    end
    
    %% 1 Filtracja punktów : usuniecie punktów które lez¹ za blisko osiagnietych pozycji
    if ~isempty(explo_points)
        for i = 1: length(poses(:,1))
            if ~isempty(explo_points)
                delate_ex_num = [];
                for ex_num = 1: length(explo_points(:,1))
                    ex_dist = norm(poses(i,1:2)- explo_points(ex_num,:));
                    if ex_dist < ClearPosition_radius
                        delate_ex_num(end+1) = ex_num;
                    end
                end
                explo_points(delate_ex_num, :) = [];
            end
        end
    end
    if PROJECT_RESULTS
        figure
        show(omap)
        hold on
        plot(explo_points(:,1),explo_points(:,2),'*r')
        title('Bez punktów lezacych za blisko osiagnietych pozycji')
    end
    
    %% 2 Filtracja punktów le¿acych za blisko wykytych przeszkód
    if ~isempty(explo_points)
        area_check = find(CheckArea_xy(explo_points, omap, size_of_checkarea) ==  0);
        explo_points(area_check, :) = [];
    end
    
    if PROJECT_RESULTS
        figure
        show(omap)
        hold on
        plot(explo_points(:,1),explo_points(:,2),'*r')
        title('Po filtracji punktow leacych za blisko scian')
    end
    
    %% 3 Filtracja : K- means
    if ~isempty(explo_points) && length(explo_points(:,1)) >= 3 % funkcja dzia³a gdy istnieja trzynajmniej 3 punkty
        X = explo_points;
        
        clust = zeros(size(X,1),2);
        for i=1:2 % bo dwie kolumny
            clust(:,i) = kmeans(X,i,'emptyaction','singleton','replicate',1);
        end
        
        va = evalclusters(X,clust,'CalinskiHarabasz');
        [~,C] = kmeans(X,va.OptimalK);
        
        explo_points = C;
    end
    %% Wyznaczenie parametru wyjœciowego : ocena pkt exploracyjnych, wybranie najlepszego
    if ~isempty(explo_points)
        explo_points = exploratory_points_rating(explo_points, omap, poses(end,:));
        
        target_num = find(explo_points(:,3) == max(explo_points(:,3)));
        target_point =  [explo_points(target_num,1:2) , 0];
        if PROJECT_RESULTS
            figure
            show(omap)
            hold on
            plot( target_point(:,1), target_point(:,2),'ob');
            hold on
            plot(explo_points(:,1),explo_points(:,2),'*r')
            hold on
            plot(poses(end,1),poses(end,2),'ok')
            title('Po K-means + najlepszy punkt')
        end
    else
        target_point = [];
    end

toc

