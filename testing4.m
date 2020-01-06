% próba wyznaczenia punktów na podstawie zajêtoœci mapy z occupancy map
clc
close all
clear 
load lidardata_4.mat map1
load lidardata_4.mat map2
load lidardata_4.mat map3
load lidardata_4.mat map4
load lidardata_4.mat map5
load lidardata_4.mat map6


current_map = map3;

maxLidarRange = 6;
mapResolution = 40;

tic
[scans,poses] = scansAndPoses(current_map);
omap = buildMap(scans,poses,mapResolution,maxLidarRange);
mat = occupancyMatrix(omap);

explo_points = [];

imshow(mat)
for i = 1:length(mat(:,1))
    points = [];
    for j = 1:length(mat(1,:))
        if (mat(i, j) <0.1) && mat(i, j) >0.05
            points(end+1,:) = [j i ];
        end
    end
    
    if ~isempty(points)
        hold on
        plot(points(:,1), points(:,2), '.r');
        explo_points = [explo_points; points];
    end
end


explo_points_xy = grid2local(omap, explo_points);
%%
size_of_checkarea = 0.2;
if ~isempty(explo_points_xy)
    area_check = find(CheckArea_xy(explo_points_xy, omap, size_of_checkarea) ==  0);
    explo_points_xy(area_check, :) = [];
end

%% 3 Filtracja : K- means
if ~isempty(explo_points_xy) && length(explo_points_xy(:,1)) >= 3 % funkcja dzia³a gdy istnieja trzynajmniej 3 punkty
    
    figure
    show(omap);
    hold on
    plot(explo_points_xy(:,1),explo_points_xy(:,2),'*r')
    
    X = explo_points_xy;
    
    clust = zeros(size(X,1),2);
    for i=1:2 % bo dwie kolumny
        clust(:,i) = kmeans(X,i,'emptyaction','singleton','replicate',1);
    end
    
    va = evalclusters(X,clust,'CalinskiHarabasz');
    [~,C] = kmeans(X,va.OptimalK);
    
    explo_points = C;
    
    hold on
    plot(explo_points_xy(:,1),explo_points_xy(:,2),'*c')
end


toc
