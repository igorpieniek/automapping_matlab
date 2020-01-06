function xy_points = BinaryToMeters(bin_points, mapObj, BinaryMap )
% Funkcja s³uzaca do zamiany uzyskanych wspolrzednych na mapie binarnej na
% wspo³rzedne na mapie w metrach

if nargin==3
    map_height = abs(mapObj.YWorldLimits(2) - mapObj.YWorldLimits(1));
    MToBin_rate = (length(BinaryMap(:,1))) / map_height; % wspolczynik przez ktora trzeba pomnozyc dlugosc na mapie zeby dostac dlugosc na obrazie
    y0 = abs(mapObj.YWorldLimits(2)) * MToBin_rate;
    x0 = abs(mapObj.XWorldLimits(1)) * MToBin_rate;


    xy_points(:,1) =   (bin_points(:, 1) - x0) / MToBin_rate;
    xy_points(:,2) = - (bin_points(:, 2) - y0) / MToBin_rate;
else
    error("ZA MA£O ARGUMENTÓW!");
end