function  explo_points_filtered = kmeans_custom(explo_points)
%% OPIS FUNKCJI
% Funkcja ma za zadanie przefiltrowaæ punkty eksploracyjne ³¹cz¹æ wiêksze
% skupiska punktów znajduj¹cych siê bardzo blisko siebie w jeden punkt.
% Jest to zmodyfikowana funkcja kmeans w taki sposób aby znajdowa³a
% optymaln¹ liczbê K - czyli liczbê skupisk w zale¿noœci od sytuacji
% 
% INPUT:
%   - explo_points - punkty eksploracyjne w formacie [x y]
% OUTPUT:
%   - explo_points_filtered - przefiltrowany punkty eksploracyjne

if  isempty(explo_points)
    explo_points_filtered = [];
elseif length(explo_points(:,1)) >= 3 % funkcja dzia³a gdy istnieja trzynajmniej 3 punkty
    X = explo_points;
    
    clust = zeros(size(X,1),2);
    for i=1:2 % bo dwie kolumny
        clust(:,i) = kmeans(X,i,'emptyaction','singleton','replicate',1);
    end
    
    va = evalclusters(X,clust,'CalinskiHarabasz');
    [~,C] = kmeans(X,va.OptimalK);
    
    explo_points_filtered = C;
else
    explo_points_filtered = explo_points;
end



