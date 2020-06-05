function convertedMap = mapConversion(map)
% Funkcja służąca do konwersji mapy w taki sposób, aby miejsca nieodkryte
% na mapie, były traktowane jako wolne. Bez konwersji wyznaczenie ściezki,
% oraz przemieszczenie się do tych obszarów jest niemożliwe. Mapa zostaje
% zbinaryzowana, a następnie poddana erozji oraz dylatacji.
% Dylatacja i erozja jest opcjonalna - mozna z niej nie korzystać, lecz
% poprawia ona funkcjonowanie całego procesu
% INPUT:
% - map - obiekt klasy occupancyMap
% - resolution - rozdzielczość mapy
% OUTPUT:
% - convertedMap - przekonwertowana mapa

if ~isa(map, 'occupancyMap')
    error("map - musi byc w formacie occupancyMap!");
end

    se = strel('cube',4);
    se2 = strel('cube',8);
    
    binMap = imbinarize(occupancyMatrix(map),0.5);
    
    % opcja bez wykorzystywania erozji i dylatacji działa również dobrze
    % choć czasem napotyka błędy
%     binMap  = imerode(binMap ,se);
%     binMap  = imdilate(binMap ,se2);
    
    convertedMap = occupancyMap(binMap , map.Resolution); 
    convertedMap.LocalOriginInWorld = map.LocalOriginInWorld;