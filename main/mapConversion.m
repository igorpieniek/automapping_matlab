function convertedMap = mapConversion(map,resolution)

    se = strel('cube',4);
    se2 = strel('cube',8);
    
    binMap = imbinarize(occupancyMatrix(map),0.5);
    binMap  = imerode(binMap ,se);
    binMap  = imdilate(binMap ,se2); 
    convertedMap = occupancyMap(binMap , resolution); %
    convertedMap.LocalOriginInWorld = map.LocalOriginInWorld;