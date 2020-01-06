function isOK = CheckArea_xy(point, Map, size_of_checkarea)

if nargin == 3 && length(point(1,:))==2 
    isOK = [];
    for k = 1:length(point(:,1))
     
        StartPt_CheckArea = [point(k,1)-(size_of_checkarea/2)  point(k,2)-(size_of_checkarea/2)];

         if ~sum( imbinarize(checkOccupancy( Map, StartPt_CheckArea,[size_of_checkarea size_of_checkarea] ) ), 'all')%suma z calego wycinka  > 0 -> obiekt jest w zakresie
             isOK(1, end+1) = true;
            
         else
             isOK(1, end+1) = false;

         end
    end
else
    error("Jeden z argumentow jest niepoprawny");
end
