function isOK = CheckArea(point, BinaryMap, size_of_checkarea)
% Funkcja sprawdzaj¹ca obszar do oko³a punktu na obrazie binarnym. 
% Sprawdzany jest prostok¹tny obszar o promieniu size_of_checkarea wyrazony
% w pikselach. Je¿eli  w danym obszarze znajdzie sie jakikolwiek punkt 1 z
% obszaru binarnego fukcja zwraca fa³sz, w przeciwnym razie prawdê 


if nargin == 3 && length(point)==2 && size_of_checkarea>1
     point = round(point(1, 1:2)); 

     start_x = point(1) - size_of_checkarea;
     end_x = point(1) + size_of_checkarea;
     start_y = point(2) - size_of_checkarea;
     end_y = point(2) + size_of_checkarea;

     cutting = BinaryMap(start_y : end_y, start_x : end_x );

     if ~(sum(cutting(:))) %suma z calego wycinka  > 0 -> obiekt jest w zakresie
         isOK = true;
         return;
     else
         isOK = false;
         return;
     end
else
    error("Jeden z argumentow jest niepoprawny");
end