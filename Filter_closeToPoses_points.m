function Filter_closeToPoses_points(explo_points, Map, poses, width_of_checkarea)


% wyznaczenie wspó³czynników kierunkowych oraz wektora przesuniêcia do
% funkcji wyznaczonej przez kolejne punkty odczytanych pozycji

for pos = 1: length(poses(:,1))-1
    a = (poses(pos+1,2)-poses(pos,2)) / (poses(pos+1,1)-poses(pos,1));
    p = (a * width_of_checkarea) / (sqrt(a^2 +1));
    q = (width_of_checkarea) / (sqrt(a^2 +1));
    poses(pos,4:6 ) = [a p q];
end

 x_max = max(poses(:,1));
 x_min = min(poses(:,1));

for ex_num = 1: length(explo_points(:,1))
   if (explo_points(:,1) > x_max) ||  (explo_points(:,1) < x_min)
       continue;
   else
       belong = [];
       for pos_num = 1: length(poses(:,1))-1
         if explo_points(ex_num,1) >= poses(pos_num,1) && explo_points(ex_num,1) <= poses(pos_num+1,1)
             belong(end+1 ) = pos_num; 
%   NIE UKONCZONE
         end
       end
       
   end
   
    
end