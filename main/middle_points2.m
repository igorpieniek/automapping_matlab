function FiltCircle = middle_points2(explo_map_occ, last_pose, midPoints)


angleIncrement = 3;
radiusPercentage = 1.1; % procent wyznaczonego promienia - kazdy promien jest pomniejszony
maxRange = 20; % to nie jest wazne bo i tak szukamy minimum

avrgNum = 4;        % ilość pozycji z jakiej sprawdzana jest średnia
riseCorrect = 0.25; % jezeli wyznaczony promien bedzie wiekszy o tyle od średniej z poprzednich pomiarow, 
                    % dla tego kąta zostanie przypisana średnia
                    

angles = 0: deg2rad(angleIncrement) : (2*pi - deg2rad(angleIncrement) );
interPoints = rayIntersection(explo_map_occ, last_pose, angles, maxRange);
d = [];


for i = 1 : 0.5*length(interPoints(:,1))
    d(end+1,:) = norm(interPoints(i,:) - interPoints(0.5*length(interPoints(:,1))+i, :));
end

[min_d , index] = min(d);
  
  
FiltCircle =[mean([interPoints(index,:); interPoints(0.5*length(interPoints(:,1)) + index,:) ] ), radiusPercentage *min_d/2 ];
  
if ~isempty(midPoints)
      if length( midPoints( :,1) ) < avrgNum
        avrg = mean(midPoints(:,3));
      else
        avrg = mean(midPoints(end+1-avrgNum : end,3));
      end

     if FiltCircle(1,3) >= avrg + riseCorrect 
         FiltCircle(1,3) = avrg;
    %    hold on
    %    viscircles(out(:,1:2), out(:,3),'Color', 'b' );
    end
end
