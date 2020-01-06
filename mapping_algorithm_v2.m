maxLidarRange = 6;
mapResolution = 70;
margin = 0.2;

% show(slamAlg);
[scansSLAM,poses] = scansAndPoses(slamAlg);
Map = buildMap(scansSLAM,poses,mapResolution,maxLidarRange);

reserch_angles = 0:(pi()/180):(2*pi);

for pos_num = 1%:length(poses)
     intersectionPoints = rayIntersection(Map,poses(pos_num,:),reserch_angles,maxLidarRange);

     min_num = 1;
     noNANfirst= intersectionPoints(min_num,:);
     while isnan(noNANfirst)
         min_num= min_num+1;
         noNANfirst = intersectionPoints(min_num,:);
     end
     
     temp = noNANfirst;
     prevPtsLength = [];
     noNaN_points = [];
     noNaN_points(1,:) = noNANfirst(1,:); 
     while min_num < length(intersectionPoints)
        while true
          min_num = min_num+1;
          nextPt = intersectionPoints(min_num,:);
          if isnan(nextPt) 
              continue;
          else
              break;
          end
        end
        
        prevPtsLength(end+1) = norm(nextPt - temp);
        temp = nextPt;
        noNaN_points(end+1,:) = nextPt(1,:);
     end
     prevPtsLength(end+1) = norm(nextPt - noNANfirst);
     
     suspectPts = []; %podejrzane punkty - cel do budowy trajektroii
     for len_num = 1:length(prevPtsLength)
         if prevPtsLength(1,len_num)>0.8
             if len_num == length(prevPtsLength) % porownac pierwszy z ostatnim
                 suspectPts(end+1,:) = [0.5 * (noNaN_points(len_num,1) + noNaN_points(1,1)), noNaN_points(len_num,2) + noNaN_points(1,2)]; 
             else 
                 suspectPts(end+1,:) = [0.5 * (noNaN_points(len_num,1) + noNaN_points(len_num +1,1)), noNaN_points(len_num,2) + noNaN_points(len_num +1,2)]; 
             end
         end
     end
end

 figure
 show(slamAlg);
 hold on
 plot(suspectPts(:,1),suspectPts(:,2),'*r') % Intersection points
 hold on

