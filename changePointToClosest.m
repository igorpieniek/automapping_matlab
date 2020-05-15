function point_out = changePointToClosest(occMap,costmap ,point)


cellStep = 3*occMap.XLocalLimits(2) / (occMap.GridSize(1));
maxRadius = 10 * cellStep;

hold on
plot(point(1,1),point(1,2),'*k' )
angleRes = deg2rad(20);
if checkOccupied(costmap, point(1,1:2))
    for i = cellStep : cellStep : maxRadius
        for angle = 0: angleRes: 2*pi
            x = point(1,1) + i*cos(angle);
            y = point(1,2) + i*sin(angle);
            if checkFree(costmap, [x,y])
                hold on
                plot(x, y, '*g')
                disp(["Found free point:", num2str([x,y]),"on dist", num2str(i),"and angle",num2str(rad2deg(angle))]);
                point_out=[x,y,point(1,3)];
                return
            else
                hold on
                plot(x, y, '*m')
            end
        end
    end
   point_out=[];
   return
   
else 
    point_out = point;
    return 
end