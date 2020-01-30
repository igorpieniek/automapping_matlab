function FiltCircle = middle_points2(explo_map_occ, last_pose)


angleIncrement = 3;
maxRange = 20; % to nie jest wazne bo i tak szukamy minimum
angles = 0: deg2rad(angleIncrement) : (2*pi - deg2rad(angleIncrement) );
interPoints = rayIntersection(explo_map_occ, last_pose, angles, maxRange);

for i = 1 : 0.5*size(interPoints)
    d(end+1,:) = norm(interPoints(i,:) - interPoints(0.5*size(interPoints)+i, :));
end
  [min_d , index] = min(d);
  
  
  FiltCircle =[mean([interPoints(index,:); interPoints(0.5*size(interPoints)+index,:) ] ), min_d/2 ];
