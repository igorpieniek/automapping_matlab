function funs = DFSalgorithm
  funs.goBack=@goBack;
  funs.goDeep=@goDeep;
end


%%  goBack function

function [parentTochild_route, child, parentNum,targetPoint, gobackFlag, continueStatus  ]= goBack(parentTochild_route, child, parentNum )

     % Usuniecie punktow - rodzicow będących punktami powrotnymi jezeli nie posiadaja dzieci - innych galezi
     to_delete = [];
     continueStatus = false;
     for p = 1 :  length(parentTochild_route(:,1))
         semeparent_number = find(child(:,3) == parentTochild_route(p,1)); % zebranie galezi o tym samym identyfikatorze rodzica
         if isempty(semeparent_number)
             to_delete(1,end+1) = p;                                       % w przypadku braku galezi o danym identyfikatorze, zostaje zapisany nr identyfikatora
         end
     end
     to_delete = unique(to_delete);
     parentTochild_route(to_delete,:) = [];                                % usunięcie punktow o zapisanym identyfikatorze


     if  parentTochild_route(end,1) == parentNum && length(parentTochild_route(:,1))>1 %jezeli operujemy caly czas na tym samym identyfikatorze rodzica
         targetPoint = parentTochild_route(end,2:3); % wyznaczenie aktualnego celu jako ostatneigu punktu z listy punktow powrotnych
         parentTochild_route(end, :) = [];
     else
         temp = find(child(:,3) == parentNum); % zebranie punktow galezi (dzieci) o tym samym identyfikatorze rodzica dla aktualnego identyfikatora
         if ~isempty(temp)

             sameparent_points = child(temp,:);                                                                     % tworzy tymaczasową macierz dzieci posiadających tych samych rodziców
             points_withrating = [exploratory_points_rating(sameparent_points(:,1:2), explo_map, startPoint, maxLidarRange) temp]; % macierz punktów w formacie [x y rate child_index]

             [targetPoint, ~, target_num] = best_point(points_withrating(:,1:2), points_withrating(:,3));          % wyznaczenie punktu target
             child(points_withrating(target_num, 4), :) = [];                                                       % usuniecie z listy dzieci punktu target

             gobackFlag = false;                                                                                   % powrot do punktu - rodzica zostal zakonczony
         else
             parentNum = parentNum -1;                                                                              % jezeli nie ma galezi (dzieci) dla danego identyfikatora rodzica
             continueStatus = true;
         end
     end
end

%%  goDeep function

function [parentTochild_route, child, parentNum, newParentFlag, target_point, gobackFlag, continueFlag, breakStatus  ]= goDeep(parentTochild_route, ...
                                                                                                                child,...
                                                                                                                parentNum,...
                                                                                                                newParentFlag,...
                                                                                                                allPoses,...
                                                                                                                explo_map,...
                                                                                                                lastPoseNum,...
                                                                                                                middlePoints,...
                                                                                                                maxLidarRange)
    gobackFlag = false;
    breakStatus = false;
    continueFlag = false;
                                                                                                            
    if parentNum == 0                                                   % na początku gdy nie ma galezi nadpisywana jest pierwsza linijka
        parentTochild_route(end,:) =[ 0 allPoses(end,1:2)];
    elseif newParentFlag 
        parentTochild_route(end+1,:) =[ parentNum  allPoses(end,1:2)]; % dopisywany jest kolejny rodzic, ale tylko przy zwiększeniu identyfikatora rodzica
        newParentFlag = false;
    end

    % Wyznaczenie punktów eksploracyjnych dla pozycji od last_pose_num do konca pozycji
    explo_points = [];
    explo_points = exploratory_points2(explo_map, explo_points, lastPoseNum, allPoses, middlePoints, maxLidarRange,0.05 );

    % weryfikacja dzieci względem osiagnietych pozycji
    if ~isempty(child) && ~isempty(middle_Pt)
        child = verify_PointsToPosses(child, middle_Pt);
    end

    % weryfikacja punktów wzgledem osiagnietych pozycji
    if ~isempty(explo_points) && ~isempty(middle_Pt)
        explo_points = verify_PointsToPosses(explo_points,middle_Pt);
    end


    if isempty(explo_points) %jezeli po tej operacji nie ma ani punktow-dzieci ani punktow eksploracyjnych mapowanie zostaje zakonczone
        if isempty(child)
            breakStatus = true;
            return;
        else
            gobackFlag = true; % jezeli nie ma punktow eksploraycjnych ale sa punkty dzieci zostaje rozpoczeta sekwencja powrotna
            continueFlag = true;
            return;
        end
    else
        if length(explo_points(:,1)) > 1

            parentNum = parentNum +1;
            newParentFlag = true;
            [target_point, explo_points, ~] = best_point(explo_points(:,1:2), explo_points(:,3));
            child = [child ; explo_points(:,1:2) repmat(parentNum, length(explo_points(:,1)), 1)];

        else
            target_point =  explo_points(1,1:2) ;
        end
    end
end





