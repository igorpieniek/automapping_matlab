classdef  DFSalgorithm
   properties
      parentToChildRoute
      parentNum
      newParentFlag
      goBackFlag
      maxLidarRange  
      inflateRatio
      allExploPoints
   end
   methods
     % CONSTRUCTOR 
     function obj = DFSalgorithm(startPoint, maxLidarRange,exploratoryInflateRatio )
        if nargin == 3
            disp('DFSalgorithm object init DONE!')
            obj.maxLidarRange = maxLidarRange;
            obj.inflateRatio = exploratoryInflateRatio;
            obj.parentToChildRoute = [];
            obj.parentToChildRoute = [0 startPoint(end,1:2)];
            obj.parentNum = 0;
            obj.allExploPoints = [];
            obj.goBackFlag = false;
            obj.newParentFlag = false;
        else
            error('Wrong number of init variables! ')
        end
     end
     function output = process(obj, map, allPoses ,middlePoints)        
        while true
            if obj.goBackFlag
                 output = obj.goBack( map, allPoses);                 
                 if ~obj.goBackFlag
                     break;
                 end                 
            else
                 output = obj.goDeep( map, allPoses ,middlePoints);
                 if obj.goBackFlag
                     continue;
                 else
                     break;
                 end
            end
        end
         
     end
        function output = goBack(obj,map,allPoses)
            % Funkcja algorytmu DFS odpowiadająca za powrót do ostatniego punktu
            % rozgałęzienia - do punktu  w którym zostało odkryte wiecej niz jeden
            % punkt eksploracyjny
            % INPUT:
            % - parentToobj.allExploPointsRoute - lista punktów po których robot może wrócic do
            % ostatniego rozgałęzienia
            % - child - lista wszystkich dostepnych punktów eksploracyjnych
            % - parentNum - unikalny identyfikator rodzica danej galezi 
            % - map - occupancyMap
            % - allPoses - wszystkie zarejestrowane pozycje
            % - maxLidarRange - w metrach
            %
            % OUTPUTS:
            % - parentTochildRoute 
            % - child 
            % - parentNum 
            % - target - punkt wybrany za cel
            % - gobackFlag - zwracene true gdy nie zostaną odnalezione żadne nowe
            % punty w aktualnie rozpatrywanej gałęzi (podjeta decyzja o powrocie do 
            % "rodzica" - wskazówka do przejscia do funkcji goback)
            % - continueStatus - zwraca true gdy główna pętla musi byc kontynuowana

             toDelete = [];
             output.target = [];
             output.exploPoints = obj.allExploPoints;
             
             while true  
                 for p = 1 :  length(obj.parentToChildRoute(:,1))
                     semeparent_number = find(obj.allExploPoints(:,3) == obj.parentToChildRoute(p,1)); % zebranie galezi o tym samym identyfikatorze rodzica
                     if isempty(semeparent_number)
                         toDelete(1,end+1) = p;                                       % w przypadku braku galezi o danym identyfikatorze, zostaje zapisany nr identyfikatora
                     end
                 end
                 toDelete = unique(toDelete);
                 obj.parentToChildRoute(toDelete,:) = [];                                % usunięcie punktow o zapisanym identyfikatorze


                 if  obj.parentToChildRoute(end,1) == obj.parentNum && length(obj.parentToChildRoute(:,1))>1 %jezeli operujemy caly czas na tym samym identyfikatorze rodzica
                     output.target = obj.parentToChildRoute(end,2:3); % wyznaczenie aktualnego celu jako ostatneigu punktu z listy punktow powrotnych
                     obj.parentToChildRoute(end, :) = [];
                 else
                     temp = find(obj.allExploPoints(:,3) == obj.parentNum); % zebranie punktow galezi (dzieci) o tym samym identyfikatorze rodzica dla aktualnego identyfikatora
                     if ~isempty(temp)

                         sameparent_points = obj.allExploPoints(temp,:);                                                                     % tworzy tymaczasową macierz dzieci posiadających tych samych rodziców
                         points_withrating = [exploratory_points_rating(sameparent_points(:,1:2), map, allPoses(end,:), obj.maxLidarRange) temp]; % macierz punktów w formacie [x y rate child_index]

                         [output.target, ~, target_num] = best_point(points_withrating(:,1:2), points_withrating(:,3));          % wyznaczenie punktu target
                         obj.allExploPoints(points_withrating(target_num, 4), :) = [];                                                       % usuniecie z listy dzieci punktu target
                        
                         obj.goBackFlag = false;                                                                                   % powrot do punktu - rodzica zostal zakonczony                   
                         output.exploPoints =  obj.allExploPoints;
                         return;
                     else
                         obj.parentNum = obj.parentNum - 1;                                                                              % jezeli nie ma galezi (dzieci) dla danego identyfikatora rodzica
                         continue;
                     end
                 end
             end
        end     
        function output = goDeep(obj, map, allPoses ,middlePoints)
            % Funkcja algorytmu DFS odpowiadająca ja wykrywanie kolejnych punktów eksploracyjnych, filtrację
            % oraz wybór najbardziej optymalnego punktu. Funkcja do cyklicznego
            % wywoływania - zwraca pod argumentem gobackFlag true jeżeli nie
            % zostały znalezione nowe punkty eksploracyjne (lub zostały
            % odfiltrowane) i została podjęta decyja o powrocie do ostatniego
            % odnalezionego rozgałęzienia (ostatniego miejsca w którym odnaleziono wiele punktów eksploracyjnych)
            % INPUT:
            % - parentTochildRoute - lista punktów po których robot może wrócic do
            % ostatniego rozgałęzienia
            % - child - lista wszystkich dostepnych punktów eksploracyjnych
            % - parentNum - unikalny identyfikator rodzica danej galezi 
            % - newParentFlag - flaga podnoszona przy odnalezieniu rozgalezienia 
            % (tj wiecej niż jeden punkt eksploracyjny odnaleziony dla jednej pozycji)
            % - allPoses - wszystkie zarejestrowane pozycje
            % - map - occupancyMap
            % - lastPoseNum - nr ostatniej rozpatrywanej pozycji robota - tak aby
            % można było rozpatrywać punkty od lastPoseNum do ostatniej względem
            % allPoses
            % - middlePoints - punkty zwrócone przez funkcje middle_points2 (do filtracji)
            % - maxLidarRange - w metrach
            % - inflateRatio - współczynnik do funkcji inflate uzytej w funkcji
            % wyszukującej punkty eksploracyjne exploratory_points2
            %
            % OUTPUTS:
            % - parentTochildRoute 
            % - child 
            % - parentNum 
            % - obj.newParentFlag 
            % - target - punkt wybrany za cel
            % - gobackFlag - zwracene true gdy nie zostaną odnalezione żadne nowe
            % punty w aktualnie rozpatrywanej gałęzi (podjeta decyzja o powrocie do 
            % "rodzica" - wskazówka do przejscia do funkcji goback)
            % - continueStatus - zwraca true gdy główna pętla musi byc kontynuowana
            % - breakStatus - zwraca true gdy główna pętla powinna zostać
            % przerwana, gdy proces zostanie uznany za zakończony - brak
            % dodatkowych punktów oraz brak istniejących
            % 

            output.endStatus = false;
            output.target = [];
            output.exploPoints = obj.allExploPoints;

            if obj.parentNum == 0                                                   % na początku gdy nie ma galezi nadpisywana jest pierwsza linijka
                obj.parentToChildRoute(end,:) =[ 0 allPoses(end,1:2)];
            elseif obj.newParentFlag 
                obj.parentToChildRoute(end+1,:) =[ obj.parentNum  allPoses(end,1:2)]; % dopisywany jest kolejny rodzic, ale tylko przy zwiększeniu identyfikatora rodzica
                obj.newParentFlag = false;
            end

            % Wyznaczenie punktów eksploracyjnych dla pozycji od last_pose_num do konca pozycji
            explo_points = [];
            disp("Exploratory points search START!");
            explo_points = exploratory_points2(map, allPoses, obj.maxLidarRange,obj.inflateRatio );

            % weryfikacja dzieci względem osiagnietych pozycji
            if ~isempty(obj.allExploPoints) && ~isempty(middlePoints)
                obj.allExploPoints = verify_PointsToPosses(obj.allExploPoints, middlePoints);
            end

            % weryfikacja punktów wzgledem osiagnietych pozycji
            if ~isempty(explo_points) && ~isempty(middlePoints)
                explo_points = verify_PointsToPosses(explo_points, middlePoints);
            end


            if isempty(explo_points) %jezeli po tej operacji nie ma ani punktow-dzieci ani punktow eksploracyjnych mapowanie zostaje zakonczone
                output.exploPoints = obj.allExploPoints;
                if ~isempty(obj.allExploPoints)                  
                    obj.goBackFlag = true; % jezeli nie ma punktow eksploraycjnych ale sa punkty dzieci zostaje rozpoczeta sekwencja powrotna
                end
            else
                if length(explo_points(:,1)) > 1

                    obj.parentNum = obj.parentNum +1;
                    obj.newParentFlag = true;
                    [output.target, explo_points, ~] = best_point(explo_points(:,1:2), explo_points(:,3));
                    obj.allExploPoints = [obj.allExploPoints ; explo_points(:,1:2) repmat(obj.parentNum, length(explo_points(:,1)), 1)];

                else
                    output.target =  explo_points(1,1:2) ;
                end
            end
        end
   end
end





% %%  goBack function
% 
% function [parentToChildRoute,...
%           child,...
%           parentNum,...
%           target,...
%           gobackFlag,...
%           continueStatus  ]= goBack(parentToChildRoute,...
%                                     child,...
%                                     parentNum,...
%                                     map,...
%                                     allPoses,...
%                                     maxLidarRange  )
%     % Funkcja algorytmu DFS odpowiadająca za powrót do ostatniego punktu
%     % rozgałęzienia - do punktu  w którym zostało odkryte wiecej niz jeden
%     % punkt eksploracyjny
%     % INPUT:
%     % - parentTochildRoute - lista punktów po których robot może wrócic do
%     % ostatniego rozgałęzienia
%     % - child - lista wszystkich dostepnych punktów eksploracyjnych
%     % - parentNum - unikalny identyfikator rodzica danej galezi 
%     % - map - occupancyMap
%     % - allPoses - wszystkie zarejestrowane pozycje
%     % - maxLidarRange - w metrach
%     %
%     % OUTPUTS:
%     % - parentTochildRoute 
%     % - child 
%     % - parentNum 
%     % - target - punkt wybrany za cel
%     % - gobackFlag - zwracene true gdy nie zostaną odnalezione żadne nowe
%     % punty w aktualnie rozpatrywanej gałęzi (podjeta decyzja o powrocie do 
%     % "rodzica" - wskazówka do przejscia do funkcji goback)
%     % - continueStatus - zwraca true gdy główna pętla musi byc kontynuowana
%     
%      toDelete = [];
%      continueStatus = false;
%      target = [];
%      gobackFlag = true;
%      
%      for p = 1 :  length(parentToChildRoute(:,1))
%          semeparent_number = find(child(:,3) == parentToChildRoute(p,1)); % zebranie galezi o tym samym identyfikatorze rodzica
%          if isempty(semeparent_number)
%              toDelete(1,end+1) = p;                                       % w przypadku braku galezi o danym identyfikatorze, zostaje zapisany nr identyfikatora
%          end
%      end
%      toDelete = unique(toDelete);
%      parentToChildRoute(toDelete,:) = [];                                % usunięcie punktow o zapisanym identyfikatorze
% 
% 
%      if  parentToChildRoute(end,1) == parentNum && length(parentToChildRoute(:,1))>1 %jezeli operujemy caly czas na tym samym identyfikatorze rodzica
%          target = parentToChildRoute(end,2:3); % wyznaczenie aktualnego celu jako ostatneigu punktu z listy punktow powrotnych
%          parentToChildRoute(end, :) = [];
%      else
%          temp = find(child(:,3) == parentNum); % zebranie punktow galezi (dzieci) o tym samym identyfikatorze rodzica dla aktualnego identyfikatora
%          if ~isempty(temp)
% 
%              sameparent_points = child(temp,:);                                                                     % tworzy tymaczasową macierz dzieci posiadających tych samych rodziców
%              points_withrating = [exploratory_points_rating(sameparent_points(:,1:2), map, allPoses(end,:), maxLidarRange) temp]; % macierz punktów w formacie [x y rate child_index]
% 
%              [target, ~, target_num] = best_point(points_withrating(:,1:2), points_withrating(:,3));          % wyznaczenie punktu target
%              child(points_withrating(target_num, 4), :) = [];                                                       % usuniecie z listy dzieci punktu target
% 
%              gobackFlag = false;                                                                                   % powrot do punktu - rodzica zostal zakonczony
%          else
%              parentNum = parentNum -1;                                                                              % jezeli nie ma galezi (dzieci) dla danego identyfikatora rodzica
%              continueStatus = true;
%          end
%      end
% end
% 
% %%  goDeep function
% 
% function [parentTochildRoute,...
%           child,...
%           parentNum,...
%           newParentFlag,...
%           target,...
%           gobackFlag,...
%           breakStatus  ]= goDeep(parentTochildRoute, ...
%                                  child,...
%                                  parentNum,...
%                                  newParentFlag,...
%                                  allPoses,...
%                                  map,...
%                                  lastPoseNum,...
%                                  middlePoints,...
%                                  maxLidarRange,...
%                                  inflateRatio)
%     % Funkcja algorytmu DFS odpowiadająca ja wykrywanie kolejnych punktów eksploracyjnych, filtrację
%     % oraz wybór najbardziej optymalnego punktu. Funkcja do cyklicznego
%     % wywoływania - zwraca pod argumentem gobackFlag true jeżeli nie
%     % zostały znalezione nowe punkty eksploracyjne (lub zostały
%     % odfiltrowane) i została podjęta decyja o powrocie do ostatniego
%     % odnalezionego rozgałęzienia (ostatniego miejsca w którym odnaleziono wiele punktów eksploracyjnych)
%     % INPUT:
%     % - parentTochildRoute - lista punktów po których robot może wrócic do
%     % ostatniego rozgałęzienia
%     % - child - lista wszystkich dostepnych punktów eksploracyjnych
%     % - parentNum - unikalny identyfikator rodzica danej galezi 
%     % - newParentFlag - flaga podnoszona przy odnalezieniu rozgalezienia 
%     % (tj wiecej niż jeden punkt eksploracyjny odnaleziony dla jednej pozycji)
%     % - allPoses - wszystkie zarejestrowane pozycje
%     % - map - occupancyMap
%     % - lastPoseNum - nr ostatniej rozpatrywanej pozycji robota - tak aby
%     % można było rozpatrywać punkty od lastPoseNum do ostatniej względem
%     % allPoses
%     % - middlePoints - punkty zwrócone przez funkcje middle_points2 (do filtracji)
%     % - maxLidarRange - w metrach
%     % - inflateRatio - współczynnik do funkcji inflate uzytej w funkcji
%     % wyszukującej punkty eksploracyjne exploratory_points2
%     %
%     % OUTPUTS:
%     % - parentTochildRoute 
%     % - child 
%     % - parentNum 
%     % - newParentFlag 
%     % - target - punkt wybrany za cel
%     % - gobackFlag - zwracene true gdy nie zostaną odnalezione żadne nowe
%     % punty w aktualnie rozpatrywanej gałęzi (podjeta decyzja o powrocie do 
%     % "rodzica" - wskazówka do przejscia do funkcji goback)
%     % - continueStatus - zwraca true gdy główna pętla musi byc kontynuowana
%     % - breakStatus - zwraca true gdy główna pętla powinna zostać
%     % przerwana, gdy proces zostanie uznany za zakończony - brak
%     % dodatkowych punktów oraz brak istniejących
%     % 
%                        
%     gobackFlag = false;
%     breakStatus = false;
%     target = [];
%                                                                                                             
%     if parentNum == 0                                                   % na początku gdy nie ma galezi nadpisywana jest pierwsza linijka
%         parentTochildRoute(end,:) =[ 0 allPoses(end,1:2)];
%     elseif newParentFlag 
%         parentTochildRoute(end+1,:) =[ parentNum  allPoses(end,1:2)]; % dopisywany jest kolejny rodzic, ale tylko przy zwiększeniu identyfikatora rodzica
%         newParentFlag = false;
%     end
% 
%     % Wyznaczenie punktów eksploracyjnych dla pozycji od last_pose_num do konca pozycji
%     explo_points = [];
%     disp("Exploratory points search START!");
%     explo_points = exploratory_points2(map, allPoses(lastPoseNum:end, :), maxLidarRange,inflateRatio );
% 
%     % weryfikacja dzieci względem osiagnietych pozycji
%     if ~isempty(child) && ~isempty(middlePoints)
%         child = verify_PointsToPosses(child, middlePoints);
%     end
% 
%     % weryfikacja punktów wzgledem osiagnietych pozycji
%     if ~isempty(explo_points) && ~isempty(middlePoints)
%         explo_points = verify_PointsToPosses(explo_points,middlePoints);
%     end
% 
% 
%     if isempty(explo_points) %jezeli po tej operacji nie ma ani punktow-dzieci ani punktow eksploracyjnych mapowanie zostaje zakonczone
%         if isempty(child)
%             breakStatus = true;
%             return;
%         else
%             gobackFlag = true; % jezeli nie ma punktow eksploraycjnych ale sa punkty dzieci zostaje rozpoczeta sekwencja powrotna
%             return;
%         end
%     else
%         if length(explo_points(:,1)) > 1
% 
%             parentNum = parentNum +1;
%             newParentFlag = true;
%             [target, explo_points, ~] = best_point(explo_points(:,1:2), explo_points(:,3));
%             child = [child ; explo_points(:,1:2) repmat(parentNum, length(explo_points(:,1)), 1)];
% 
%         else
%             target =  explo_points(1,1:2) ;
%         end
%     end
% end





