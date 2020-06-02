function funs = DFSalgorithm
  funs.goBack=@goBack;
  funs.goDeep=@goDeep;
end


%%  goBack function

function [parentToChildRoute,...
          child,...
          parentNum,...
          target,...
          gobackFlag,...
          continueStatus  ]= goBack(parentToChildRoute,...
                                    child,...
                                    parentNum,...
                                    map,...
                                    allPoses,...
                                    maxLidarRange  )
    % Funkcja algorytmu DFS odpowiadająca za powrót do ostatniego punktu
    % rozgałęzienia - do punktu  w którym zostało odkryte wiecej niz jeden
    % punkt eksploracyjny
    % INPUT:
    % - parentTochildRoute - lista punktów po których robot może wrócic do
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
     continueStatus = false;
     target = [];
     gobackFlag = true;
     
     for p = 1 :  length(parentToChildRoute(:,1))
         semeparent_number = find(child(:,3) == parentToChildRoute(p,1)); % zebranie galezi o tym samym identyfikatorze rodzica
         if isempty(semeparent_number)
             toDelete(1,end+1) = p;                                       % w przypadku braku galezi o danym identyfikatorze, zostaje zapisany nr identyfikatora
         end
     end
     toDelete = unique(toDelete);
     parentToChildRoute(toDelete,:) = [];                                % usunięcie punktow o zapisanym identyfikatorze


     if  parentToChildRoute(end,1) == parentNum && length(parentToChildRoute(:,1))>1 %jezeli operujemy caly czas na tym samym identyfikatorze rodzica
         target = parentToChildRoute(end,2:3); % wyznaczenie aktualnego celu jako ostatneigu punktu z listy punktow powrotnych
         parentToChildRoute(end, :) = [];
     else
         temp = find(child(:,3) == parentNum); % zebranie punktow galezi (dzieci) o tym samym identyfikatorze rodzica dla aktualnego identyfikatora
         if ~isempty(temp)

             sameparent_points = child(temp,:);                                                                     % tworzy tymaczasową macierz dzieci posiadających tych samych rodziców
             points_withrating = [exploratory_points_rating(sameparent_points(:,1:2), map, allPoses(end,:), maxLidarRange) temp]; % macierz punktów w formacie [x y rate child_index]

             [target, ~, target_num] = best_point(points_withrating(:,1:2), points_withrating(:,3));          % wyznaczenie punktu target
             child(points_withrating(target_num, 4), :) = [];                                                       % usuniecie z listy dzieci punktu target

             gobackFlag = false;                                                                                   % powrot do punktu - rodzica zostal zakonczony
         else
             parentNum = parentNum -1;                                                                              % jezeli nie ma galezi (dzieci) dla danego identyfikatora rodzica
             continueStatus = true;
         end
     end
end

%%  goDeep function

function [parentTochildRoute,...
          child,...
          parentNum,...
          newParentFlag,...
          target,...
          gobackFlag,...
          continueStatus,...
          breakStatus  ]= goDeep(parentTochildRoute, ...
                                 child,...
                                 parentNum,...
                                 newParentFlag,...
                                 allPoses,...
                                 map,...
                                 lastPoseNum,...
                                 middlePoints,...
                                 maxLidarRange,...
                                 inflateRatio)
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
    % - newParentFlag 
    % - target - punkt wybrany za cel
    % - gobackFlag - zwracene true gdy nie zostaną odnalezione żadne nowe
    % punty w aktualnie rozpatrywanej gałęzi (podjeta decyzja o powrocie do 
    % "rodzica" - wskazówka do przejscia do funkcji goback)
    % - continueStatus - zwraca true gdy główna pętla musi byc kontynuowana
    % - breakStatus - zwraca true gdy główna pętla powinna zostać
    % przerwana, gdy proces zostanie uznany za zakończony - brak
    % dodatkowych punktów oraz brak istniejących
    % 
                       
    gobackFlag = false;
    breakStatus = false;
    continueStatus = false;
    target = [];
                                                                                                            
    if parentNum == 0                                                   % na początku gdy nie ma galezi nadpisywana jest pierwsza linijka
        parentTochildRoute(end,:) =[ 0 allPoses(end,1:2)];
    elseif newParentFlag 
        parentTochildRoute(end+1,:) =[ parentNum  allPoses(end,1:2)]; % dopisywany jest kolejny rodzic, ale tylko przy zwiększeniu identyfikatora rodzica
        newParentFlag = false;
    end

    % Wyznaczenie punktów eksploracyjnych dla pozycji od last_pose_num do konca pozycji
    explo_points = [];
    disp("Exploratory points search START!");
    explo_points = exploratory_points2(map, explo_points, lastPoseNum, allPoses, middlePoints, maxLidarRange,inflateRatio );

    % weryfikacja dzieci względem osiagnietych pozycji
    if ~isempty(child) && ~isempty(middlePoints)
        child = verify_PointsToPosses(child, middlePoints);
    end

    % weryfikacja punktów wzgledem osiagnietych pozycji
    if ~isempty(explo_points) && ~isempty(middlePoints)
        explo_points = verify_PointsToPosses(explo_points,middlePoints);
    end


    if isempty(explo_points) %jezeli po tej operacji nie ma ani punktow-dzieci ani punktow eksploracyjnych mapowanie zostaje zakonczone
        if isempty(child)
            breakStatus = true;
            return;
        else
            gobackFlag = true; % jezeli nie ma punktow eksploraycjnych ale sa punkty dzieci zostaje rozpoczeta sekwencja powrotna
            continueStatus = true;
            return;
        end
    else
        if length(explo_points(:,1)) > 1

            parentNum = parentNum +1;
            newParentFlag = true;
            [target, explo_points, ~] = best_point(explo_points(:,1:2), explo_points(:,3));
            child = [child ; explo_points(:,1:2) repmat(parentNum, length(explo_points(:,1)), 1)];

        else
            target =  explo_points(1,1:2) ;
        end
    end
end





