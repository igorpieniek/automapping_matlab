function P_inter = F2_inter_point(P_ref, P1, P2)
% Funkcja sluzy do wyznaczenia punktu przeciecia miedzy prost¹ utworzona
% przez punkty P1 i P2, oraz prost¹ prostopad³a do tej prostej
% przechodzacej przez punkt odniesienia P_ref. Punkty zapisane w formacie
% (x, y)

if nargin == 3 
    if ~(length(P_ref)== 2 && length(P1)== 2 && length(P2)==2)
        error("Jeden z argumentów nie jest puktem")
    end
        %rozpatrywany punkt Pref(k,m), P1(e,f)
        %wspolczynnik kierunkowy dla prostej P1 - P2
        if  ( P1(1,1) - P2(1,1) ) == 0 % przypadek pionowej lini
           x =  P1(1,1);
           y = P_ref(1,2);
        elseif ( P1(1,2) - P2(1,2) ) == 0 % przypadek poziomej lini
           x =  P_ref(1,1);
           y = P1(1,2);
        else 
            a_dir =( P1(1,2) - P2(1,2) ) / ( P1(1,1) - P2(1,1) );
            % punkt przeciêcia wyznaczony z ukladu rownan dwoch
            % przecinajacych sie prostych x = (m + 1/a*k - f + ae) / a+ 1/a
            k = P_ref(1,1);
            m = P_ref(1,2);
            e = P1(1,1);
            f = P1(1,2);
            x = (m + ((1/a_dir)*k) - f + (a_dir * e))/ (a_dir + (1/a_dir));
            y = (a_dir * P_ref(1,1)) + f - (a_dir * e);

        end
        P_inter = [x y];
    
else
    error("Podano za ma³o argumentów")
end