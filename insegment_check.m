function in_range = insegment_check(Pcheck,P1,P2)
%Funkcja sluzy do okreslenia czy punkt Pcheck lezy na odcinku laczacym
%punkty P1 i P2
% OUTPUT : bool arg - jezeli jest false to punkt jest z poza odcinka

in_range = true;
if nargin == 3 
    if ~(length(Pcheck)== 2 && length(P1)== 2 && length(P2)==2)
        error("Jeden z argumentów nie jest puktem")
    end
            x_border1 = P1(1,1);
            x_border2 = P2(1,1);
            x_inter = Pcheck(1,1);
            y_inter = Pcheck(1,2);
            
            
            if x_border1 > x_border2
                if (x_inter > x_border1) || ((x_inter < x_border2))
                    in_range = false;
                end
            elseif x_border1  == x_border2
                y_border1 = P1(1,2);
                y_border2 = P2(1,2);
                if(y_border1 > y_border2)
                    if (y_inter > y_border1) || ((y_inter < y_border2))
                        in_range = false;
                    end
                elseif y_border1 < y_border2
                    if (y_inter > y_border2) || ((y_inter < y_border1))
                        in_range = false;
                    end
                end
            else
                if (x_inter > x_border2) || ((x_inter < x_border1))
                    in_range = false;
                end
            end
else
    error("Podano za ma³o argumentów")
end