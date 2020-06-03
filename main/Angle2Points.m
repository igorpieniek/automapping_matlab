function angle = Angle2Points(P1,P2)
% Funkcja do pomiaru k¹ta pomiêdzy dwoma punktami wzglêdem punktu P1.
% Zwraca wartoœæ k¹ta w radianach
% INPUT:
%   - P1 - punkt referencyjny [x y]
%   - P2 - punkt do ktorego zostanie wyliczony k¹t [x y]
% OUTPUT:
%   - angle - k¹t pomiedzy punktami P1 i P2 wyrazony w radianach


if ~( nargin ==2 )
    error('Nieprawidlowa ilosc argumentow');
elseif ~( size(P1)==[1 2] )
    error('Jako argument P1 mo¿na podac tylko jeden punkt w formacie [x y]');
elseif ~( size(P2)==[1 2] )
    error('Jako argument P2 mo¿na podac tylko jeden punkt w formacie [x y]');
end

angle = atan2(P2(2)-P1(2),P2(1)-P1(1) );

%% STARA WERSJA
% if  ( P1(1,1) - P2(1,1) ) == 0 % przypadek pionowej lini x1 = x2
%     if(P2(1,2) -  P1(1,2)) >=0
%         angle = pi/2;
%     else
%         angle = -pi/2;
%     end
% elseif ( P1(1,2) - P2(1,2) ) == 0 % przypadek poziomej lini y1 = y2
%     if(P2(1,1) -  P1(1,1)) >=0
%         angle = 0;
%     else
%         angle = -pi;
%     end
% else
%         % I cwiartka
%     if (P2(1,1) -  P1(1,1)) >0 && (P2(1,2) -  P1(1,2)) >0
%         angle =atan(( P1(1,2) - P2(1,2) ) / ( P1(1,1) - P2(1,1) ));
%         % II cwiartka
%     elseif (P2(1,1) -  P1(1,1)) <0 && (P2(1,2) -  P1(1,2)) >0
%         angle =pi +atan(( P1(1,2) - P2(1,2) ) / ( P1(1,1) - P2(1,1) ));
%         % III cwiartka
%     elseif (P2(1,1) -  P1(1,1)) <0 && (P2(1,2) -  P1(1,2)) <0
%         angle = pi+atan(( P1(1,2) - P2(1,2) ) / ( P1(1,1) - P2(1,1) ));
%         % IV cwiartka
%     elseif (P2(1,1) -  P1(1,1)) >0 && (P2(1,2) -  P1(1,2)) <0
%          angle =2*pi+atan(( P1(1,2) - P2(1,2) ) / ( P1(1,1) - P2(1,1) ));
%     end
% end
