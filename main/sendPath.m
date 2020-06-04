function sendPath(path, publisher, rosmsg)
% Funkcja przesyłająca ściezkę do sieci ROS. W przypadku użycia jako
% ścieżki pustej tablicy do sieci ROS zostanie przesłan a ramka informująca
% o potrzebie zatrzymania silników robota
% INPUT:
% - path - ścieżka zapisana jako macierz [X Y FI] gdzie FI jest w radianach
% - publisher - obiekt publish'era ROS (ros.Publisher)
% - rosmsg - obiekt klasy rosmessage - zawierajacy strukture wiadomosci

%% ZATRZYMANIE ROBOTA - PUSTA ŚCIEŻKA
if isempty(path)
    rosmsg.Vector.X = 0;
    rosmsg.Vector.Y = 0;
    rosmsg.Vector.Z = 0;
    rosmsg.Header.FrameId = "STOP";
    send(publisher, rosmsg);
    disp('MOTORS STOPPED!')
    return
    
end

%% PRZESYŁANIE ŚCIEZKI NA ROBOTA
disp('SENDING PATH')

% pierwsza wiadomosc - aktualna pozycja
rosmsg.Header.Seq = 0;
rosmsg.Header.FrameId = "POSITION";
rosmsg.Vector.X = path(1,1);
rosmsg.Vector.Y = path(1,2);
rosmsg.Vector.Z = path(1,3) ;
send(publisher, rosmsg);

% Przesył kolejnych punktów ściezki
rosmsg.Header.FrameId = "PATH";
for i = 2: length(path(:,1))
    rosmsg.Vector.X = path(i,1);
    rosmsg.Vector.Y = path(i,2);
    rosmsg.Vector.Z = path(i,3);
    send(publisher, rosmsg);
end

% Wiadomosc informująca o tym że ściezka została w całości przesłana
rosmsg.Header.FrameId = "PATH_END";
send(publisher, rosmsg);
disp('SENDING PATH..........DONE!')

