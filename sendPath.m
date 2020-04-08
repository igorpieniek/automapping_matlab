function sendPath(path, publisher, rosmsg, index)

send(pub_automap, rosmsg);

% pierwsza wiadomosc - aktualna pozycja
rosmsg.Header.Seq = index;
rosmsg.Header.FrameId = "POSITION";
rosmsg.Header.Vector.X = path(1,1);
rosmsg.Header.Vector.Y = path(1,2);
rosmsg.Header.Vector.Z = path(1,3);
send(publisher, rosmsg);

% sciezka
rosmsg.Header.FrameId = "PATH";
for i = 2: length(path(:,1))
    rosmsg.Header.Vector.X = path(i,1);
    rosmsg.Header.Vector.Y = path(i,2);
    rosmsg.Header.Vector.Z = path(i,3);
    send(publisher, rosmsg);
end

% koncowa wiadomosc
rosmsg.Header.FrameId = "PATH_END";
send(publisher, rosmsg);

