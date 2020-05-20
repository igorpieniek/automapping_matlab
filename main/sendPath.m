function sendPath(path, publisher, rosmsg, index)

if isempty(path)
    rosmsg.Vector.X = 0;
    rosmsg.Vector.Y = 0;
    rosmsg.Vector.Z = 0;
    rosmsg.Header.FrameId = "STOP";
    send(publisher, rosmsg);
    return
    
end
% pierwsza wiadomosc - aktualna pozycja
rosmsg.Header.Seq = index;
rosmsg.Header.FrameId = "POSITION";
rosmsg.Vector.X = path(1,1);
rosmsg.Vector.Y = path(1,2);
rosmsg.Vector.Z = path(1,3) ;
send(publisher, rosmsg);

% sciezka
rosmsg.Header.FrameId = "PATH";
for i = 2: length(path(:,1))
    rosmsg.Vector.X = path(i,1);
    rosmsg.Vector.Y = path(i,2);
    rosmsg.Vector.Z = path(i,3);
    send(publisher, rosmsg);
end

% koncowa wiadomosc
rosmsg.Header.FrameId = "PATH_END";
send(publisher, rosmsg);

