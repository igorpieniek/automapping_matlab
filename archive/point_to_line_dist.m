function dist = point_to_line_dist(P_ref, P1, P2)
%Ta funkcja s³u¿y do wyznaczenia odleglosci punktu (P_ref) od prostej
%wyznaczonej przez parê punktów P1 i P2. Fukcja dziala tylko dla
%p³aszczyzny 2D. Punkt jest dwuelementow¹ tablica [x y].

if nargin == 3 
    if ~(length(P_ref)== 2 && length(P1)== 2 && length(P2)==2)
        error("Jeden z argumentów nie jest puktem")
    end
     a = P1 - P2;
     b = P_ref - P2;
     a(end+1) = 0; % to aby metoda dzialala dla 2d
     b(end+1) = 0;
     dist = norm(cross(a,b)) / norm(a);
else
    error("Podano za ma³o argumentów")
end