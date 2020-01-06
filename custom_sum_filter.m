function fil_map = custom_sum_filter(map, SE_size,accept_point_num)
% Funkcja filtrujaca mape zapisana binarnie.Dla danego punktu mapy funkcja 
% sumuje obszar wokol niego oraz wartosc tego punktu. Jezeli suma jest
% wieksza od liczby zawartej w 3 argumencie, punkt zostaje zachowany 
% (podstawiona zostaje 1). W przeciwnym wypadku punkt zostaje usuniety (0).
% INPUT:
%   - map - obraz binarny
%   - SE_size - rozmiar w pikselach boku kwadratu analizowanego obszaru
%               wokó³ punktu
%   - acceps_point_num - jezeli suma bedzie > od tego arg. pod analizowany
%               punkt zostanie podstawiona 1



if nargin == 3 
    ws = zeros(length(map(:,1))+2 , length(map(1,:))+2);                        % workspace array (z brzegami do analizy)
    ws(2 :(1+length(map(:,1))), 2:(1+length(map(1,:))) ) = map;                 % przypisanie mapy
    fil_map = [];
    fil_map = map;
    fil_map(end, 1:length(fil_map(1,:))) = 0;
    fil_map(1:length(fil_map(:,1)), end) = 0;
    for w = (1+floor(SE_size/2)) : (length(ws(:,1)) -1- (floor(SE_size/2)))
        for k = (1+(floor(SE_size/2))) : (length(ws(1,:)) - 1-(floor(SE_size/2)))
            start_x = k - floor(SE_size/2);
            end_x = k + floor(SE_size/2);
            start_y = w - floor(SE_size/2);
            end_y = w + floor(SE_size/2);

            matrix = ws(start_y : end_y, start_x : end_x );

            if (sum(matrix(:)))> accept_point_num
                 fil_map(w-1,k-1) = 1;
            else
                 fil_map(w-1,k-1) = 0;
            end
        end
    end
elseif nargin == 2
    fil_map = custom_sum_filter(map, SE_size,2);
else
    error("Za ma³o argumentow!");
end