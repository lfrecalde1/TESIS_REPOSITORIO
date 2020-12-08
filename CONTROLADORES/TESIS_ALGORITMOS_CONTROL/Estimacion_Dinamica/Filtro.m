function w_filtrada = Filtro(w,w_1)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes 
variacion=abs(w-w_1);

if variacion>5
    w_filtrada=w_1;
    
else
    w_filtrada=w;
end

end

