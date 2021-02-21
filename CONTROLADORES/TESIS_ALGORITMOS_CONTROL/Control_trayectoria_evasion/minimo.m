function [minimo_datos] = minimo(world,hx,hy)
%UNTITLED Summary of this function goes here

mundo=world';
h=[hx;hy];
[i,j]=size(mundo);
for k=1:j
    d(k)=norm(mundo(:,k)-h);
    
end
[i_min,j_min]=min(d);
minimo_datos=mundo(:,j_min);

end

