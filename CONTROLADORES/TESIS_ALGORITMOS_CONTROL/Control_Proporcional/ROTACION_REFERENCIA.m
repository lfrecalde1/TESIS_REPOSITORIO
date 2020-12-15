function [x,y,xp,yp] =ROTACION_REFERENCIA(xd,yd,xdp,ydp,rotacion)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


T=[cos(rotacion),sin(rotacion);...
   -sin(rotacion),cos(rotacion)];
X=T*[xd;yd];

x=X(1);
y=X(2);

XP=T*[xdp;ydp];
xp=XP(1);
yp=XP(2);

end

