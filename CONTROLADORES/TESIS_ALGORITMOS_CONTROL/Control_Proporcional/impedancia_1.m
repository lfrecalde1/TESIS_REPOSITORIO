function [xa,xa_1,xa_2,F_1,F_2] = impedancia_1(F,F_1,F_2,xa_1,xa_2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
a1=-1.038;
a2=0.2466;
b1=0.1281;
b2=0.08034;

xa=-a1*xa_1-a2*xa_2+b1*F_1+b2*F_2;

xa_2=xa_1;
xa_1=xa;

F_2=F_1;
F_1=F;

end

