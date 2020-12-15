clc,clear all,close all;
x=[-0.5:0.005:1];

b=-0.14;
alpha=[0:0.1:5];
saturacion=1./(0.5+exp(-70*(x+b)));
figure
plot(x,saturacion)
A=[x',saturacion']