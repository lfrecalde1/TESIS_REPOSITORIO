clc,clear all,close all;
t=0:0.1:10;
ts=0.1;
x(1)=5;
for k=1:length(t)
    xp=-1*x(k);
    x(k+1)=x(k)+xp*ts;
    
end
figure
plot(t,x(1:length(t)))