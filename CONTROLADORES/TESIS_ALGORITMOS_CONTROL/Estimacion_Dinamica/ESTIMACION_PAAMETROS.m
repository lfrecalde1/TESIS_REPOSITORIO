%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXPARAMETROS DINAMICOS DE LA PLATAFORMA MOVILXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
clc,clear all,close all;

load('IDENTIFICACION_DATOS');
t=[1:ts:tfinal-ts];

To=ts; 
land =3; 

u=[t-1;u(1:length(t))]';
w=[t-1;w(1:length(t))]';

uref=[t-1;uref_c(1:length(t))]';
wref=[t-1;wref_c(1:length(t))]';

sim('REGRESION_LINEAL.mdl');

TF=[Tu;Tw];
YF=[Yu;Yw];

F=pinv(TF'*TF)*TF'*YF
PARAMETROS=F;
save('PARAMETROS.mat','PARAMETROS');

