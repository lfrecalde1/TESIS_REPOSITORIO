% PROGRAMA PARA CALCULAR LA DINAMICA DE UUN ROBOT MOVIL TIPO DIFERENCIAL
clc,clear all,close all;
syms m1 m2 a tetha tetha_p r R Kpa Kpb Rpa Kpt Kpr Kdt Kdr real 
% m es la masa del robot movil
% a es la distancia al centro de masa
% tetha es el angulo de grio del robot
% tetha_p es la velocidad angular
% r radio de las ruedas
% R distancia entre las ruedas
% Kpa constante del torque
% Kpb contante electrop motriz
% Rpa resistencia electro motriz


Mp=[m1,0,-a*(m1)*sin(tetha);...
   0,m1,a*(m1)*cos(tetha);...
   -a*(m1)*sin(tetha),a*(m1)*cos(tetha),m1*(a^2+1)]
% M es la matriz de inercia 
Cp=[0,0,-a*(m1)*cos(tetha)*tetha_p;...
   0,0,-a*(m1 )*sin(tetha)*tetha_p;...
   0,0,0]
% hp_p=S_p*up
% hp_p_p=Sp_p*up+Sp*up_p

Sp=[cos(tetha),-a*sin(tetha);...
    sin(tetha),a*cos(tetha);...
    0,1]
Sp_p=[-sin(tetha)*tetha_p,-a*cos(tetha)*tetha_p;...
       cos(tetha)*tetha_p,-a*sin(tetha)*tetha_p;...
       0,0]
   
% matrices para tranformar las fuerzas del modelo dinamico a los torques
Bp=[cos(tetha)/r, cos(tetha)/r;...
    sin(tetha)/r,sin(tetha)/r;...
    R/(r),-R/(r)]
% matrices de constantes internas de los motores
Dp=[Kpa/Rpa,Kpa/Rpa;...
    Kpa/Rpa,-Kpa/Rpa]

Ep=[Kpa*Kpb/(Rpa*r),(R*Kpa*Kpb)/(2*Rpa*r);...
    Kpa*Kpb/(Rpa*r),-(R*Kpa*Kpb*R)/(2*Rpa*r)]
% matrices de los pid  para la velocidad lineal y angular
Lp=[Kpt,0;...
    0,Kpr]
Jp=[Kdt,0;...
    0,Kdr]

M=pinv(Lp)*(pinv(Dp)*pinv(Sp'*Bp)*Sp'*Mp*Sp+Jp);
MO=simplify(M)

C=pinv(Lp)*(pinv(Dp)*pinv(Sp'*Bp)*Sp'*Mp*Sp_p+pinv(Dp)*pinv(Sp'*Bp)*Sp'*Cp*Sp+pinv(Dp)*Ep+Lp);
CO=simplify(C)
