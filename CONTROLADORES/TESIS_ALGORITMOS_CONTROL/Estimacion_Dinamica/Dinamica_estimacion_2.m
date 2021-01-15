function f=Dinamica_estimacion_2(x)

global t  U  W UP  WP WREF UREF 
% constantes faltantes
ts=0.05;
H=1*eye(2);
% ingreso de matrices
hd=[U;W];

va(1,1)=0;
va(2,1)=0;
%L = [Fun(H,hd(:,1),va(:,1))];
for i=1:length(t)
    
    u=U(i);
    w=W(i);
  
    
%     up=UP(i);
%     wp=WP(i);
    
    
    uref=UREF(i);
    wref=WREF(i);
    
    
%     v=[u;w];
    vref=[uref;wref];
    %vp=[up;wp];
    
     M11 = x(1);
     M12 = 0;
    
     M21 = 0;
     M22 = x(2);
    
     M = [M11 M12;
          M21 M22];
 
% e) Matriz de Fuerzas Centrípetas y de Coriolis
     Cs11 = x(3);
     Cs12 = x(4)+x(5)*va(2,i);
    
     Cs21 =x(6)*va(2,i);
     Cs22 =x(7);
     
     C = [Cs11 Cs12;
          Cs21 Cs22];

    %Dinamica
   vp = pinv(M)*(vref-C*va(:,i));
   va(:,i+1)=vp*ts+va(:,i);
   L(i)=(Fun(H,hd(:,i),va(:,i)));
 % d) Matriz de Inercia

    end
  %f=sqrt(F'*F); % funcion de minimizacion
 f=0.5*(sum(L)); % funcion de minimizacion
% F'
end
function [F] = Fun(H,hd,h)
    alpha=1;    %Peso de errores de posici�n
    he = hd-h;    %Errores de control
    F = alpha*he'*H*he;  
end