function f=Dinamica_estimacion(x)

global t  U  W UP  WP WREF UREF 
% constantes faltantes


% ingreso de matrices

F=[];
for i=1:length(t)
    
    u=U(i);
    w=W(i);
  
    
    up=UP(i);
    wp=WP(i);
    
    
    uref=UREF(i);
    wref=WREF(i);
    
    
    v=[u;w];
    vr=[uref;wref];
    vp=[up;wp];
 % d) Matriz de Inercia
     M11 = x(1);
     M12 = 0;
    
     M21 = 0;
     M22 = x(2);
    
     M = [M11 M12;
          M21 M22];
 
% e) Matriz de Fuerzas Centrípetas y de Coriolis
     Cs11 = x(3);
     Cs12 = x(4)+x(5)*w;
    
     Cs21 =x(6)*w;
     Cs22 =x(7);
     
     C = [Cs11 Cs12;
          Cs21 Cs22];

    %Dinamica
    min=vr-M*vp-C*v;
    F=[F;min];
    
    end
  %f=sqrt(F'*F); % funcion de minimizacion
 f=0.5*(F'*F); % funcion de minimizacion
% F'
end