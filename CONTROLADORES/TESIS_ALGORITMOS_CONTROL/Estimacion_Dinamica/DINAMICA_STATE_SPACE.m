function [Dinamica] = DINAMICA_STATE_SPACE(vref,v,ts,PARAMETROS)


% a) Velocidades de la plataforma móvil 
     us = v(1);
     ws = v(2);


% b) Estados de la plataforma móvil 

     

  
% c) Parámetros Dinámicos de la plataforma movil

     C1=PARAMETROS(1);
     C2=PARAMETROS(2);
     C3=PARAMETROS(3);
     C4=PARAMETROS(4);
     C5=PARAMETROS(5);
     C6=PARAMETROS(6);
     C7=PARAMETROS(7);
     
% % % d) Matriz de Inercia
%     A=[-10,0;...
%         -0,-10];
%  
% % e) Matriz de Fuerzas Centrípetas y de Coriolis
%     B=[10,0;...
%         0,10];
%     
%     
% % % g) Modelo Dinámico Plataforma Movil
%     vp = A*v+B*vref;

     M11 = C1;
     M12 = 0;
    
     M21 = 0;
     M22 = C2;
     
   
    
     M = [M11 M12;
          M21 M22];

% % e) Matriz de Fuerzas Centrípetas y de Coriolis
     Cs11 = C3;
     Cs12 = C4+C5*ws;
    
     Cs21 =C6*ws;
     Cs22 =C7;
     
    
     
     C = [Cs11 Cs12;
          Cs21 Cs22]
%    
% e) Matriz de Fuerzas Centrípetas y de Coriolis
%      Cs11 = C3;
%      Cs12 = 0;
%     
%      Cs21 =0;
%      Cs22 =C7;
%      
%      C = [Cs11 Cs12;
%           Cs21 Cs22];
    
      
% g) Modelo Dinámico Plataforma Movil
    B=inv(M);
    A=-inv(M)*C;
    vp = inv(M)*(vref-C*v);
    v = v+vp*ts;

  
    
    v_1 = [v]';
    

    Dinamica = [v_1];
end