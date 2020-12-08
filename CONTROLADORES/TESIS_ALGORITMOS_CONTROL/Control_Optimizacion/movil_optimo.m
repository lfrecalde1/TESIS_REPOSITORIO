function [L] = movil_optimo(z,H,hd,hd1,h,q,l,ts)
%1) Definici�n de variables
    %a) Acciones de control (incognita)        
     v_1 = z;       %z=[u,w,q1_p,q2_p,q3_p,q4_p]';

    %b) Estados de control
    th = q(2);
  
    
    %c) Par�mtros del Manipulador M�vil
     a = l(1);
 

%2) Matriz Jacobiana
    j11 = cos(th);
    j12 = -a*sin(th);
   
        
    j21 = sin(th);
    j22 = +a*cos(th);

        

     
    J = [j11 j12; 
         j21 j22];
    
%2) Posiciones deseados k+1    
    hk1 = ts*J*v_1+h; 
    R=[0.2,0;0,0.2];
    error_1=hd-h;
    error=hd1-hk1;
    rate_error=error-error_1;
%3) Integraci�n del Funcional        
    L = rate_error'*H*rate_error+error'*R*error; %Trapecio
end


