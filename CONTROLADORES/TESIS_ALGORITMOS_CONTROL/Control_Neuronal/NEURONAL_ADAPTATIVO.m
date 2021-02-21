function [SALIDA] = NEURONAL_ADAPTATIVO(JACOBIANO_X,JACOBIANO_Y,h,hd,phi,a)
he=tanh(hd-h);
sigma=1*diag([1 1]);

%NEURONAL ADAPTATIVO
%   EN ESTA SECCION SE CALCULO LA RAZON DE CAMBIOS PARA CADA CONSTANTE DEL
%   CONTROLADOR CINEMATICO
JACOBIANO=[JACOBIANO_X(1),JACOBIANO_X(2);...
           JACOBIANO_Y(1),JACOBIANO_Y(2)];
 E_d=[cos(phi)*he(1),sin(phi)*he(2);...
      -(sin(phi)*he(1))/a,(cos(phi)*he(2))/a];

 dqdw=he'*sigma*JACOBIANO*E_d;
 SALIDA=dqdw';

end

