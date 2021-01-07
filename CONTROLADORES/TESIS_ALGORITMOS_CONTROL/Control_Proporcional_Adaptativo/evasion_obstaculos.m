function    [rotacion,angulo,d]=evasion_obstaculos(lidarSub)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here


%% LECTURA DE LOS DATOS DEL SENSOR
scanMsg = receive(lidarSub);

data=readCartesian(scanMsg);

x=data(:,1);
y=data(:,2);
dist=sqrt(x.^2+y.^2);

[minDist,j]=min(dist);

minimos_finales=[x(j),y(j)];
    
angulo=atan2(y(j),x(j));

%% CALCULO DE LA FUERZA FICTICIA
d=minDist;

%% DECLARACION DEL RADIO DE SEGURIDAD
r=(pi/2-abs(angulo))*(0.45/(pi/2));
if(r<0)
    r=0.3;
else
   r=r; 
end

b=-0.14;
x=r-d;

Fa=((pi/2)-abs(angulo));

if(Fa<0)
    
   Fa=0; 
   
else
    
Fa=Fa;
    
end
Fa=Fa*(1/1);
F=(1/(1+exp(-70*(x+b))))*Fa;
% 
% TOTAL=(1/(1+exp(-70*(x+b))));
% 
angulo_maximo=(pi/2);
if(angulo<angulo_maximo)&&(angulo>-angulo_maximo)
   auxiliar=(1/(1+exp(-70*(x+b))));
else
    auxiliar=0;
end
F_n=auxiliar*sin(angulo);
F_t=auxiliar*cos(angulo);

rotacion=F*sign(angulo);
end

