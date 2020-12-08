function [estimado,jacobiano,wjo,wjo_1,wjo_2,wij,wij_1,wij_2] = NEURONAL_LINEA(hx,uref_c,wref_c,N1,xite_1,alfa_1,wjo,wij,wjo_1,wij_1,wjo_2,wij_2)
%NEURONAL EN LINEA
%   ESTA FUNCION ES LA ENCARGADA DE APROXIMAR LOS ESTADOS DEL ROBOT CON
%   RESPCETO A LAS ACCIONES DE CONTROL ADEMAS CALCULO LOS JACOBIANOS LOS
%   CUALES SERAN UTILIZADOS EN EL ALGORITMO ADAPTABLE


I_1=zeros(N1,1);
Iout_1=zeros(N1,1);
FI_1=zeros(N1,1);
dwij=0*wij;
      
    y_1=hx;
    x_1=zeros(2,1);
    x_1(1)=uref_c;
    x_1(2)=wref_c;
    for j=1:1:N1
        I_1(j)=x_1'*wij(:,j);
        Iout_1(j)=1/(1+exp(-I_1(j)));
    end
    yo_1=wjo'*Iout_1; 
    e_1=y_1-yo_1;

    wjo=wjo_1+(xite_1*e_1)*Iout_1+alfa_1*(wjo_1-wjo_2);
    for j=1:1:N1
        FI_1(j)=exp(-I_1(j))/(1+exp(-I_1(j)))^2;
    end
    for i=1:1:2
    for j=1:1:N1
        dwij(i,j)=e_1*xite_1*FI_1(j)*wjo(j)*x_1(i);
    end
    end
    wij=wij_1+dwij+alfa_1*(wij_1-wij_2);

    yu_1=0;
    yw_1=0;
    for j=1:1:N1
    yu_1=yu_1+wjo(j)*wij(1,j)*FI_1(j);
    yw_1=yw_1+wjo(j)*wij(2,j)*FI_1(j);
    end

    dyu_1=yu_1;
    dyw_1=yw_1;

    jacobiano=[dyu_1 dyw_1]';
    estimado=yo_1;
wij_2=wij_1;wij_1=wij;
wjo_2=wjo_1;wjo_1=wjo;
end

