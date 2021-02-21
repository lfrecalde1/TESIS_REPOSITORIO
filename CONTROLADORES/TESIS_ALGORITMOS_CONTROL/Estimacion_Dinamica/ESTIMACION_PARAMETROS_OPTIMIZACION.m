%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXPARAMETROS DINAMICOS DE LA PLATAFORMA MOVIL OPTIMIZACIONXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

clc,clear all,close all;
warning off
global t  U W UP  WP WREF UREF 
load('IDENTIFICACION_DATOS.mat')

UREF=uref_c;
WREF=wref_c;

u_sal=u(1:length(uref_c));
w_sal=w(1:length(uref_c));

t=0:ts:(length(UREF)-1)*ts;

U=u_sal;
W=w_sal;

UP=[0 diff(U)/ts];
WP=[0 diff(W)/ts];

up_sal=UP;
wp_sal=WP;

landa=5;%lambda

F1=tf(landa,[1 landa]);


U=lsim(F1,U,t);
W=lsim(F1,W,t);

UP=lsim(F1,UP,t);
WP=lsim(F1,WP,t);

UREF=lsim(F1,UREF,t);
WREF=lsim(F1,WREF,t);

x0=zeros(1,7)+0.5;

options = optimoptions('fminunc','Display','iter',...
    'Algorithm','quasi-newton','TolFun',1e-20,'TolX',1e-20,...
    'MaxFunEvals',1e+100,'MaxIter',1000);
[x,fval,exit,salida] = fminunc(@Dinamica_estimacion,x0,options);

x

save('DINAMICA_PLATAFORMA.mat','x')

va(1,1)=0;
va(2,1)=0;

for i=1:length(UREF)
    vref=[uref_c(i);wref_c(i)];
%     v=[uref_c(i);wref_c(i)];
 % d) Matriz de Inercia
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
    
end
%% GRAFICAS DEL SISTEMA
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
    subplot(2,1,1)
    plot(t,u_sal(1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
    plot(t,uref_c,'Color',[46,188,89]/255,'linewidth',1); hold on
    plot(t,va(1,1:length(UREF)),'Color',[83,57,217]/255,'linewidth',1); hold on
    grid on;
    grid minor;
    %title('$\textrm{Validacion}$','Interpreter','latex','FontSize',9);
    legend({'$\mu$','$\mu_{ref}$','$\mu_{m}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
    subplot(2,1,2)
    plot(t,w_sal(1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
    plot(t,wref_c,'Color',[46,188,89]/255,'linewidth',1); hold on
    plot(t,va(2,1:length(UREF)),'Color',[83,57,217]/255,'linewidth',1); hold on
    grid on
    legend({'$\dot\psi$','$\dot\psi_{ref}$','$\dot\psi_{m}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
print -dpng VALIDATION_OPTIMIZACION
print -depsc VALIDATION_OPTIMIZACION

% figure
% plot(up_sal)
% hold on
% plot(UP)
% figure
% plot(u_sal,'r')
% hold on
% plot(U,'b')
