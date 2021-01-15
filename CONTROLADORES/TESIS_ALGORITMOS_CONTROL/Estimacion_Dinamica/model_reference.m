clc,clear all,close all;
%% PARAMETROS OBTENIDOS DE LA ESTIMACION DE PARAMETROS
load('IDENTIFICACION_DATOS');
% load('PARAMETROS.mat');
load('DINAMICA_PLATAFORMA.mat');
PARAMETROS=x;
t=[to:ts:tfinal];
u_d(1) = 0;
w_d(1) = 0;
[uref_c,wref_c] = SENAL_1(t,0);

chi=[0.1180,0.0561,0.4941,0.1235,0.0474,0.0866,0.5584]';

for k=1:length(t)
     tic
     %% VECTORES PARA FORMA GENERAL
     v=[u_d(k) w_d(k)]';
     %% DINAMICA DE LA PLATAFORMA MOVIL
     
     vrefp_u=diff([uref_c uref_c(end)])/ts;
     vrefp_w=diff([wref_c wref_c(end)])/ts;
     
     vp_u=diff([u_d u_d(end)])/ts;
     vp_w=diff([w_d w_d(end)])/ts;
     
     vp=[vp_u(k) vp_w(k)]';
     
     vrefp=[vrefp_u(k) vrefp_w(k)]';
     
%      ue(k)=u_d(k)-uref_c(k);
%      
%      we(k)=w_d(k)-wref_c(k);
       
     ue(k)=uref_c(k)-u_d(k);
     
     we(k)=wref_c(k)-w_d(k);   
     
     vref_e=[ue(k) we(k)]';
     
     vd=[uref_c(k) wref_c(k)]';
     
    vref =Adaptativo_1(vrefp,vref_e,v,vd,chi(:,k),ts);
%      vref=CONTROL_DINAMICA(vrefp,vref_e,v,PARAMETROS)
%      vref =[uref_c(k) wref_c(k)]';
     controlu(k)=vref(1);
     controlw(k)=vref(2);
     chi(:,k+1)=vref(3:9,1);

     [Dinamica] = DINAMICA_STATE_SPACE(vref(1:2,1),v,ts,PARAMETROS);
     %% VELOCIDADES DEL ROBOT
     u_d(k+1)=Dinamica(1);
     w_d(k+1)=Dinamica(2);
     %% POSICIONES DEL ROBOT 
     toc
end
%% GRAFICAS DEL SISTEMA
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
    subplot(2,1,1)
%     plot(t,u(1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
    plot(t,uref_c,'.','Color',[46,188,89]/255,'linewidth',1); hold on
    plot(t,u_d(1:length(t)),'Color',[83,57,217]/255,'linewidth',1); hold on
    plot(t,controlu,'--','Color',[46,188,89]/255,'linewidth',1); hold on
    grid on
    title('$\textrm{Validation}$','Interpreter','latex','FontSize',9);
    legend({'$\mu$','$\mu_{ref}$','$\mu_{m}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
    subplot(2,1,2)
%     plot(t,w(1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
    plot(t,wref_c,'Color',[46,188,89]/255,'linewidth',1); hold on
    plot(t,w_d(1:length(t)),'Color',[83,57,217]/255,'linewidth',1); hold on
    plot(t,controlw,'--','Color',[46,188,89]/255,'linewidth',1); hold on
    grid on
    legend({'$\dot\psi$','$\dot\psi_{ref}$','$\dot\psi_{m}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);

figure
plot(t,chi(:,1:length(t)))