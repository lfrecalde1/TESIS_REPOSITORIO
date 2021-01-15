%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXCONTROL DE TRAYECTORIA DE UNA PLATAFORMA MOVILXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

%% PARAMETROS DE TIEMPO
clc,clear all,close all;
ts=0.05;
tf=20;
to=0;
t=[to:ts:tf];

%% CARGAR DATOS DE LA DINAMICA DEL ROBOT MOVIL
load('DINAMICA_PLATAFORMA');
PARAMETROS=x;

%% INICIALIZACION DE LA COMUNICACION CON ROS
rosshutdown
setenv('ROS_MASTER_URI','http://192.168.0.104:11311');
setenv('ROS_IP','192.168.0.103');
rosinit

%% ENLACE A LOS TOPICOS DE ROS NECESARIOS
robot = rospublisher('/cmd_vel');
velmsg = rosmessage(robot);
odom = rossubscriber('/odom');
lidarSub = rossubscriber('/scan');

%% LECTURA DEL ROBOT PARA CONDICIONES INICIALES
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
vel=odomdata.Twist.Twist;
quat = pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);

%% DISTANCIA HACIA EL PUNTO DE INTERES
a=0.1;

%% POSICIONES INICIALES 
hx(1)=pose.Position.X;
hy(1)=pose.Position.Y;
phi(1)=angles(1);

%% VELOCIDADES DE CONTROL INICIALES REALES
u(1) = vel.Linear.X;
w(1) = vel.Angular.Z;

%% CINEMATICA DIRECTA 
hx(1)=hx(1)+a*cos(phi(1));
hy(1)=hy(1)+a*sin(phi(1));

%% VELOCIDADES EN CADA EJE DEL ROBOT INICIALES
hxp(1)=u(1)*cos(phi(1))-a*w(1)*sin(phi(1));
hyp(1)=u(1)*sin(phi(1))+a*w(1)*cos(phi(1));

%% TRAYECTORIA DESEADAS
hxd=0.3*cos(0.3*t);
hyd=0.3*sin(0.3*t);

hxdp=[0 diff(hxd)/ts];
hydp=[0 diff(hyd)/ts];
%% GANANCIASfigure DEL CONTROLADOR CINEMATICO
KP=1; 

%% GANANCIA PARA LOS ACTUADORES
K2=1; 

%% FUERZA DE INTERACCION CON EL OBJETO 

F_1=0;
F_2=0;
xa_1=0;
xa_2=0;

%% PARAMETROS ADAPTATIVOS
chi=zeros(6,length(t));
%% BUCLE DE SIMULACION 
for k=1:length(t)
    tic;
    %% TOMA DE DATOS DESDE LIDAR Y SACAR EL ANGULO FUERZA Y DISTANCIA
    %[F(k),angulo(k),d]=evasion_obstaculos(lidarSub);
    
    %% GENERACION DE IMPEDANCIA
    %[xa(k),xa_1,xa_2,F_1,F_2] = impedancia_1(F(k),F_1,F_2,xa_1,xa_2);
    
    %% ROTACION DEL PUNTO DESEADO
    %[hxd(k),hyd(k),hxdp(k),hydp(k)]=ROTACION_REFERENCIA(hxd(k),hyd(k),hxdp(k),hydp(k),xa(k));
    %% ERROR DE CONTROL
    hxe(k)=hxd(k)-hx(k);
    
    hye(k)=hyd(k)-hy(k);
    
    %% VECTORES PARA FORMA GENERAL
    hd=[hxd(k) hyd(k)]';
    
    hdp=[hxdp(k) hydp(k)]';
    
    h =[hx(k),hy(k)]';
    
    hp=[hxp(k),hyp(k)]';
    
    q=[0 phi(k)]'; 
    
    %% CONTROLADOR CINEMATICO
    qref=PROPORCIONAL(KP,K2,hd,hdp,h,q,a);
     
    %% VELOCIDADES CINEMATICAS O VELOCIDADES DESEADAS PARA EL BLOQUE DE COMPENSACIÓN DIN�?MICA
    uref_c(k)=qref(1);
    wref_c(k)=qref(2);
    
    %% ERRORES DE VELOCIDAD VREF
    ue(k)=uref_c(k)-u(k);
    we(k)=wref_c(k)-w(k);
    vref_e=[ue(k) we(k)]';
    
    %% DERIVADAS DE LAS VREF DESEADAS
    vrefp_u=diff([uref_c uref_c(end)])/ts;
    vrefp_w=diff([wref_c wref_c(end)])/ts;
    vrefp=[vrefp_u(k) vrefp_w(k)]';
    vd=[uref_c(k) wref_c(k)]';
    v=[u(k) w(k)];
    %% COMPENSACION DINAMICA PLATAFORMA MOVIL
%     Dinamica = COMPENSACION_DINAMICA_PLATAFORMA_MOVIL_N(vrefp,vref_e,vd,q,ts,PARAMETROS);
    Dinamica=Adaptativo_1(vrefp,vref_e,v,vd,chi(:,k),ts);
    %% DINAMICA DE LA PLATAFORMA MOVIL
    uref(k)=Dinamica(1);
    wref(k)=Dinamica(2);
    chi(:,k+1)=Dinamica(3:8,1);

    %% ENVIO DE DATOS AL ROBOT
    velmsg.Linear.X = uref(k);
    velmsg.Angular.Z =wref(k);
    send(robot,velmsg);
    
    %% LECTURA DE LAS POSICIONES DEL ROBOT
    odomdata = receive(odom,3);
    pose = odomdata.Pose.Pose;
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    
    %% LECTURA DE LAS VELOCIDADES DE CONTROL REALES DEL ROBOT
    vel=odomdata.Twist.Twist;
    
    %% VELOCIDADES DEL ROBOT
    u(k+1)=vel.Linear.X;
    
    w(k+1)=Filtro(vel.Angular.Z,w(k));
   
    %% CINEMATICA DIRECTA
    phi(k+1)=angles(1);
     
    hxp(k+1)=u(k+1)*cos(phi(k+1))-a*w(k+1)*sin(phi(k+1));
    hyp(k+1)=u(k+1)*sin(phi(k+1))+a*w(k+1)*cos(phi(k+1));
     
    hx(k+1)=pose.Position.X+a*cos(phi(k+1));
    hy(k+1)=pose.Position.Y+a*sin(phi(k+1));
    
    while(toc<ts)
    end
    t_sample(k)=toc;

end
%% DETENER A LA PALTAFORMA MOVIL
velmsg.Linear.X = 0;
velmsg.Angular.Z = 0;
send(robot,velmsg);
rosshutdown;

%% GRAFICAS DEL SISTEMA
% figure
% plot(t,F,'-r')
% hold on 
% grid on
% plot(t,xa,'-g')
% 
% figure
% plot(t,angulo,'-r')
% grid on
% hold on

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(hxd,hyd,'-','Color',[56,171,217]/255,'linewidth',1.5); hold on;
plot(hx,hy,'Color',[32,185,29]/255,'linewidth',1.5); hold on;
grid on;
grid minor;
legend({'$\mathbf{\eta}_{p_{des}}$','$\mathbf{\eta}_{p}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Trayectoria Deseada y Trayectoria Descrita}$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9)

print -dpng SIMULATION_1_PROPORCIONAL
print -depsc SIMULATION_1_PROPORCIONAL

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
    plot(t,hxe,'Color',[226,76,44]/255,'linewidth',1); hold on;
    plot(t,hye,'Color',[46,188,89]/255,'linewidth',1); hold on;
    grid on;
    grid minor;
    legend({'$\tilde{x_{p}}$','$\tilde{y_{p}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Errores de Posicion}$','Interpreter','latex','FontSize',9);
    ylabel('$[m]$','Interpreter','latex','FontSize',9);
  
subplot(2,1,2)
    plot(t,ue,'Color',[223,67,85]/255,'linewidth',1); hold on
    plot(t,we,'Color',[56,171,217]/255,'linewidth',1); hold on
    grid on;
    grid minor;
    legend({'$\tilde\mu$','$\tilde{\dot\psi_{p}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Errores de Velocidad}$','Interpreter','latex','FontSize',9);
    xlabel('$\textrm{Tiempo}[s]$','Interpreter','latex','FontSize',9);ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);

    
print -dpng CONTROL_ERRORS_1_PROPORCIONAL
print -depsc CONTROL_ERRORS_1_PROPORCIONAL

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
    subplot(2,1,1)
    plot(t,u(1:length(t)),'Color',[216,72,178]/255,'linewidth',1); hold on
    plot(t,w(1:length(t)),'Color',[129,123,110]/255,'linewidth',1); hold on
    plot(t,uref_c,'--','Color',[216,72,178]/255,'linewidth',1); hold on
    plot(t,wref_c,'--','Color',[129,123,110]/255,'linewidth',1); hold on
    grid on;
    grid minor;
    title('$\textrm{Velocidades a la Salida del Robot y Velocidades de Control Cinematico}$','Interpreter','latex','FontSize',9);
    xlabel('$\textrm{Tiempo}[s]$','Interpreter','latex','FontSize',9);ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
    legend({'$\mu$','$\dot\psi_{p}$','$\mu_{ref_{c}}$','$\dot\psi_{p_{ref_{c}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    
    subplot(2,1,2)
    plot(t,uref,'Color',[217,204,30]/255,'linewidth',1); hold on
    plot(t,wref,'Color',[83,57,217]/255,'linewidth',1); grid on
    grid on;
    grid minor;
    legend({'$\mu_{ref}$','$\dot\psi_{p_{ref}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Velocidades de Compensacion Dinamica}$','Interpreter','latex','FontSize',9);
    xlabel('$\textrm{Tiempo }[s]$','Interpreter','latex','FontSize',9);;ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);

print -dpng CONTROL_VALUES_1_PROPORCIONAL
print -depsc CONTROL_VALUES_1_PROPORCIONAL

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t,chi(:,1:length(t)),'linewidth',1);
hold on
grid on
%plot(t(1:length(chi_real)),chi_real,'--','linewidth',1);
title('$\textrm{Adaptative Parameters}$','Interpreter','latex','FontSize',9);
legend({'$\chi_e$'},'Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Parameters}$','Interpreter','latex','FontSize',9);
print -dpng PARAMETROS
print -depsc PARAMETROS