%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXCONTROL DE TRAYECTORIA DE UNA PLATAFORMA MOVILXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

%% PARAMETROS DE TIEMPO
clc,clear all,close all;
ts=0.1;
tf=30;
to=0;
t=[to:ts:tf];

%% CARGAR DATOS DE LA DINAMICA DEL ROBOT MOVIL
load('DINAMICA_PLATAFORMA');
PARAMETROS=x;

%% INICIALIZACION DE LA COMUNICACION CON ROS
rosshutdown
setenv('ROS_MASTER_URI','http://192.168.0.104:11311');
setenv('ROS_IP','192.168.0.102');
rosinit

%% ENLACE A LOS TOPICOS DE ROS NECESARIOS
robot = rospublisher('/cmd_vel');
velmsg = rosmessage(robot);
odom = rossubscriber('/odom');

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
hxd=0.4*cos(0.3*t);
hyd=0.4*sin(0.3*t);

hxdp=-0.4*0.3*sin(0.3*t);
hydp=0.4*0.3*cos(0.3*t);

%% GANANCIA PARA LOS ACTUADORES
K2=1; 

%% DATOS DE LA RED NEURONAL 1
xite_1=0.5;
alfa_1=0.1;
N1=6;
wjo=rands(N1,1);
wjo_1=wjo;wjo_2=wjo_1;
wij=rands(2,N1);
wij_1=wij;wij_2=wij;

%% DATOS DE LA RED NEURONAL 2
xite_2=0.5;
alfa_2=0.1;
N2=6;
vjo=rands(N2,1);
vjo_1=vjo;vjo_2=vjo_1;
vij=rands(2,N2);
vij_1=vij;vij_2=vij;

%% DATOS DE LA RED NEURONAL 3
xite_3=0.5;
alfa_3=0.1;
N3=6;
zjo=rands(N3,1);
zjo_1=zjo;zjo_2=zjo_1;
zij=rands(2,N3);
zij_1=zij;zij_2=zij;

%% DATOS DEL CONTROLADOR NEURONAL ADAPTATIVO VALORES INICIALES
chi(1,1)=0.5;
chi(2,1)=0.5;


%% BUCLE DE SIMULACION 
for k=1:length(t)
    tic;
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
    KP=[chi(:,k)];
    
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
    v=[uref_c(k) wref_c(k)]';
    
    %% COMPENSACION DINAMICA PLATAFORMA MOVIL
    Dinamica = COMPENSACION_DINAMICA_PLATAFORMA_MOVIL_N(vrefp,vref_e,v,q,ts,PARAMETROS);
     
    %% DINAMICA DE LA PLATAFORMA MOVIL
    uref(k)=Dinamica(1);
    wref(k)=Dinamica(2);
     
    %% ENVIO DE DATOS AL ROBOT
    velmsg.Linear.X =uref(k);
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
    
    %% ESTIMACION DE LOS JACOBIANOS DEL SISTEMA 
    [hx_estimado(k+1),JACOBIANO_X,wjo,wjo_1,wjo_2,wij,wij_1,wij_2] = NEURONAL_LINEA(hx(k+1),uref_c(k),wref_c(k),N1,xite_1,alfa_1,wjo,wij,wjo_1,wij_1,wjo_2,wij_2);
    [hy_estimado(k+1),JACOBIANO_Y,vjo,vjo_1,vjo_2,vij,vij_1,vij_2] = NEURONAL_LINEA(hy(k+1),uref_c(k),wref_c(k),N2,xite_2,alfa_2,vjo,vij,vjo_1,vij_1,vjo_2,vij_2);
    [phi_estimado(k+1),JACOBIANO_PHI,zjo,zjo_1,zjo_2,zij,zij_1,zij_2] = NEURONAL_LINEA(phi(k+1),uref_c(k),wref_c(k),N3,xite_3,alfa_3,zjo,zij,zjo_1,zij_1,zjo_2,zij_2);
    ACTUALIZACION=NEURONAL_ADAPTATIVO(JACOBIANO_X,JACOBIANO_Y,h,hd,phi(k+1),a);
    chi(:,k+1)=chi(:,k)-ACTUALIZACION;
    
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
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(hxd,hyd,'--','Color',[56,171,217]/255,'linewidth',1.5); hold on;
plot(hx,hy,'Color',[32,185,29]/255,'linewidth',1.5); hold on;
grid on;
grid minor;
legend({'$\mathbf{\eta}_{p_{des}}$','$\mathbf{\eta}_{p}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Trayectoria Deseada y Trayectoria Descrita}$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9)

print -dpng SIMULATION_1_NEURONAL
print -depsc SIMULATION_1_NEURONAL

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

    
print -dpng CONTROL_ERRORS_1_NEURONAL
print -depsc CONTROL_ERRORS_1_NEURONAL

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

print -dpng CONTROL_VALUES_1_NEURONAL
print -depsc CONTROL_VALUES_1_NEURONAL

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
    plot(t,hx(1:length(t)),'Color',[217,204,30]/255,'linewidth',1); hold on;
    plot(t,hx_estimado(1:length(t)),'--','Color',[83,57,217]/255,'linewidth',1); hold on
    grid on;
    grid minor;
    title('$\textrm{Posicion y Orientacion Estimacion}$','Interpreter','latex','FontSize',9);
    legend({'$x$','$\bar{x}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    ylabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
    plot(t,hy(1:length(t)),'Color',[56,171,217]/255,'linewidth',1); hold on;
    plot(t,hy_estimado(1:length(t)),'--','Color',[223,67,85]/255,'linewidth',1); hold on
    grid on;
    grid minor;
    %title('$\textrm{Position Estimator Performance}$','Interpreter','latex','FontSize',9);
    legend({'$y$','$\bar{y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);
    
subplot(3,1,3)
    plot(t,phi(1:length(t)),'Color',[32,185,29]/255,'linewidth',1); hold on;
    plot(t,phi_estimado(1:length(t)),'--','Color',[56,171,217]/255,'linewidth',1); hold on
    grid on;
    grid minor;
    %title('$\textrm{Position Estimator Performance}$','Interpreter','latex','FontSize',9);
    legend({'$\psi$','$\bar{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    xlabel('$\textrm{Time }[s]$','Interpreter','latex','FontSize',9)
    ylabel('$\psi[rad]$','Interpreter','latex','FontSize',9);
    
print -dpng ESTIMACIONES
print -depsc ESTIMACIONES

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
plot(t,chi(1,1:length(t)),'Color',[83,57,217]/255,'linewidth',1);
grid on;
title('$\textrm{Evolucion de las Ganancias}$','Interpreter','latex','FontSize',9);
legend({'$K_{x}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$K_x$','Interpreter','latex','FontSize',9);

subplot(2,1,2)
plot(t,chi(2,1:length(t)),'Color',[56,171,217]/255,'linewidth',1.5);
grid on;
grid on;
%title('$\textrm{Adaptative Gains Evolution}$','Interpreter','latex','FontSize',9);
legend({'$K_{y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$K_y$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time }[s]$','Interpreter','latex','FontSize',9)

print -dpng PARAMETROS_ADAPTATIVOS
print -depsc PARAMETROS_ADAPTATIVOS