%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXCONTROL DE TRAYECTORIA DE UNA PLATAFORMA MOVILXXXXXXXXXXXXXXXXX
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
% angles=pose.Orientation.Z;

%% DISTANCIA HACIA EL PUNTO DE INTERES
a=0.1;
l=[a];

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

hd=[hxd;hyd];
%% RESTRICCION PARA LAS ACCIONES DE CONTROL
lb = [-0.2,-2.55]';
ub = [ 0.2, 2.55]';
z0=[u(1),w(1)]';

%% CONFIGURACION DEL METODO DE OPTIMIZACION A UTILIZAR
options = optimset('Algorithm','sqp','Display','off');

%% DEFINICION DE LOS PASOS HACIA DELANTE A PREDECIR
N=10;

%% OBJETOS SIMULADOS
% world=World(lidarSub,[hx(1),hy(1),phi(1)]);
% Obj=world;
%% BUCLE DE SIMULACION 
for k=1:length(t)-N
    tic;
    %% ERROR DE CONTROL
    hxe(k)=hxd(k)-hx(k);
    
    hye(k)=hyd(k)-hy(k);
    
    %% VECTORES PARA FORMA GENERAL   
    h =[hx(k),hy(k)]';
   
    q=[0 phi(k)]'; 
    
    v=[u(k),w(k)]';
    %% GANANCIA PARA OPTIMIZADOR
    %% GANANCIA PARA LA PARTE DE POSICION
    H=[1,0;...
        0,1];
    %% GANANCIA PARA LA PARTE DE LAS ACCIONES DE CONTROL
    R=1*eye(2);
    
    %% CONTROLADOR BASADO EN OPTIMIZACION
    f_obj = @(z) movil_optimo(z,H,R,hd,h,q,l,ts,N,k,v,PARAMETROS);
     
    qref = fmincon(f_obj,z0,[],[],[],[],lb,ub,[],options);
     
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
    vc=[uref_c(k) wref_c(k)]';
    
    %% COMPENSACION DINAMICA PLATAFORMA MOVIL
    Dinamica = COMPENSACION_DINAMICA_PLATAFORMA_MOVIL_N(vrefp,vref_e,vc,q,ts,PARAMETROS);
     
    %% DINAMICA DE LA PLATAFORMA MOVIL
    uref(k)=Dinamica(1);
    wref(k)=Dinamica(2);
     
    %% ENVIO DE DATOS AL ROBOT
    velmsg.Linear.X =uref_c(k);
    velmsg.Angular.Z =wref_c(k);
    send(robot,velmsg);
    
    %% LECTURA DE LAS POSICIONES DEL ROBOT
    odomdata = receive(odom,3);
    pose = odomdata.Pose.Pose;
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
%     angles=pose.Orientation.Z;
    
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
       
    z0 = [uref_c(k),wref_c(k)]';
    
    %% LECTURA DE LOS OBJETOS
%     world=World(lidarSub,[hx(k+1),hy(k+1),phi(k+1)]);
%     Obj=world;
    
    while(toc<ts)
    end
    toc
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
plot(hxd(1,1:length(hx)),hyd(1,1:length(hx)),'--','Color',[56,171,217]/255,'linewidth',1.5); hold on;
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(hxd(1,1:length(hx)),hyd(1,1:length(hx)),'--','Color',[56,171,217]/255,'linewidth',1.5); hold on;
plot(hx,hy,'Color',[32,185,29]/255,'linewidth',1.5); hold on;
grid on;
grid minor;
legend({'$\mathbf{\eta}_{p_{des}}$','$\mathbf{\eta}_{p}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Trayectoria Deseada y Trayectoria Descrita}$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9)

grid on;
grid minor;
legend({'$\mathbf{\eta}_{p_{des}}$','$\mathbf{\eta}_{p}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Trayectoria Deseada y Trayectoria Descrita}$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9)

print -dpng SIMULATION_1_OPTIMIZACION
print -depsc SIMULATION_1_OPTIMIZACION

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
t=[0:ts:tf-ts];
subplot(2,1,1)
    plot(t(1:length(hxe)),hxe,'Color',[226,76,44]/255,'linewidth',1); hold on;
    plot(t(1:length(hxe)),hye,'Color',[46,188,89]/255,'linewidth',1); hold on;
    grid on;
    grid minor;
    legend({'$\tilde{x_{p}}$','$\tilde{y_{p}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Errores de Posicion}$','Interpreter','latex','FontSize',9);
    ylabel('$[m]$','Interpreter','latex','FontSize',9);
  
subplot(2,1,2)
    plot(t(1:length(hxe)),ue,'Color',[223,67,85]/255,'linewidth',1); hold on
    plot(t(1:length(hxe)),we,'Color',[56,171,217]/255,'linewidth',1); hold on
    grid on;
    grid minor;
    legend({'$\tilde\mu$','$\tilde{\dot\psi_{p}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Errores de Velocidad}$','Interpreter','latex','FontSize',9);
    xlabel('$\textrm{Tiempo}[s]$','Interpreter','latex','FontSize',9);ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);

    
print -dpng CONTROL_ERRORS_1_OPTIMIZACION
print -depsc CONTROL_ERRORS_1_OPTIMIZACION

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
    subplot(2,1,1)
    plot(t(1:length(u)),u,'Color',[216,72,178]/255,'linewidth',1); hold on
    plot(t(1:length(u)),w,'Color',[129,123,110]/255,'linewidth',1); hold on
    plot(t(1:length(uref_c)),uref_c,'--','Color',[216,72,178]/255,'linewidth',1); hold on
    plot(t(1:length(wref_c)),wref_c,'--','Color',[129,123,110]/255,'linewidth',1); hold on
    grid on;
    grid minor;
    title('$\textrm{Velocidades a la Salida del Robot y Velocidades de Control Cinematico}$','Interpreter','latex','FontSize',9);
    xlabel('$\textrm{Tiempo}[s]$','Interpreter','latex','FontSize',9);ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
    legend({'$\mu$','$\dot\psi_{p}$','$\mu_{ref_{c}}$','$\dot\psi_{p_{ref_{c}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    
    subplot(2,1,2)
    plot(t(1,1:length(uref)),uref,'Color',[217,204,30]/255,'linewidth',1); hold on
    plot(t(1,1:length(uref)),wref,'Color',[83,57,217]/255,'linewidth',1); grid on
    grid on;
    grid minor;
    legend({'$\mu_{ref}$','$\dot\psi_{p_{ref}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Velocidades de Compensacion Dinamica}$','Interpreter','latex','FontSize',9);
    xlabel('$\textrm{Tiempo }[s]$','Interpreter','latex','FontSize',9);;ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);

print -dpng CONTROL_VALUES_1_OPTIMIZACION
print -depsc CONTROL_VALUES_1_OPTIMIZACION