%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXIDENTIFICACION DINAMICA DE LA PLATAFORMA MOVILXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

%% PARAMETROS DE TIEMPO
clc,clear all,close all;
ts=0.1;
tfinal=50;
to=0;
t=[to:ts:tfinal];

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
a=0.18;

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

[uref_c,wref_c] = SENAL_1(t,1);
%% BUCLE DE SIMULACION 
for k=1:length(t)
    tic;
    
    %% ERRORES DE VELOCIDAD VREF
    ue(k)=uref_c(k)-u(k);
    we(k)=wref_c(k)-w(k);
    vref_e=[ue(k) we(k)]';
    
    %% ENVIO DE DATOS AL ROBOT
    velmsg.Linear.X =uref_c(k);
    velmsg.Angular.Z =wref_c(k);
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

%% ALMACENACION DATOS PARA LA ESTIMACION
save('IDENTIFICACION_DATOS.mat','u','w','uref_c','wref_c','ts','tfinal','to');

%% GRAFICAS DEL SISTEMA
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
    plot(t,u(1:length(t)),'Color',[223,67,85]/255,'linewidth',1); hold on;
    plot(t,w(1:length(t)),'Color',[56,171,217]/255,'linewidth',1); hold on;
    plot(t,uref_c,'--','Color',[223,67,85]/255,'linewidth',1); hold on;
    plot(t,wref_c,'--','Color',[56,171,217]/255,'linewidth',1); hold on;
    grid on;
    grid minor;
    legend({'$\mu$','$\dot\psi_{p}$','$\mu_{ref_{c}}$','$\dot\psi_{p_{ref_{c}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Velocidades a la Salida del Robot Y Velocidades de Excitacion}$','Interpreter','latex','FontSize',9);
    ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
  
subplot(2,1,2)
    plot(t,ue,'Color',[223,67,85]/255,'linewidth',1); hold on
    plot(t,we,'Color',[56,171,217]/255,'linewidth',1); hold on
    grid on;
    grid minor;
    legend({'$\tilde\mu$','$\tilde{\dot\psi_{p}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Errores de Velocidad}$','Interpreter','latex','FontSize',9);
    xlabel('$\textrm{Timpo}[s]$','Interpreter','latex','FontSize',9);ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);

    
print -dpng Identificacion_1
print -depsc Identificacion_1

