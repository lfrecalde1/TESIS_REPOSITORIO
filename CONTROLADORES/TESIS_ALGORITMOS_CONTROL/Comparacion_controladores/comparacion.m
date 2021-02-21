clc,clear all,close all;
%% cargar datos de los errores de los controladores
load('hep.mat');
load('hef.mat');
load('heo.mat');
load('hem.mat');
load('hen.mat');
%% generacion del grafico de errores cuadraticos medios en x y
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
%% Generacion de los nombres de los controladores
controlador= categorical({'Proporcional','Fuzzy','Optimo','Mpc','Neuronal'});
controlador = reordercats(controlador,{'Proporcional','Fuzzy','Optimo','Mpc','Neuronal'});

%% Obtencion del error cuadratico medio
proporcional=hep*hep'/length(hep);
fuzzy=hef*hef'/length(hef);
optimo=heo*heo'/length(heo);
mpc=hem*hem'/length(heo);
neu=hen*hen'/length(hen);
%% Almancenamiento para el error cuadratico medio
Mse=[proporcional(1,1),proporcional(2,2);fuzzy(1,1),fuzzy(2,2);...
    optimo(1,1),optimo(2,2);...
    mpc(1,1),mpc(2,2);...
    neu(1,1),neu(2,2)];
%% Generacion de las graficas de los errores cuadraticos medios
bar(controlador,Mse);  
grid on;
grid minor;
legend({'$MSE_{x}$','$MSE_{y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Mse Proporcional}$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Controladores}$','Interpreter','latex','FontSize',9); ylabel('$\textrm{MSE}$','Interpreter','latex','FontSize',9)

print -dpng MSE_X_PROPORCIONAL
print -depsc MSE_Y_PROPORCIONAL