function Dinamica = Adaptativo_1(vrefp,vref_e,v,vd,PARAMETROS,ts)

chi=PARAMETROS;
% a) Velocidades de la plataforma móvil 
     u = v(1);
     w = v(2);

     udp=vrefp(1);
     wdp=vrefp(2);
    
    ud=vd(1);
    wd=vd(2);
 
n=[udp,0;...
    0,wdp;...
    u,0;...
    0,w;...
    ud,0;...
    0,wd];

K=[10,0;...
    0,1];

chip=n*K*vref_e;

chi=chi+chip*ts;
% b) Estados de la plataforma móvil 
control=n'*chi;
% ESTA ESS UNA LEY DE CONTROL ADAPTATIVA

  
% c) Parámetros Dinámicos de la plataforma movil

    Dinamica = [control;chi];
end