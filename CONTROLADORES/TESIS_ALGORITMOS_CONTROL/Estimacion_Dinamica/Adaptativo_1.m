function Dinamica = Adaptativo_1(vrefp,vref_e,v,vd,PARAMETROS,ts)
vref_e=-vref_e;
chi=PARAMETROS;
% a) Velocidades de la plataforma móvil 
     u = v(1);
     w = v(2);

     udp=vrefp(1);
     wdp=vrefp(2);
    
    ud=vd(1);
    wd=vd(2);
    

    
K1=2*eye(2,2);

KD=1*eye(2,2);

K2=1*eye(2,2);

s=K2*tanh(inv(K2)*K1*vref_e);
% 
n=[udp,0,ud,wd,w*wd,0,0;...
    0,wdp,0,0,0,ud*w,wd];
% n=[udp,0,ud,0;...
%     0,wdp,0,wd];

delta=[10,0,0,0,0,0,0;
        0,1,0,0,0,0,0;
        0,0,10,0,0,0,0;
        0,0,0,1,0,0,0;
        0,0,0,0,1,0,0
        0,0,0,0,0,1,0;
        0,0,0,0,0,0,1];

chip=-delta*n'*s;

chi=chi+chip*ts;

control=n*chi-KD*s;

    
  
% c) Parámetros Dinámicos de la plataforma movil

    Dinamica = [control;chi];
end