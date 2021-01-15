function cinematico= PROPORCIONAL(K1,K2,hd,hdp,h,q,a)
     he=hd-h;
     phi=q(2);
     K1=K1*diag([1 1]);
     K2=K2*diag([1 1]);
     J11=cos(phi);
     J12=-a*sin(phi);
     J21=sin(phi);
     J22=a*cos(phi);
     J=[J11 J12;
        J21 J22];
     %% LEY DE CONTROL SEGUIMIENTO DE TRAYECTORIA
     cinematico=inv(J)*(hdp+K2*tanh(inv(K2)*K1*he));
end
