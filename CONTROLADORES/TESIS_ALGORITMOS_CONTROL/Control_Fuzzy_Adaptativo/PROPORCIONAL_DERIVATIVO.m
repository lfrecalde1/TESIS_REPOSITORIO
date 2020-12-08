function cinematico= PROPORCIONAL_DERIVATIVO(K1,K3,K2,hd,hdp,h,hp,q,a,si)
     he=hd-h;
     
     hep=hdp-hp;
     
     phi=q(2);
     
     K1=diag(K1);
     
     K3=diag(K3);
     
     
     K2=K2*diag([1 1]);
     
     J11=cos(phi);
     
     J12=-a*sin(phi);
     
     J21=sin(phi);
     
     J22=a*cos(phi);
     
     J=[J11 J12;
        J21 J22];
    
     s=he+K2*si;
     %% LEY DE CONTROL SEGUIMIENTO DE TRAYECTORIA
%      cinematico=inv(J)*(hdp+K2*tanh(K2^-1*K1*he)+K2*tanh(K2^-1*K3*hep));
     cinematico=inv(J)*(hdp+K2*tanh(K2^-1*K1*he));
    
end
