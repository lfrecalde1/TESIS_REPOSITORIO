function [Kp,Kd]= FUZZY_ADAPTATIVO(self_x,self_y,hd,hdp,h,hp)
    he=hd-h;
    hep=hdp-hp;
     % CALCULPO DE LAS GANANCIAS
    K_x=evalfis(tanh([he(1),hep(1)]),self_x);
    K_y=evalfis(tanh([he(2),hep(2)]),self_y);
    Kp=[K_x(1) K_y(1)]';
    Kd=[K_x(2) K_y(2)]';
end
