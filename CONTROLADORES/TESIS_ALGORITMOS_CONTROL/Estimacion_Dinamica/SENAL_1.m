function [uref_c, wref_c] = SENAL_1(t,type)
%SENAL1
%   PROGRAMA PARA LA GENERACION DE LAS VELOCIDADES DE EXCITACION
A1 = 0.04;
A2 = 0.1;
W1 = 0.7;
W2 = 0.6;
W3 = 0.5;
W4 = 0.4;
W5 = 0.3;
W6 = 0.2;
for k=1:length(t)
if(type==0)
uref_c(k) = A1*sin(W1*t(k))+A1*sin(W2*t(k))+A1*sin(W3*t(k))+A1*sin(W4*t(k))+A1*sin(W5*t(k))+A1*sin(W6*t(k));
wref_c(k) = A2*sin(W1*t(k))+A2*sin(W2*t(k))+A2*sin(W3*t(k))+A2*sin(W4*t(k))+A2*sin(W5*t(k))+A2*sin(W6*t(k));
end
if(type==1)
uref_c(k)=0.2*sin(0.7*t(k));
%uref_c(k)=0.3*sqrt(0.2^2+(0.2*pi*cos(0.3*pi*t(k)))^2);
wref_c(k)=0.7*(-0.3*pi^2*sin(0.3*pi*t(k)))/(1+pi^2*cos(0.3*pi*t(k))^2);
end
if(type==2)
   % Identificación
        if k<=50
            uref_c(k)=0.2;
            wref_c(k)=0.1;
        else if (k>50) && (k<=100)  
                uref_c(k)=0;
                wref_c(k)=0.1;
            else if (k>100) && (k<=120)  
                    uref_c(k)=-0.2;
                    wref_c(k)=0.2;
                else if (k>120) && (k<=200)  
                        uref_c(k)=-0.2;
                        wref_c(k)=0;    
                    else if (k>200) && (k<=250)  
                            uref_c(k)=0;
                            wref_c(k)=0.3;
                        else if (k>250) && (k<=300)  
                                uref_c(k)=0.3;
                                wref_c(k)=0.1;
                            else if (k>300) && (k<=350)  
                                    uref_c(k)=0.1;
                                    wref_c(k)=0.3;
                                else if (k>350) && (k<=400)
                                        uref_c(k)=0;
                                        wref_c(k)=-0.1;
                                    else if (k>400) && (k<=450)
                                            uref_c(k)=0;
                                            wref_c(k)=-0.5;
                                        else if (k>450) && (k<=550)  
                                                uref_c(k)=0;
                                                wref_c(k)=0.1;
                                            else if (k>550) && (k<=600)  
                                                    uref_c(k)=-0.05;
                                                    wref_c(k)=0.5;
                                                else if (k>600) && (k<=650)  
                                                        uref_c(k)=-0.2;
                                                        wref_c(k)=0.05;
                                                    else if (k>650) && (k<=700)  
                                                            uref_c(k)=0.1;
                                                            wref_c(k)=0;
                                                        else if (k>700) && (k<=750)  
                                                                uref_c(k)=0.3;
                                                                wref_c(k)=0;
                                                            else if k>750  
                                                                    uref_c(k)=0;
                                                                    wref_c(k)=0;
                                                                end
                                                            end
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
end
end
end

