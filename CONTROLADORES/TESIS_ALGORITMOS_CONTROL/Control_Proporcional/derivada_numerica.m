function v_p = derivada_numerica(v,v_1,k,ts)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here
if(k==1)
   v_p=0;
else
    v_p=(v-v_1)/ts;
end

end

