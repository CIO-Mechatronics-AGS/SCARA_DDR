function [wk, theta_k, ik] = motorS(theta_k_1,tL,T,ik_1,Vk,wk_1)
% Parámetros del motor
% Ra=7.06;
% La=16.94E-3;
% Jm=7.2E-6;
% Bm=313.04E-6;
% kb=0.0239;
% kt=0.0532;

Ra = 4;
La = 2.75E-6;
Jm = 2.825E-6;
Bm = 3.2077E-6;
kb = 0.0255;
kt = 0.0374;

% Cálculo de la corriente
den_ik = (Jm + Bm*T)*(La + Ra*T)+ kt*kb*T^2;
num_ik = La*(Jm+Bm*T)*ik_1 - kb*T*(Jm*wk_1 + T*tL)+ T*(Jm+Bm*T)*Vk;
ik = num_ik/den_ik;

% Cálculo de la velocidad angular
den_wk = 1/(Jm + Bm*T);
wk = (Jm*wk_1 + kt*T*ik + T*tL)*den_wk;

% Cálculo de la posición angular
theta_k = theta_k_1 + T*wk;
end

