classdef control_omega < handle
    properties
        theta_ref = 100 ;% radianes
        tL = 0.045/10;
        ik_1 = 0;
        wk_1 = 0;
        Vk_1 = 0;
        theta_k_1 = 0;
        w_ref_k = 1;
        e2k_1 = 0;

        % Parámetros del motor
        Ra=7.06;
        La=16.94E-3;
        Jm=7.2E-6;
        Bm=313.04E-6;
        kb=0.0239;
        kt=0.0532;

        ti = 0;
        lambda = 0;
        Kc = 0;
        T = 0;
   end
   methods
       %constructor
       function obj = control_omega()
            obj.ti = obj.Jm*obj.Ra/(obj.kb*obj.kt);
            obj.lambda = 0.1;
            obj.Kc = obj.ti/(15.28*obj.lambda);
            obj.T = 2*pi/(10*(1/obj.ti));
            obj.theta_ref = 100;
       end
       function wk = control_w(obj, wk_1)
         
            % La salida del PI es Vk
            e2k = obj.w_ref_k - wk_1;
            Vk = obj.Vk_1 + obj.Kc*(e2k - obj.e2k_1 + (obj.T/obj.ti)*e2k);

            % Solución del modelo
            [wk, theta_k, ik] = motor(obj.theta_k_1,obj.tL,obj.T,obj.ik_1,Vk,wk_1);

            obj.theta_k_1 = theta_k;
            obj.theta_k_1 = theta_k;
            obj.ik_1 = ik;
            obj.e2k_1 = e2k;
            obj.Vk_1 = Vk;
            
       end
   end
end
