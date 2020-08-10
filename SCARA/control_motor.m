classdef control_motor < handle
    properties
        % Parámetros del motor
        Ra=7.06;
        La=16.94E-3;
        Jm=7.2E-6;
        Bm=313.04E-6;
        kb=0.0239;
        kt=0.0532;
        
        % Parámetros iniciales
        max_lim = 0.5;
        tL = 0.045/1000;
        
        % Parámetros iniciales del control
        Kp = 0;
        ti = 0;
        lambda = 0;
        Kc = 0;
        T = 0;
    end
    
    methods
        function obj = control_motor()            
            % Parámetros del control
            obj.Kp = 1;
            obj.ti = obj.Jm*obj.Ra/(obj.kb*obj.kt);
            obj.lambda = 0.1;
            obj.Kc = obj.ti/(15.28*obj.lambda);
            obj.T = 2*pi/(10*(1/obj.ti));
        end
        
        function [th_end,ik_1,wk_1,Vk_1,e2k_1] = control(obj,th_init,th_end,ik_1,wk_1,Vk_1,e2k_1)
            theta_ref = th_end;
            theta_k_1 = th_init;
            % La salida del P es la referencia de w
            e1k = theta_ref - theta_k_1;
            w_ref_k = obj.Kp*e1k;
            
            % Limitar la referencia de la velocidad angular del motor
            if(w_ref_k > obj.max_lim)
                w_ref_k = obj.max_lim;
            elseif(w_ref_k < -obj.max_lim)
                w_ref_k = -obj.max_lim;
            end
            
            % La salida del PI es Vk
            e2k = w_ref_k - wk_1;
            Vk = Vk_1 + obj.Kc*(e2k - e2k_1 + (obj.T/obj.ti)*e2k);
            
            % Solución del modelo
            [wk, theta_k, ik] = motor(theta_k_1,obj.tL,obj.T,ik_1,Vk,wk_1);
            
            wk_1 = wk;
            th_end = theta_k;
            e2k_1 = e2k;
            ik_1 = ik;
            Vk_1 = Vk;
        end
    end
end

