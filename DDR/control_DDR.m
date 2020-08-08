classdef control_DDR < handle
   properties
      
       ts = 0.1;
       
       %Setting of initial conditions of the control
       xc = 0; %origin in x
       yc = 0; %origin in y
       phi = 0
       L = 1; %Distance between wheels 
       R = 1;
       a = 0; %distance from the center of wheels til the control
       
       xr = 0;
       yr = 0;
       
       K1=0.5; 
       K2=0.5;
       q2=0.5;

       %omega angular velocity control for left wheel
       control_L = control_omega();
       vel_L = 0;
       vel_L_1 = 0;

       %omega angular velocity control for Right wheel
       control_R = control_omega();
       vel_R = 0;
       vel_R_1 = 0;

   end
   methods
       %constructor
       function obj = control_DDR(xc, yc, phi)
           
           %initial coordenates of the robot in funtion of control localization
           obj.phi = phi; %orientation respect of x axle
           obj.xr = xc + obj.a * cos(obj.phi);
           obj.yr = yc + obj.a * sin(obj.phi);
           
           
       end
       
       function [xr, yr, phi] = control(obj, xrd, yrd, phid)
           
           l=sqrt((xrd-obj.xr)^2+(yrd-obj.yr)^2);
           zeta=atan2((yrd-obj.yr),(xrd-obj.xr))-obj.phi;
           psi=atan2((yrd-obj.yr),(xrd-obj.xr))-phid;
           
           %d) Ley de control    
           u=obj.K1*cos(zeta)*l;     % Velocidad lineal de entrada
           w=obj.K2*zeta+(obj.K1/zeta)*cos(zeta)*sin(zeta)*(zeta+obj.q2*psi); 
           

           %transformation of linear and angular speed to omega velocity of each wheel
           obj.control_R.w_ref_k = (u + w/2) / obj.R;
           obj.control_L.w_ref_k = (u - w/2) / obj.R;

           %Right Wheel control
           obj.vel_R = obj.control_R.control_w(obj.vel_R_1);
           obj.vel_R_1 = obj.vel_R;

           %Left Wheel control
           obj.vel_L = obj.control_L.control_w(obj.vel_L_1);
           obj.vel_L_1 = obj.vel_L;

           %Transformation of omega velocity of each wheel to linear and angular speedo
           u = obj.R * (obj.vel_R_1 + obj.vel_L_1) / 2;
           w = obj.R * (obj.vel_R_1 - obj.vel_L_1) / obj.L;

           %control actions on the robot
           xrp = u * cos(obj.phi) - obj.a * w * sin(obj.phi);
           yrp = u * sin(obj.phi) + obj.a * w * cos(obj.phi);

           %Positions
           obj.xr  = obj.xr  +  obj.ts * xrp;
           obj.yr  = obj.yr  +  obj.ts * yrp;
           obj.phi = obj.phi +  obj.ts * w  ;
           
           xr = obj.xr;
           yr = obj.yr;
           phi = obj.phi;
      end
      
   end
end
