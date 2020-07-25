clc
clear
close all
ts=0.1;
t=0:ts:8;
%Setting of initial conditions of the control
xc(1)=0; %origin in x
yc(1)=0; %origin in y
phi(1)=pi*3/2; %orientation respect of x axle
L = 1; %Distance between wheels 
R=1
%Expected point to be reached
xrd=10; %Type here the desired x value
yrd=10; %Type here the desired y value

a=0; %distance from the center of wheels til the control
% u=1*ones(1,length(t));
% w=0.1*ones(1,length(t));

%initial coordenates of the robot in funtion of control localization
xr(1)=xc(1)+a*cos(phi(1));
yr(1)=yc(1)+a*sin(phi(1));

%omega angular velocity control for left wheel
control_L = control_omega();
vel_L = 0;
vel_L_1 = 0;
vels_L =[];

%omega angular velocity control for Right wheel
control_R = control_omega();
vel_R = 0;
vel_R_1 = 0;
vels_R =[];

for k = 1:length(t)
    
    %Error calculation
    xre(k) = xrd - xr(k);
    yre(k) = yrd - yr(k);
    %Error matrixv (2,1)
    e=[xre(k);yre(k)];
    
    %Jacobian matrix (2,2)
    J=[cos(phi(k)) -sin(phi(k)); sin(phi(k)) cos(phi(k))];
    phi(k)
    %Matrix of gaining parameters (2,2)
    K=[.5 0;0 .5];
    
    %control law
    v=inv(J)*K*e;
    
    %initial lineal and angular velocities of the robot
    u(k)=v(1);
    w(k)=v(2);
    
    %transformation of linear and angular speed to omega velocity of each wheel
    control_R.w_ref_k = (u(k) + w(k)/2)/R;
    control_L.w_ref_k = (u(k) - w(k)/2)/R;
    
    %Right Wheel control
    vels_R = [vels_R vel_R];
    vel_R = control_R.control_w(vel_R_1);
    vel_R_1 = vel_R;
    
    %Left Wheel control
    vels_L = [vels_L vel_L];
    vel_L = control_L.control_w(vel_L_1);
    vel_L_1 = vel_L;
    
    %Transformation of omega velocity of each wheel to linear and angular speedo
    u(k) = R*(vel_R_1 + vel_L_1)/2;
    w(k) = R*(vel_R_1 - vel_L_1)/L;
    
    %control actions on the robot
    xrp(k)=u(k)*cos(phi(k))-a*w(k)*sin(phi(k));
    yrp(k)=u(k)*sin(phi(k))+a*w(k)*cos(phi(k));
    
    %Positions
    xr(k+1)=xr(k)+ts*xrp(k);
    yr(k+1)=yr(k)+ts*yrp(k);
    phi(k+1)=phi(k)+ts*w(k);
    
    xc(k+1)=xr(k+1)-a*cos(phi(k+1));
    yc(k+1)=yr(k+1)-a*sin(phi(k+1));

    plot(xc(1),yc(1),'xr',xrd,yrd,'xr')
    hold on
    plot(xr,yr,"o"),xlim([-20,20]),ylim([-20 20])
    drawnow;
    %pause(0.00001)
end
tiledlayout(2,2)

% Top plot
nexttile
plot(vels_R)

% bottom plot
nexttile
plot(vels_L)

nexttile
plot(xr,yr),xlim([-20,20]),ylim([-20 20])