clc
clear
close all

xr = 5;
yr = 0;
phi = -pi/2;
roboto = control_DDR(xr, yr, phi);

th_init = [0; 0; 0];

l1 = 0.5;   %---> h desfase de altura con respecto al suelo
l2 = 0.8;   %---> L1 longitud del primer eslabón
l3 = 0.8;   %---> L2 longitud del segundo eslabón
d3_max = 0.2; %---> longitud del tercer eslabón
d = (80/4)/100; %---> metros que se mueve por cada 360°

control_th1 = control_SCARA();
control_th2 = control_SCARA();
control_th3 = control_SCARA();

control_th = [control_th1 control_th2 control_th3];

otro = 1;

while otro == 1
    % Ingresar coordenadas deseadas para el efector final
    prompt = 'Ingrese X final DDR: ';
    x0 = input(prompt);
    prompt = 'Ingrese Y final DDR: ';
    y0 = input(prompt);
    prompt = 'Ingrese TH final DDR: ';
    th0 = input(prompt);
    prompt = 'Ingrese X final SCARA: ';
    px = input(prompt);
    prompt = 'Ingrese Y final SCARA: ';
    py = input(prompt);
    prompt = 'Ingrese Z final SCARA: ';
    pz = input(prompt);
    
    th_final = IK_SCARA(px, py, pz, l1, l2, l3, th_init, d);
    
    figure(1)
    % Graficar poses iniciales
    T00 = [cos(phi) -sin(phi) 0 xr; sin(phi) cos(phi) 0 yr; 0 0 1 0; 0 0 0 1];
    [T02, T03] = DK_SCARA(T00, th_init, l1, l2, l3, d);
    plot3(xr, yr, 0,'o'),xlim([-20,20]),ylim([-20 20]),zlim([0 1])
    hold on
    plot3([xr xr],[yr yr],[0 l1])
    hold on
    plot3([xr T02(1,4)],[yr T02(2,4)],[l1 T02(3,4)])
    title('Coordenadas');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    grid on
    hold on
    plot3([T02(1,4) T03(1,4)],[T02(2,4) T03(2,4)],[T02(3,4) T02(3,4)])
    hold on
    plot3([T03(1,4) T03(1,4)],[T03(2,4) T03(2,4)],[T03(3,4) T03(3,4)+d3_max])
    hold off
    pause(0.01)
    
    while (abs(xr-x0) > 0.01 || abs(yr-y0) > 0.01 || abs(phi-th0) > 0.01)
        %[xr, yr, phi] = roboto.control(-1, 15, pi*1/2);
        [xr, yr, phi] = roboto.control(x0, y0, th0);
        T00 = [cos(phi) -sin(phi) 0 xr; sin(phi) cos(phi) 0 yr; 0 0 1 0; 0 0 0 1];
        [T02, T03] = DK_SCARA(T00, th_init, l1, l2, l3, d);
        plot3(xr, yr, 0,'o'),xlim([-20,20]),ylim([-20 20]),zlim([0 1])
        hold on
        plot3([xr xr],[yr yr],[0 l1])
        hold on
        plot3([xr T02(1,4)],[yr T02(2,4)],[l1 T02(3,4)])
        title('Coordenadas');
        xlabel('X'); ylabel('Y'); zlabel('Z');
        grid on
        hold on
        plot3([T02(1,4) T03(1,4)],[T02(2,4) T03(2,4)],[T02(3,4) T02(3,4)])
        hold on
        plot3([T03(1,4) T03(1,4)],[T03(2,4) T03(2,4)],[T03(3,4) T03(3,4)+d3_max])
        hold off
        pause(0.01)
    end
    
    T00 = [cos(phi) -sin(phi) 0 xr; sin(phi) cos(phi) 0 yr; 0 0 1 0; 0 0 0 1];
    
    for i = 1:3
        ik_1 = 0;
        wk_1 = 0;
        Vk_1 = 0;
        e2k_1 = 0;
        
        while (abs(th_init(i)-th_final(i)) > 0.01)
            [th_init(i),ik_1,wk_1,Vk_1,e2k_1] = control_th(i).control(th_init(i),th_final(i),ik_1,wk_1,Vk_1,e2k_1);
            [T02, T03] = DK_SCARA(T00, th_init, l1, l2, l3, d);
            plot3(xr, yr, 0,'o'),xlim([-20,20]),ylim([-20 20]),zlim([0 1])
            hold on
            plot3([xr xr],[yr yr],[0 l1])
            hold on
            plot3([xr T02(1,4)],[yr T02(2,4)],[l1 T02(3,4)])
            title('Coordenadas');
            xlabel('X'); ylabel('Y'); zlabel('Z');
            grid on
            hold on
            plot3([T02(1,4) T03(1,4)],[T02(2,4) T03(2,4)],[T02(3,4) T02(3,4)])
            hold on
            plot3([T03(1,4) T03(1,4)],[T03(2,4) T03(2,4)],[T03(3,4) T03(3,4)+d3_max])
            hold off
            POS = [T03(1,4), T03(2,4), T03(3,4)];
            pause(0.01)
        end
    end
    
    disp('Ha llegado al punto indicado.')
    POS
    
    prompt = '¿Desea ingresar otra posición? ';
    otro = input(prompt);
end