clc; %clear all;

th_init = [0; 0; 0];

l1 = 0.5;   %---> h desfase de altura con respecto al suelo
l2 = 0.8;   %---> L1 longitud del primer eslabón
l3 = 0.8;   %---> L2 longitud del segundo eslabón
d3_max = 0.2; %---> longitud del tercer eslabón
d = (80/4)/100; %---> metros que se mueve por cada 360°

otro = 1;

while otro == 1
    % Ingresar coordenadas deseadas para el efector final
    prompt = 'Ingrese X inicial: ';
    x0 = input(prompt);
    prompt = 'Ingrese Y inicial: ';
    y0 = input(prompt);
    prompt = 'Ingrese TH inicial: ';
    th0 = input(prompt);
    prompt = 'Ingrese el valor de PX: ';
    px = input(prompt);
    prompt = 'Ingrese el valor de PY: ';
    py = input(prompt);
    prompt = 'Ingrese el valor de PZ: ';
    pz = input(prompt);
    th = zeros(3,4);
    
    T00 = [cos(th0) -sin(th0) 0 x0; sin(th0) cos(th0) 0 y0; 0 0 1 0; 0 0 0 1];
    T = [1 0 0 px; 0 1 0 py; 0 0 1 pz; 0 0 0 1];
    Tf = T00*T;
    
    % ángulo theta 2
    c2 = (px^2 + py^2 - l3^2 - l2^2)...
        /(2*l2*l3);
    s2 = sqrt(1 - c2^2);
    
    th(2,1:2) = atan2(s2,c2);
    th(2,3:4) = atan2(-s2,c2);
    
    % ángulo theta 1
    % para +s2
    th2 = th(2,1);
    c1 = (px*(cos(th2)*l3 + l2) + py*sin(th2)*l3)...
        /(px^2+py^2);
    s1 = sqrt(1 - c1^2);
    
    th(1,1) = atan2(s1,c1);
    th(1,2) = atan2(-s1,c1);
    
    % ángulo theta 1
    % para -s2
    th2 = th(2,3);
    c1 = (px*(cos(th2)*l3 + l2) + py*sin(th2)*l3)...
        /(px^2+py^2);
    s1 = sqrt(1 - c1^2);
    
    th(1,3) = atan2(s1,c1);
    th(1,4) = atan2(-s1,c1);
    
    % d3 desfase del actuador con respecto a la altura del carro DDR
    d3 = pz - l1;
    
    for i = 1:4
        th1 = th(1,i);
        th2 = th(2,i);
        T01 = [cos(th1) -sin(th1) 0 0; sin(th1) cos(th1) 0 0; 0 0 1 l1; 0 0 0 1];
        T12 = [cos(th2) -sin(th2) 0 l2; sin(th2) cos(th2) 0 0; 0 0 1 0; 0 0 0 1];
        T23 = [1 0 0 l3; 0 1 0 0; 0 0 1 d3; 0 0 0 1];
        
        T03 = T01*T12*T23;
        
        if (abs(T03(1,4)-px) < 0.00001 && abs(T03(2,4)-py) < 0.00001 && abs(T03(3,4)-pz) < 0.00001)
            th(3,i) = 1;
        end
    end
    
    k = sum(th(3,:) == 1);
    a = 1;
    th_final = zeros(2,k);
    
    for i = 1:4
        if th(3,i) == 1
            th_final(1:2,a) = th(1:2,i);
            a = a + 1;
        end
    end
    
    if k >= 1
        dif = abs(th_final(1:2,:)-th_init(1:2));
        [val,indice] = min(dif(1,:));
        
        th1 = th_final(1,indice);
        th2 = th_final(2,indice);
        th3 = d3*2*pi/d;
    end
    
    th_final = [th1; th2; th3];
    
    control_th1 = control_motor();
    control_th2 = control_motor();
    control_th3 = control_motor();
    
    control_th = [control_th1 control_th2 control_th3];
    
    % grafico del movimiento del brazo robótico respecto al punto deseado
    figure(1)
    
    for i = 1:3
        ik_1 = 0;
        wk_1 = 0;
        Vk_1 = 0;
        e2k_1 = 0;
        
        while (abs(th_init(i)-th_final(i)) > 0.01)
            [th_init(i),ik_1,wk_1,Vk_1,e2k_1] = control_th(i).control(th_init(i),th_final(i),ik_1,wk_1,Vk_1,e2k_1);
            T01 = [cos(th_init(1)) -sin(th_init(1)) 0 0; sin(th_init(1)) cos(th_init(1)) 0 0; 0 0 1 l1; 0 0 0 1];
            T12 = [cos(th_init(2)) -sin(th_init(2)) 0 l2; sin(th_init(2)) cos(th_init(2)) 0 0; 0 0 1 0; 0 0 0 1];
            d3 = th_init(3)*d/(2*pi);
            T23 = [1 0 0 l3; 0 1 0 0; 0 0 1 d3; 0 0 0 1];
            T02 = T00*T01*T12;
            T03 = T00*T01*T12*T23;
            plot3([x0 x0],[y0 y0],[0 l1])
            hold on
            plot3([x0 T02(1,4)],[y0 T02(2,4)],[l1 T02(3,4)])
            title('Coordenadas');
            xlabel('X'); ylabel('Y'); zlabel('Z');
            grid on
            hold on
            plot3([T02(1,4) T03(1,4)],[T02(2,4) T03(2,4)],[T02(3,4) T02(3,4)])
            hold on
            plot3([T03(1,4) T03(1,4)],[T03(2,4) T03(2,4)],[T03(3,4) T03(3,4)+d3_max])
            axis([x0-2 x0+2 y0-2 y0+2 0 1])
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