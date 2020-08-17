clc; clear all;

th_init = [1; 1];

l1 = 0.5;   %---> h desfase de altura con respecto al suelo
l2 = 0.8;   %---> L1 longitud de brazo uno
l3 = 0.8;   %---> L2 longitud de brazo dos

otro = 1;

while otro == 1
    % Ingresar coordenadas deseadas para el efector final
    prompt = 'Ingrese el valor de PX: ';
    px = input(prompt);
    prompt = 'Ingrese el valor de PY: ';
    py = input(prompt);
    %prompt = 'Ingrese el valor de PZ: ';
    %pz = input(prompt);
    pz=0;
    h = sqrt(px^2+py^2);
    if (h > 1.6 || (px==0 && py==0))
        disp('El valor indicado esta fuera de las dimensiones del brazo.')
    else
   
    th = zeros(3,4);
    
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
        dif = abs(th_final(1:2,:)-th_init);
        [val,indice] = min(dif(1,:));
        
        th1 = th_final(1,indice);
        th2 = th_final(2,indice);
    end
    
    % envio de valores a simulink
    th_final_sim = th1;
    th_init_sim = th_init(1,1);
    value = sim('control_brazo2');
    t = value.tout;
    pos1 = value(1).yout{1}.Values.Data(:,1);
    
    % grafico del movimiento del brazo robotico respecto al punto deseado
    figure(1)
    for i = 1:length(t)
        th1 = pos1(i);
        th2_init = th_init(2,1);
        T01 = [cos(th1) -sin(th1) 0 0; sin(th1) cos(th1) 0 0; 0 0 1 l1; 0 0 0 1];
        T12 = [cos(th2_init) -sin(th2_init) 0 l2; sin(th2_init) cos(th2_init) 0 0; 0 0 1 0; 0 0 0 1];
        T23 = [1 0 0 l3; 0 1 0 0; 0 0 1 d3; 0 0 0 1];
        T02 = T01*T12;
        T03 = T01*T12*T23;
        plot([0 T02(1,4)],[0 T02(2,4)])
        title('Coordenadas');
        xlabel('X'); ylabel('Y')
        grid on
        hold on
        plot(px,py,'r*')
        plot(T02(1,4),T02(2,4),'bo')
        plot([T02(1,4) T03(1,4)],[T02(2,4) T03(2,4)])
        plot(0,0,'bo')
        plot(T03(1,4), T03(2,4),'o');
        axis([-2 2 -2 2])
        hold off
        POS = [T03(1,4), T03(2,4)];
        pause(0.01)
    end
    
    % envio de valores a simulink
    th_final_sim = th2;
    th_init_sim = th_init(2,1);
    value = sim('control_brazo2');
    t = value.tout;
    pos2 = value(1).yout{1}.Values.Data(:,1);
    
    for i = 1:length(t)
        th2 = pos2(i);
        T01 = [cos(th1) -sin(th1) 0 0; sin(th1) cos(th1) 0 0; 0 0 1 l1; 0 0 0 1];
        T12 = [cos(th2) -sin(th2) 0 l2; sin(th2) cos(th2) 0 0; 0 0 1 0; 0 0 0 1];
        T23 = [1 0 0 l3; 0 1 0 0; 0 0 1 d3; 0 0 0 1];
        T02 = T01*T12;
        T03 = T01*T12*T23;
        plot([0 T02(1,4)],[0 T02(2,4)])
        title('Coordenadas');
        xlabel('X'); ylabel('Y')
        grid on
        hold on
        plot(px,py,'r*')
        plot(T02(1,4),T02(2,4),'bo')
        plot([T02(1,4) T03(1,4)],[T02(2,4) T03(2,4)])
        plot(0,0,'bo')
        plot(T03(1,4), T03(2,4),'o');
        axis([-2 2 -2 2])
        hold off
        POS = [T03(1,4), T03(2,4)];
        pause(0.01)
    end
    
    px = T03(1,4);
    py = T03(2,4);
    disp('A llegado al punto indicado.')
    
    end
    prompt = '¿Desea ingresar otra posición? ';
    otro = input(prompt);
    th_init = [th1; th2];
    clc
    
end