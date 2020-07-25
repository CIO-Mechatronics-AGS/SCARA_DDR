clear;
clc;

th_init = [0; 0];

l1 = 0.5;
l2 = 0.8;
l3 = 0.8;

px = -0.5;
py = 0.3;
pz = 0.3;

th = zeros(3,4);

% theta 2
c2 = (px^2 + py^2 - l3^2 - l2^2)/(2*l2*l3);
s2 = sqrt(1 - c2^2);

th(2,1:2) = atan2(s2,c2);
th(2,3:4) = atan2(-s2,c2);

% theta 1
% para +s2
th2 = th(2,1);
c1 = (px*(cos(th2)*l3 + l2) + py*sin(th2)*l3)/(px^2+py^2);
s1 = sqrt(1 - c1^2);

th(1,1) = atan2(s1,c1);
th(1,2) = atan2(-s1,c1);

% para -s2
th2 = th(2,3);
c1 = (px*(cos(th2)*l3 + l2) + py*sin(th2)*l3)/(px^2+py^2);
s1 = sqrt(1 - c1^2);

th(1,3) = atan2(s1,c1);
th(1,4) = atan2(-s1,c1); 


% d3
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
    
    th1 = th(1,indice);
    th2 = th(2,indice);
end

value = sim('control_brazo');
t = value.tout;
pos1 = value(1).yout{1}.Values.Data(:,1);
pos2 = value(1).yout{1}.Values.Data(:,2);

figure(1)
for i = 1:length(t)
    
    th1 = pos1(i);
    th2 = pos2(i);
    T01 = [cos(th1) -sin(th1) 0 0; sin(th1) cos(th1) 0 0; 0 0 1 l1; 0 0 0 1];
    T12 = [cos(th2) -sin(th2) 0 l2; sin(th2) cos(th2) 0 0; 0 0 1 0; 0 0 0 1];
    T23 = [1 0 0 l3; 0 1 0 0; 0 0 1 d3; 0 0 0 1];
    T02 = T01*T12;
    T03 = T01*T12*T23;
    plot([0 T02(1,4)],[0 T02(2,4)])
    hold on
    plot([T02(1,4) T03(1,4)],[T02(2,4) T03(2,4)])
    axis([-1.6 1.6 -1.6 1.6])
    hold off
    pause(0.01)
end
