function [T02, T03] = DK_SCARA(T00, th, l1, l2, l3, d)

T01 = [cos(th(1)) -sin(th(1)) 0 0; sin(th(1)) cos(th(1)) 0 0; 0 0 1 l1; 0 0 0 1];
T12 = [cos(th(2)) -sin(th(2)) 0 l2; sin(th(2)) cos(th(2)) 0 0; 0 0 1 0; 0 0 0 1];
d3 = th(3)*d/(2*pi);
T23 = [1 0 0 l3; 0 1 0 0; 0 0 1 d3; 0 0 0 1];
T02 = T00*T01*T12;
T03 = T00*T01*T12*T23;

end

