clc
clear
close all
ts=0.1;
t=0:ts:8;

roboto = control_DDR2(0 ,0 ,pi );

for k = 1:length(t)
    
    [xr, yr, phi] = roboto.control(-1, 15);
    hold on
    plot(xr,yr,"o"),xlim([-20,20]),ylim([-20 20])
    drawnow;
    pause(0.01)
end