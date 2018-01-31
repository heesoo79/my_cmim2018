clear all
close all
options = odeset('RelTol',1e-10,'AbsTol',1e-15);
[t1,y1] = ode15s(@forced,[0 100],[0; 0],options);
figure
plot(t1,y1(:,1));
set(gca,'YLim',[-5e-2 5e-2]);
title('Forced and Damped vibration system with ODE15S');
xlabel('Time t');
ylabel('Displacement [m]');