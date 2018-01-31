clear all
close all
tic
[t1,y1] = ode45(@free,[0 10],[0; 0.5]);
toc
figure
plot(t1,y1(:,1),'-o');
title('Solution for Free and Damped vibration system with ODE45');
xlabel('Time t');
ylabel('Displacement [m]');

tic
[t2,y2] = ode15s(@free,[0 10],[0; 0.5]);
toc
figure
plot(t2,y2(:,1),'-o');
title('Solution for Free and Damped vibration system with ODE15S');
xlabel('Time t');
ylabel('Displacement [m]');

tic
[t3,y3] = ode23s(@free,[0 10],[0; 0.5]);
toc
figure
plot(t3,y3(:,1),'-o');
title('Solution for Free and Damped vibration system with ODE23S');
xlabel('Time t');
ylabel('Displacement [m]');

options = odeset('RelTol',1e-6,'AbsTol',1e-9);
tic
% options = odeset('RelTol',1e-20,'AbsTol',[1e-10 1e-10]);
[t4,y4] = ode15s(@free,[0 10],[0; 0.5],options);
toc
figure
plot(t4,y4(:,1),'-o');
title('Solution for Free and Damped vibration system with ODE15S');
xlabel('Time t');
ylabel('Displacement [m]');