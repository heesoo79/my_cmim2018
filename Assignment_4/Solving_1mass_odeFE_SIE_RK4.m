%% Calculation 1 mass-spring system using odeFE,SIE,RK4

m = 1 % kg
k = 100 % N/m

% test function  
test_fun = @(t, y) [y(2); -k/m*y(1)];
% test function for odeSIE
test_fun1 = @(t, y) y; 
test_fun2 = @(t, y) -100*y;

% solution params
dt1 = 1e-5;
dt2 = 1e-7;
tk = 3;

% initial conditions
y0 = [1, 0];

% solution of ODE
tic
[T1, Y1] = odeFE(test_fun, [dt1, tk], y0);
toc
tic
[T2, Y2] = odeFE(test_fun, [dt2, tk], y0);
toc
tic
[T3, Y3] = odeSIE(test_fun1, test_fun2,[dt1, tk], y0);
toc
tic
[T4, Y4] = odeRK4(test_fun,[dt1, tk], y0);
toc
% total system energy
Etotal1 = 0.5*m*Y1(2,:).^2 + 0.5*k*Y1(1,:).^2;
Etotal2 = 0.5*m*Y2(2,:).^2 + 0.5*k*Y2(1,:).^2;
Etotal3 = 0.5*m*Y3(2,:).^2 + 0.5*k*Y3(1,:).^2;
Etotal4 = 0.5*m*Y4(2,:).^2 + 0.5*k*Y4(1,:).^2;
% Plot
figure
set(gcf,'paperunits','inches')
set(gcf,'position',[5 5 800 300])
plot(T1, Y1(1, :),T2, Y2(1, :), T3, Y3(1, :), T4, Y4(1, :))
title('Displacement vibration','FontWeight','normal','FontName','Times','Fontsize',14)
xlabel('Time [sec]','Fontsize',12)
ylabel('Displacement [m]','Fontsize',12)
grid on 
figure
set(gcf,'paperunits','inches')
set(gcf,'position',[5 5 800 300])
plot(T1, Etotal1,T2, Etotal2, T3, Etotal3, T4, Etotal4)
title('Total energy','FontWeight','normal','FontName','Times','Fontsize',14)
xlabel('Time [sec]','Fontsize',12)
ylabel('Energy [kgm^2/s^2]','Fontsize',12)
set(gca,'YLim',[49.995 50.02])
grid on   
% Print
print('Calculation_1mass_system','-depsc')
