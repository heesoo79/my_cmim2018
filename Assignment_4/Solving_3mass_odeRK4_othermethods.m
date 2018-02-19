%% Calculation for 3 masses_spring system using odeRK4, ode45, ode15s, ode15s(modal coordinates)
clear all
close all
% test function  
m1 = 2;
m2 = 1.5;
m3 = 1;
k1 = 20000;
k2 = 15000;
k3 = 10000;
M = [m1 0 0
    0 m2 0
    0 0 m3]
K = [k1+k2 -k2 0
    -k2 k2+k3 -k3
    0 -k3 k3]
% 
test_fun = @(t, y) [ y(4:6); inv(M)*(-1)*K*y(1:3) ]
% modal coordinates
[V, D] = eig(K, M)   
eigenvalue = sqrt(D)
Mhat = V'*M*V
Khat = V'*K*V
test_fun1 = @(t, y) [ y(4:6) ; -D*y(1:3) ];
% solution params
dt = 1e-6;
tk = 1;
% initial conditions
y0 = [1, 1, 1, 0, 0, 0];
y0_modal(1:3,:) = inv(V)*y0(:,1:3)'% initial condition for modal coordinates
y0_modal(4:6,:) = inv(V)*y0(:,4:6)'% initial condition for modal coordinates
% solution of ODE
tic
[T1, Y1] = odeRK4(test_fun,[dt, tk], y0);
toc
tic
[T2, Y2] = ode45(test_fun,[dt, tk], y0);
toc
tic
[T3, Y3] = ode15s(test_fun,[dt, tk], y0);
toc
tic
[T4, YY] = ode15s(test_fun1,[dt, tk], y0_modal); % modal coordinates
Y4 = zeros(6,size(T4,1));
Y4(1:3,:) = V * YY(:,1:3)';
Y4(4:6,:) = V * YY(:,4:6)';
toc
%% difference between solutions
difference1 = (Y2(size(T2,1),1)-Y1(1,size(T1,2)))/Y1(1,size(T1,2))*100
difference2 = (Y3(size(T3,1),1)-Y1(1,size(T1,2)))/Y1(1,size(T1,2))*100
difference3 = (Y4(1,size(T4,1))-Y1(1,size(T1,2)))/Y1(1,size(T1,2))*100
% plot
figure
set(gcf,'paperunits','inches')
set(gcf,'position',[5 5 800 300])
plot(T1, Y1(1,:));
hold on
plot(T2, Y2(:,1));
hold on
plot(T3, Y3(:,1));
hold on
plot(T4, Y4(1,:));
xlabel('Time [sec]','Fontsize',12)
ylabel('Displacement [m]','Fontsize',12)
grid on 

figure
set(gcf,'paperunits','inches')
set(gcf,'position',[5 5 800 300])
plot(T1, Y1(1,:));
hold on
plot(T2, Y2(:,1));
hold on
plot(T3, Y3(:,1));
hold on
plot(T4, Y4(1,:));
xlabel('Time [sec]','Fontsize',12)
ylabel('Displacement [m]','Fontsize',12)
set(gca,'XLim',[0.98 1])
set(gca,'YLim',[0.05 0.15])
grid on 
% print
print('Calculation_3masses_system','-depsc')
