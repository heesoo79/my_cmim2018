function dydt = forced(t,y)
m = 5;
zeta = 0.1;
k = 100;
c = 2*zeta*sqrt(m*k);
A = 1;

% omega = 0.02*t;
% omega = 0.05*t;
omega = 0.1*t;

dydt = [y(2);(((A/m)*sin(omega*t))-((c/m)*y(2))-((k/m)*y(1)))]; 
