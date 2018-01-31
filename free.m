function dydt = free(t,y)
m = 5;
yeta = 0.1;
k = 100;
c = 2*yeta*sqrt(m*k);
dydt = [y(2); -1*c/m*y(2)-1*k/m*y(1)];