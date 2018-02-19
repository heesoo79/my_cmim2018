function [ T, Y ] = odeRK4( fun, tspan, y0 )
%ODERK4 Simple integration using Runge-Kutta Method (fourth-order formula)
%   fun - function handle with interface fun(t, y)
%   tspan - two element vector with dt and tend
%   y0 - initial conditions

% time step
dt = tspan(1);


T = 0:dt:tspan(2);

n = length(T);

Y = zeros(length(y0), n);

% populate initial conditions
Y(:, 1) = y0(:);
k1 = zeros(size(y0,2),1);
k2 = zeros(size(y0,2),1);
k3 = zeros(size(y0,2),1);
k4 = zeros(size(y0,2),1);

% compute the solution
for i = 2:n
    k1 = dt*fun(T(i-1), Y(:, i-1));
    k2 = dt*fun(T(i-1) + 0.5*dt, Y(:, i-1) + 0.5*k1);
    k3 = dt*fun(T(i-1) + 0.5*dt, Y(:, i-1) + 0.5*k2);
    k4 = dt*fun(T(i-1) + dt, Y(:, i-1) + k3);
    Y(:, i) = Y(:, i-1) + 1/6*k1 + 1/3*k2 + 1/3*k3 + 1/6*k4;
end
end

