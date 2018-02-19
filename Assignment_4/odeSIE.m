function [ T, Y ] = odeSIE( fun1, fun2, tspan, y0 )
%ODESIE Simple integration using Semi-Implicit Euler
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

% compute the solution
for i = 2:n
    Y(2, i) = Y(2, i-1) + dt * fun2(T(i-1), Y(1, i-1));
    Y(1, i) = Y(1, i-1) + dt * fun1(T(i-1), Y(2, i));
end

end

