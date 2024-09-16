functio dxdt = parafoil_ode(t, x, u_func, wind_func, para_struct)
dxdt = zeros(x);

% Extract state variables
x = x(1);
y = x(2);
omega = x(3);

% Extract control inputs
ut = u_func(t);         % differential
wt = wind_func(t);      % wind speed, vWx, vWy, (Assumption: vWz==0)

% Extract parameters




x, y = {1, 2};