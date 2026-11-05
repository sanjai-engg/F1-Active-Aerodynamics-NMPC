function xdot = NMPC_StateFcn(x,u)

% ==============================
% STATES
% ==============================
vx     = x(1);
vy     = x(2);
r      = x(3);
ey     = x(4);
epsi   = x(5);
ey_int = x(6);   % integral state

% ==============================
% INPUTS
% ==============================
delta   = u(1);
alpha_f = u(2);
alpha_r = u(3);

% ==============================
% PARAMETERS
% ==============================
m = 820;
Iz = 1150;
a = 1.95;
b = 1.65;
rho = 1.225;
A = 3.0;
g = 9.81;

mu = 1.05;

vx_safe = max(vx,0.5);

% ==============================
% AERODYNAMICS
% ==============================
Cl = 35*(0.4*alpha_f + 0.6*alpha_r);
Cd = 0.5 + 1.5*(alpha_f^2 + alpha_r^2);

F_aero = 0.5 * rho * A * Cl * vx^2;
F_drag = 0.5 * rho * A * Cd * vx^2;

% ==============================
% NORMAL LOAD
% ==============================
Fz_total = m*g + F_aero;

Fzf = (b/(a+b)) * Fz_total;
Fzr = (a/(a+b)) * Fz_total;

% ==============================
% TIRE MODEL
% ==============================
Cf = 20000;
Cr = 25000;

alpha_f_slip = delta - (vy + a*r)/vx_safe;
alpha_r_slip = -(vy - b*r)/vx_safe;

Fyf = Cf * alpha_f_slip;
Fyr = Cr * alpha_r_slip;

% ==============================
% FRICTION LIMIT
% ==============================
Fy_front_max = mu * Fzf;
Fy_rear_max  = mu * Fzr;

Fyf = Fy_front_max * tanh(Fyf / Fy_front_max);
Fyr = Fy_rear_max  * tanh(Fyr / Fy_rear_max);

% ==============================
% TRACK CURVATURE
% ==============================
kappa = 1/140;
r_ref = vx * kappa;

% ==============================
% VEHICLE DYNAMICS
% ==============================
vx_dot = -0.6*F_drag/m + vy*r;
vy_dot = (Fyf + Fyr)/m - vx*r;
r_dot  = (a*Fyf - b*Fyr)/Iz - 0.3*r;

% ==============================
% ERROR DYNAMICS
% ==============================
ey_dot     = vy + vx_safe * epsi;
epsi_dot   = r - r_ref;
ey_int_dot = ey;   % integral

% ==============================
% OUTPUT
% ==============================
xdot = [vx_dot; vy_dot; r_dot; ey_dot; epsi_dot; ey_int_dot];

end