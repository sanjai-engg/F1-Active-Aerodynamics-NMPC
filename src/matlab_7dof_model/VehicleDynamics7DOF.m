function dxdt = VehicleDynamics7DOF(x, u)
    % 7-DOF Double-Track Vehicle Model (RB19 + Real Airfoil Polars)
    
    % --- Red Bull RB19 Specifications ---
    params.m = 820;           
    params.I = 1150;          
    params.a = 1.95;          
    params.b = 1.65;           
    params.c = 1.0;           
    params.g = 9.81;          
    params.rho = 1.225;       
    params.Area = 3.0;        % Massive area to simulate F1 Venturi Floor
    params.Cd = 0.50;         % Base chassis drag
    params.R_w = 0.36;        % Pirelli 18-inch tires
    params.Iw = 1.6;          
    params.Bd = 0.1;          
    params.mu = 1.60;         % High-grip C5 Soft Compound
    
    params.B_x = 12; params.C_x = 2.3; params.E_x = 1.0; 
    params.B_y = 12; params.C_y = 2.3; params.E_y = 1.0; 
    
    % 1. Unpack States
    X_pos = x(1); Y_pos = x(2); psi = x(3); 
    xdot = x(4); ydot = x(5); psidot = x(6);
    omega_w = x(7:10)'; 
    
    % 2. Pure Physical Inputs
    delta = u(1);
    T_cmd = u(4); % Pure Nm, handled by NMPC ScaleFactor now!
    T_net = [T_cmd/4, T_cmd/4, T_cmd/4, T_cmd/4]; 
    
    alpha_front_rad = u(2); 
    alpha_rear_rad  = u(3);
    
    % =========================================================================
    % --- SELIG S1223 & EPPLER E423 POLARS ---
    P_cl_front = [-51.1332, 9.06902, 5.66426, 1.22553];
    P_cd_front = [0.682896, -0.078203, 0.0146243];
    
    P_cl_rear  = [-62.6616, 9.04976, 5.28816, 1.10471];
    P_cd_rear  = [1.1672, -0.177174, 0.0143083];
    
    Cl_front = polyval(P_cl_front, alpha_front_rad);
    Cd_front = polyval(P_cd_front, alpha_front_rad);
    
    Cl_rear = polyval(P_cl_rear, alpha_rear_rad);
    Cd_rear = polyval(P_cd_rear, alpha_rear_rad);
    % =========================================================================
    
    delta_vec = [delta, delta, 0, 0]; 
    
    % 3. Kinematics & Slip
    vy = [ydot + params.a * psidot, ydot + params.a * psidot, ...
          ydot - params.b * psidot, ydot - params.b * psidot];
    vx = [xdot - params.c * psidot, xdot + params.c * psidot, ...
          xdot - params.c * psidot, xdot + params.c * psidot];
          
    vl_w = vy .* sin(delta_vec) + vx .* cos(delta_vec);
    vc_w = vy .* cos(delta_vec) - vx .* sin(delta_vec);
    
    s = (params.R_w .* omega_w - vl_w) ./ vx;
    alpha = atan(vc_w ./ vx);
    
    % 4. Aerodynamics & Normal Loads
    F_aero_front = 0.5 * params.rho * Cl_front * params.Area * xdot^2;
    F_aero_rear  = 0.5 * params.rho * Cl_rear  * params.Area * xdot^2;
    
    Cd_dynamic = params.Cd + Cd_front + Cd_rear;
    F_drag = 0.5 * params.rho * Cd_dynamic * params.Area * xdot^2;
    
    Fzf_static = (params.b * params.m * params.g) / (2 * (params.a + params.b));
    Fzr_static = (params.a * params.m * params.g) / (2 * (params.a + params.b));
    
    Fz_vec = [Fzf_static + F_aero_front/2, Fzf_static + F_aero_front/2, ...
              Fzr_static + F_aero_rear/2,  Fzr_static + F_aero_rear/2];
              
    % 5. Pacejka Tire Forces
    D_vec = params.mu .* Fz_vec; 
    Fl = D_vec .* sin(params.C_x * atan(params.B_x .* s - params.E_x * (params.B_x .* s - atan(params.B_x .* s))));
    Fc = D_vec .* sin(params.C_y * atan(params.B_y .* alpha - params.E_y * (params.B_y .* alpha - atan(params.B_y .* alpha))));
    
    Fx = Fl .* cos(delta_vec) - Fc .* sin(delta_vec);
    Fy = Fl .* sin(delta_vec) + Fc .* cos(delta_vec);
    
    % 6. Equations of Motion
    xddot = (1/params.m) * (params.m * ydot * psidot + sum(Fx) - F_drag);
    yddot = (1/params.m) * (-params.m * xdot * psidot + sum(Fy));
    
    psiddot = (1/params.I) * (params.a * (Fy(1) + Fy(2)) - params.b * (Fy(3) + Fy(4)) ...
              + params.c * (-Fx(1) + Fx(2) - Fx(3) + Fx(4)));
              
    Xdot = xdot * cos(psi) - ydot * sin(psi);
    Ydot = xdot * sin(psi) + ydot * cos(psi);
    
    omega_wdot = (T_net - Fl .* params.R_w - params.Bd .* omega_w) ./ params.Iw;
    
    dxdt = [Xdot; Ydot; psidot; xddot; yddot; psiddot; omega_wdot'];
end