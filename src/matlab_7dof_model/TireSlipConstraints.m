function c = TireSlipConstraints(X, U, e, data, varargin)
    % Custom inequality constraints for NMPC — tire lateral slip limits
    % c <= 0 means constraint is satisfied

    % Extract states and inputs
    vx = X(:, 4); vy = X(:, 5); r  = X(:, 6);
    delta = U(:, 1); alpha_f_wing = U(:, 2); alpha_r_wing = U(:, 3);
    
    % *** FIX 1: lf/lr must match VehicleDynamics7DOF (params.a=1.95, params.b=1.65) ***
    lf = 1.95;   % was 1.2 — caused wrong slip angle calculation
    lr = 1.65;   % was 1.3

    % *** FIX 2: Velocity floor — prevents NaN in Jacobian when vx ≈ 0 ***
    VX_MIN = 1.0;
    vx_s = max(abs(vx), VX_MIN);   % Always positive for atan2 denominator

    % Smooth kinematic slip angles
    alpha_front = atan2((vy + lf .* r), vx_s) - delta;
    alpha_rear  = atan2((vy - lr .* r), vx_s);
    
    % F1 AERODYNAMICS: Grip envelope expands with downforce (velocity squared)
    aero_coefficient = 0.0002; 
    downforce_grip = aero_coefficient .* (vx_s.^2) .* (alpha_f_wing + alpha_r_wing);
    alpha_max = 0.15 + downforce_grip; 
    
    % DIFFERENTIABLE CONSTRAINTS (c <= 0 = satisfied)
    c = [
         alpha_front - alpha_max;
        -alpha_front - alpha_max;
         alpha_rear  - alpha_max;
        -alpha_rear  - alpha_max
    ];

    % *** FIX 3: Force column vector — fmincon requires this exact format ***
    c = c(:);
end