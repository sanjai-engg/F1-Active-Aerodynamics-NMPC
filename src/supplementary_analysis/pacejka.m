% % % % % Generalized Magic Formula Tire Model Characteristics
% % % % clear; clc;
% % % % 
% % % % % Define the range of input slip angles (degrees)
% % % % X = -20:0.1:20; 
% % % % 
% % % % % Define three different normal loads (Newtons)
% % % % Fz_loads = [3000, 5000, 7000];
% % % % 
% % % % % Create a new figure and hold for multiple plots
% % % % figure('Name', 'Magic Formula Tire Characteristics', 'Color', 'w');
% % % % hold on; grid on;
% % % % 
% % % % % Loop through each normal load to calculate and plot characteristics
% % % % for i = 1:length(Fz_loads)
% % % %     Fz = Fz_loads(i);
% % % % 
% % % %     % Generalized empirical relations for parameters based on normal load Fz.
% % % %     % In a real application (like Q1g), these are defined by specific polynomials.
% % % %     D = 1.1 * Fz;               % Peak factor (roughly proportional to load)
% % % %     C = 1.3;                    % Shape factor (typically ~1.3 for lateral force)
% % % %     BCD = 15 * Fz;              % Cornering stiffness dependency on load
% % % %     B = BCD / (C * D);          % Stiffness factor
% % % %     E = -0.5;                   % Curvature factor
% % % %     Sh = 0;                     % Horizontal shift
% % % %     Sv = 0;                     % Vertical shift
% % % % 
% % % %     % Apply horizontal shift
% % % %     x = X + Sh;
% % % % 
% % % %     % Magic Formula Equation
% % % %     Y = D * sin(C * atan(B.*x - E.*(B.*x - atan(B.*x)))) + Sv;
% % % % 
% % % %     % Plot the characteristic curve
% % % %     plot(X, Y, 'LineWidth', 2, 'DisplayName', sprintf('F_z = %.1f kN', Fz/1000));
% % % % end
% % % % 
% % % % % Formatting the plot for the report
% % % % title('Tire Lateral Force vs. Slip Angle at Various Normal Loads');
% % % % xlabel('Slip Angle, \alpha (Degrees)');
% % % % ylabel('Lateral Force, F_y (Newtons)');
% % % % legend('Location', 'SouthEast');
% % % % hold off;
% % % 
% % % % ME5670 Assignment 2 - Q2(c)
% % % % Stability Condition and Safe Slip Ratio Analysis
% % % clear; clc;
% % % 
% % % % Define slip ratio range
% % % lambda = 0:0.001:0.4; 
% % % 
% % % % Define Magic Formula approximated parameters for Dry and Icy roads
% % % % Mu(lambda) = D * sin(C * atan(B * lambda))
% % % % Parameters: [B, C, D]
% % % params_dry = [15, 1.4, 0.85]; 
% % % params_ice = [30, 1.2, 0.15]; 
% % % 
% % % % Calculate Mu for both conditions
% % % mu_dry = params_dry(3) * sin(params_dry(2) * atan(params_dry(1) * lambda));
% % % mu_ice = params_ice(3) * sin(params_ice(2) * atan(params_ice(1) * lambda));
% % % 
% % % % Calculate the stability derivative (d(mu)/d(lambda)) using numerical gradient
% % % dmu_dlambda_dry = gradient(mu_dry, lambda);
% % % dmu_dlambda_ice = gradient(mu_ice, lambda);
% % % 
% % % % Find Safe Slip Ratios (Peak Mu where derivative crosses 0)
% % % [~, peak_idx_dry] = max(mu_dry);
% % % lambda_safe_dry = lambda(peak_idx_dry);
% % % 
% % % [~, peak_idx_ice] = max(mu_ice);
% % % lambda_safe_ice = lambda(peak_idx_ice);
% % % 
% % % % --- Plotting ---
% % % figure('Name', 'Longitudinal Stability Conditions', 'Color', 'w');
% % % 
% % % % Plot Dry Pavement
% % % subplot(2,1,1);
% % % hold on; grid on;
% % % plot(lambda, mu_dry, 'k', 'LineWidth', 2);
% % % plot(lambda, dmu_dlambda_dry, 'b--', 'LineWidth', 1.5);
% % % xline(lambda_safe_dry, 'r', 'LineWidth', 1.5, 'Label', 'Safe Slip Limit');
% % % title('Dry Pavement Stability Condition');
% % % xlabel('Longitudinal Slip Ratio (\lambda)');
% % % ylabel('\mu and d\mu/d\lambda');
% % % legend('\mu (Adhesion)', 'Stability Gradient (d\mu/d\lambda)');
% % % text(0.05, 2, 'STABLE (d\mu/d\lambda > 0)', 'Color', 'b', 'FontWeight', 'bold');
% % % text(0.25, 2, 'UNSTABLE (d\mu/d\lambda < 0)', 'Color', 'r', 'FontWeight', 'bold');
% % % hold off;
% % % 
% % % % Plot Icy Road
% % % subplot(2,1,2);
% % % hold on; grid on;
% % % plot(lambda, mu_ice, 'k', 'LineWidth', 2);
% % % plot(lambda, dmu_dlambda_ice, 'b--', 'LineWidth', 1.5);
% % % xline(lambda_safe_ice, 'r', 'LineWidth', 1.5, 'Label', 'Safe Slip Limit');
% % % title('Icy Road Stability Condition');
% % % xlabel('Longitudinal Slip Ratio (\lambda)');
% % % ylabel('\mu and d\mu/d\lambda');
% % % legend('\mu (Adhesion)', 'Stability Gradient (d\mu/d\lambda)');
% % % hold off;
% % % 
% % % fprintf('Dry Road Safe Slip Ratio: %.3f\n', lambda_safe_dry);
% % % fprintf('Icy Road Safe Slip Ratio: %.3f\n', lambda_safe_ice);
% % 
% % % ME5670 Assignment 2 - Q4(a)
% % % Steady State Gains Calculation
% % clear; clc;
% % 
% % % Assumed Vehicle Parameters (Can be updated with specific values later)
% % m = 1500;           % Mass (kg)
% % L = 2.6;            % Wheelbase (m)
% % a = 1.2;            % CG to front axle (m)
% % b = L - a;          % CG to rear axle (m)
% % Cf = 60000;         % Front axle cornering stiffness (N/rad)
% % Cr = 65000;         % Rear axle cornering stiffness (N/rad)
% % 
% % % Velocity range from 0 to 40 m/s (approx 144 km/hr)
% % V = 0:1:40; 
% % 
% % % Calculate Understeer Gradient (Kus)
% % Kus = (m / L) * ((b / Cf) - (a / Cr));
% % fprintf('Understeer Gradient (Kus) = %.4f rad/(m/s^2)\n', Kus);
% % 
% % % Calculate Gains
% % Path_Gain = 1 ./ (L + Kus .* V.^2);             % (1/m)/rad
% % Accel_Gain = (V.^2) ./ (L + Kus .* V.^2);       % (m/s^2)/rad
% % 
% % % Plotting the Gains
% % figure('Name', 'Steady State Cornering Gains', 'Color', 'w');
% % 
% % subplot(2,1,1);
% % plot(V, Path_Gain, 'b', 'LineWidth', 2);
% % title('Path Curvature Gain vs. Velocity');
% % xlabel('Velocity (m/s)');
% % ylabel('Path Gain (1/R) / \delta');
% % grid on;
% % 
% % subplot(2,1,2);
% % plot(V, Accel_Gain, 'r', 'LineWidth', 2);
% % title('Lateral Acceleration Gain vs. Velocity');
% % xlabel('Velocity (m/s)');
% % ylabel('Acceleration Gain (a_y / \delta)');
% % grid on;
% 
% % ME5670 Assignment 2 - Q4(c)
% % Steady State Gains using Stability Derivatives
% clear; clc;
% 
% % Define symbolic variables for the derivatives
% syms m U Y_beta Y_r Y_delta N_beta N_r N_delta delta
% 
% % Define the steady-state system matrix A and input vector B
% A = [-Y_beta, (m*U - Y_r); 
%      -N_beta, -N_r];
% 
% B = [Y_delta; 
%      N_delta] * delta;
% 
% % Solve for the state vector X = [beta; r]
% % A * X = B  =>  X = inv(A) * B
% X = A \ B;
% 
% beta_ss = simplify(X(1));
% r_ss = simplify(X(2));
% 
% % Display Yaw Rate Gain (r/delta)
% Yaw_Rate_Gain = r_ss / delta;
% disp('Steady State Yaw Rate Gain (r/delta):');
% pretty(Yaw_Rate_Gain)

% ME5670 Assignment 2 - Q4(e)
% Stability Derivatives and Steady-State Gains vs. Velocity
clear; clc; close all;

% 1. Vehicle Parameters
m = 830;            % Mass (kg)
a = 0.87;           % CG to front axle (m)
b = 1.29;           % CG to rear axle (m)
L = a + b;          % Wheelbase (m)
Cf = 67369;         % Front cornering stiffness (N/rad)
Cr = 63411;         % Rear cornering stiffness (N/rad)
Mz1 = 2010;         % Front aligning torque stiffness (Nm/rad)
Mz2 = 1366;         % Rear aligning torque stiffness (Nm/rad)

% Aerodynamic Parameters
rho = 1.23;         % Air density (kg/m^3)
S = 1.7;            % Frontal area (m^2)
Cy_beta = -2.2;     % Lateral force coefficient
CMz_beta = -0.6;    % Yaw moment coefficient

% 2. Velocity Array (Avoid zero to prevent division by zero in Yr and Nr)
U = 1:0.5:50;       % Speed from 1 m/s to 50 m/s (~180 km/hr)

% 3. Calculate Stability Derivatives as functions of U
q = 0.5 * rho .* U.^2; % Dynamic Pressure

Y_beta  = -(Cf + Cr) + q .* S .* Cy_beta;
Y_r     = (1./U) .* (b * Cr - a * Cf);
Y_delta = Cf * ones(size(U)); 

N_beta  = -(a * Cf + Mz1) + (b * Cr - Mz2) + q .* S .* L .* CMz_beta;
N_r     = -(1./U) .* (a * (a * Cf + Mz1) + b * (b * Cr - Mz2));
N_delta = (a * Cf + Mz1) * ones(size(U));

% 4. Calculate Steady State Gains
% Determinant of the system matrix
Delta = Y_beta .* N_r + N_beta .* (m .* U - Y_r);

% Gains
Yaw_Gain   = (N_beta .* Y_delta - Y_beta .* N_delta) ./ Delta;
Slip_Gain  = (-Y_delta .* N_r - N_delta .* (m .* U - Y_r)) ./ Delta;
Accel_Gain = U .* Yaw_Gain;
Path_Gain  = (1./U) .* Yaw_Gain;

% 5. Plotting Results
figure('Name', 'Variation of Stability Derivatives', 'Color', 'w', 'Position', [100, 100, 800, 600]);
subplot(2,2,1); plot(U, Y_beta, 'b', 'LineWidth', 2); title('Y_\beta vs Speed'); xlabel('U (m/s)'); ylabel('N/rad'); grid on;
subplot(2,2,2); plot(U, Y_r, 'r', 'LineWidth', 2); title('Y_r vs Speed'); xlabel('U (m/s)'); ylabel('Ns/rad'); grid on;
subplot(2,2,3); plot(U, N_beta, 'b', 'LineWidth', 2); title('N_\beta vs Speed'); xlabel('U (m/s)'); ylabel('Nm/rad'); grid on;
subplot(2,2,4); plot(U, N_r, 'r', 'LineWidth', 2); title('N_r vs Speed'); xlabel('U (m/s)'); ylabel('Nms/rad'); grid on;

figure('Name', 'Vehicle Steady-State Gains', 'Color', 'w', 'Position', [150, 150, 800, 600]);
subplot(2,2,1); plot(U, Yaw_Gain, 'k', 'LineWidth', 2); title('Yaw Rate Gain (r/\delta)'); xlabel('U (m/s)'); ylabel('(rad/s)/rad'); grid on;
subplot(2,2,2); plot(U, Slip_Gain, 'm', 'LineWidth', 2); title('Side Slip Gain (\beta/\delta)'); xlabel('U (m/s)'); ylabel('rad/rad'); grid on;
subplot(2,2,3); plot(U, Accel_Gain, 'g', 'LineWidth', 2); title('Lateral Accel Gain (a_y/\delta)'); xlabel('U (m/s)'); ylabel('(m/s^2)/rad'); grid on;
subplot(2,2,4); plot(U, Path_Gain, 'c', 'LineWidth', 2); title('Path Curvature Gain ((1/R)/\delta)'); xlabel('U (m/s)'); ylabel('(1/m)/rad'); grid on;