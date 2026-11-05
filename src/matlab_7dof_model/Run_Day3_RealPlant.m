% Day 4: Full 130R Simulation (SQP Active Solver)
clear; clc; close all;
disp('Initializing NMPC with 130R Track Data...');

%% 1. Controller Setup
nx = 10; ny = 10; nu = 4;
nlobj = nlmpc(nx, ny, nu);                  
nlobj.Model.StateFcn = "VehicleDynamics7DOF";
nlobj.Ts = 0.05;                            

% --- THE SPEED HACK: OPTIMIZED SQP SETTINGS ---
% 1. Compromise on Vision (30 steps = 1.5 seconds = ~97 meters of vision).
nlobj.PredictionHorizon = 30; 
nlobj.ControlHorizon = 3;     

% 2. Loosen the Perfectionism
nlobj.Optimization.SolverOptions.Algorithm = 'sqp'; 
nlobj.Optimization.SolverOptions.StepTolerance = 1e-2;       % Loosened!
nlobj.Optimization.SolverOptions.OptimalityTolerance = 1e-2; % Loosened!
nlobj.Optimization.SolverOptions.MaxIterations = 15;         % Hard-capped at 15!

% Actuator Constraints
nlobj.ManipulatedVariables(1).Min = -deg2rad(30); nlobj.ManipulatedVariables(1).Max = deg2rad(30);
nlobj.ManipulatedVariables(2).Min = 0; nlobj.ManipulatedVariables(2).Max = deg2rad(15);
nlobj.ManipulatedVariables(3).Min = 0; nlobj.ManipulatedVariables(3).Max = deg2rad(15);

nlobj.ManipulatedVariables(4).Min = -1.0; 
nlobj.ManipulatedVariables(4).Max = 1.0;
nlobj.ManipulatedVariables(4).RateMin = -0.2; 
nlobj.ManipulatedVariables(4).RateMax = 0.2;

nlobj.States(4).Min = 10; 

% Native Scaling
nlobj.ManipulatedVariables(1).ScaleFactor = 1;
nlobj.ManipulatedVariables(2).ScaleFactor = 1;
nlobj.ManipulatedVariables(3).ScaleFactor = 1;
nlobj.ManipulatedVariables(4).ScaleFactor = 5000;

% Weights
nlobj.Weights.OutputVariables = [100, 100, 50, 40, 2, 5, 0.1, 0.1, 0.1, 0.1];
nlobj.Weights.ManipulatedVariables = [0, 0, 0, 0.01];
nlobj.Weights.ManipulatedVariablesRate = [0.5, 1.0, 1.0, 0.1];

%% 2. Simulation Parameters
Ts = nlobj.Ts;
T_final = 15; 
N_steps = T_final / Ts;
x_history = zeros(N_steps, nx);
u_history = zeros(N_steps, nu);

%% 3. Initial Conditions
vx_init = 65; 
w_init = vx_init / 0.36; 
x0 = [0, 0, 0, vx_init, 0, 0, w_init, w_init, w_init, w_init]';
x = x0;
u = [0, 0, 0, 0]';

load('130R_DoubleApex_Trajectory.mat'); 
v_target = 65;

%% 4. Main Control Loop
mv = u;
options = nlmpcmoveopt;
P = nlobj.PredictionHorizon; 
yref_plot = zeros(N_steps, nx); 

disp('Running Closed-Loop Simulation (SQP Solver Active)...');
for k = 1:N_steps
    
    distances = sqrt((TrackData.X - x(1)).^2 + (TrackData.Y - x(2)).^2);
    [~, min_idx] = min(distances);
    current_s = TrackData.Distance_s(min_idx);
    
    if current_s >= TrackData.Distance_s(end) - 5
        disp('🏁 Finish Line Crossed! Terminating simulation safely...');
        x_history = x_history(1:k-1, :);
        u_history = u_history(1:k-1, :);
        yref_plot = yref_plot(1:k-1, :);
        T_final = (k-1) * Ts; 
        break;
    end
    
    ref_preview = zeros(P, nx);
    for j = 1:P
        target_s = current_s + (j * v_target * Ts);
        ref_preview(j, 1) = interp1(TrackData.Distance_s, TrackData.X, target_s, 'linear', 'extrap');
        ref_preview(j, 2) = interp1(TrackData.Distance_s, TrackData.Y, target_s, 'linear', 'extrap');
        ref_preview(j, 3) = interp1(TrackData.Distance_s, TrackData.Heading_psi, target_s, 'linear', 'extrap');
        ref_preview(j, 4) = v_target;
        ref_preview(j, 7:10) = v_target / 0.36; 
    end
    yref_plot(k, 1:4) = ref_preview(1, 1:4);
    
    [mv, options, ~] = nlmpcmove(nlobj, x, mv, ref_preview, [], options);
    u_history(k, :) = mv';
    
    dt_physics = 0.001; 
    num_substeps = Ts / dt_physics; 
    for sub = 1:num_substeps
        dx = VehicleDynamics7DOF(x, mv);
        x = x + dx * dt_physics; 
        if x(4) < 1; x(4) = 1; end
    end
    x_history(k, :) = x';
    
    if mod(k, 20) == 0; fprintf('Simulated %.2f seconds...\n', k*Ts); end
end
disp('Simulation Complete. Generating Plots...');

%% 5. Visualization
figure('Name', 'Day 4: Full 130R Simulation (RB19 Spec)', 'Position', [100, 100, 1000, 800]);
subplot(3,1,1);
plot(x_history(:, 1), x_history(:, 2), 'k-', 'LineWidth', 2); hold on;
plot(yref_plot(:, 1), yref_plot(:, 2), 'r--', 'LineWidth', 1.5); 
title('Top-Down Vehicle Trajectory vs 130R Target'); xlabel('Global X (m)'); ylabel('Global Y (m)'); grid on; axis equal;
legend('Car Trajectory', '130R Racing Line', 'Location', 'Best');
subplot(3,1,2);
plot(0:Ts:(T_final-Ts), x_history(:, 4), 'b-', 'LineWidth', 2); hold on;
yline(65, 'r--', 'Target Speed (234 km/h)');
title('Longitudinal Velocity (v_x)'); xlabel('Time (s)'); ylabel('Velocity (m/s)'); grid on;
subplot(3,1,3);
plot(0:Ts:(T_final-Ts), rad2deg(u_history(:, 2)), 'g-', 'LineWidth', 2); hold on;
plot(0:Ts:(T_final-Ts), rad2deg(u_history(:, 3)), 'm-', 'LineWidth', 2);
title('Active Aero Commands'); xlabel('Time (s)'); ylabel('Angle of Attack (Degrees)');
legend('Front Wing', 'Rear Wing'); grid on;