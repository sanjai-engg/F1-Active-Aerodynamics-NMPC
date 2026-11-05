% Airfoil_Setup.m
clear; clc;

% 1. Load Data (Skipping the 2 header lines)
front_data = readmatrix('SeligS1223.txt', 'NumHeaderLines', 0);
rear_data  = readmatrix('Eppler_E423.txt', 'NumHeaderLines', 0);

% 2. Isolate the operational window (0 to 15 degrees)
f_idx = front_data(:,1) >= 0 & front_data(:,1) <= 15;
r_idx = rear_data(:,1)  >= 0 & rear_data(:,1)  <= 15;

% 3. Convert Alpha to Radians (CRITICAL FOR NMPC)
alpha_front_rad = deg2rad(front_data(f_idx, 1));
cl_front = front_data(f_idx, 2);
cd_front = front_data(f_idx, 3);

alpha_rear_rad = deg2rad(rear_data(r_idx, 1));
cl_rear = rear_data(r_idx, 2);
cd_rear = rear_data(r_idx, 3);

% 4. Polynomial Regression (3rd order for Lift, 2nd order for Drag)
P_cl_front = polyfit(alpha_front_rad, cl_front, 3);
P_cd_front = polyfit(alpha_front_rad, cd_front, 2);

P_cl_rear = polyfit(alpha_rear_rad, cl_rear, 3);
P_cd_rear = polyfit(alpha_rear_rad, cd_rear, 2);

% 5. Print the coefficients for the Plant model
fprintf('--- COPY AND PASTE THESE INTO VehicleDynamics7DOF.m ---\n\n');
fprintf('P_cl_front = [%g, %g, %g, %g];\n', P_cl_front);
fprintf('P_cd_front = [%g, %g, %g];\n', P_cd_front);
fprintf('P_cl_rear  = [%g, %g, %g, %g];\n', P_cl_rear);
fprintf('P_cd_rear  = [%g, %g, %g];\n\n', P_cd_rear);
fprintf('-------------------------------------------------------\n');