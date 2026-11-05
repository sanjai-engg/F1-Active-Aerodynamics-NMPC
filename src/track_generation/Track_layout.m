% 130R Reference Trajectory Generator - Double Apex (Modern Layout)
clear; clc; close all;

%% 1. Define Track Segments (Distance in meters)
ds = 0.1; % 10cm resolution 

% Curvatures
R_apex1 = 85;   k1 = 1 / R_apex1;
R_apex2 = 300;  k2 = 1 / R_apex2;

% Segment Lengths
L_str1   = 300; % Backstretch
L_cloth1 = 20;  % Entry transition
L_arc1   = 20;  % Tight first apex
L_mid    = 50;  % Blend from 85m to 340m radius
L_arc2   = 100;  % Sweeping second apex
L_cloth2 = 40;  % Exit transition
L_str2   = 150; % Run-up to Casio Triangle

%% 2. Build Curvature Vector (\kappa)
% Create independent arrays for each segment's curvature
k_str1   = zeros(1, length(0:ds:L_str1));
k_cloth1 = linspace(0, k1, length(ds:ds:L_cloth1));
k_arc1   = k1 * ones(1, length(ds:ds:L_arc1));
k_mid    = linspace(k1, k2, length(ds:ds:L_mid)); % The bridging clothoid
k_arc2   = k2 * ones(1, length(ds:ds:L_arc2));
k_cloth2 = linspace(k2, 0, length(ds:ds:L_cloth2));
k_str2   = zeros(1, length(ds:ds:L_str2));

% Concatenate into one continuous curvature profile
kappa = [k_str1, k_cloth1, k_arc1, k_mid, k_arc2, k_cloth2, k_str2];
s = (0:length(kappa)-1) * ds;

%% 3. Integrate to find Heading (\psi) and Coordinates (X, Y)
psi = cumtrapz(s, kappa);  % Heading angle (radians)
X = cumtrapz(s, cos(psi)); % Global X (meters)
Y = cumtrapz(s, sin(psi)); % Global Y (meters)

%% 4. Package Data
TrackData = table(s', X', Y', psi', kappa', ...
    'VariableNames', {'Distance_s', 'X', 'Y', 'Heading_psi', 'Curvature_k'});

save('130R_DoubleApex_Trajectory.mat', 'TrackData');

%% 5. Plot the Double-Apex Layout
% figure('Name', '130R Modern Layout', 'Position', [100, 100, 1000, 400]);
figure('Name', '130R Modern Layout');
% subplot(1, 2, 1);
plot(X, Y, 'r', 'LineWidth', 2);
grid on; axis equal;
title('130R Double-Apex Coordinate Layout');
xlabel('Global X (m)'); ylabel('Global Y (m)');

% subplot(1, 2, 2);
% plot(s, kappa, 'b', 'LineWidth', 2);
% grid on;
% title('Curvature Profile (Note the Mid-Corner Step)');
% xlabel('Distance s (m)'); ylabel('Curvature \kappa (1/m)');