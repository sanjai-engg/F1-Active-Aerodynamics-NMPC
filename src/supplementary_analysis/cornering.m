% --- ME5670: Cornering Conditions (State-Space Method) ---
clear; clc;

% Base Parameters
m = 1000;         
Iz = 1500;        
U = 25;           
Cf = 30000;       
Cr = 30000;       

%% 1. UNDERSTEER SIMULATION (CG Forward)
a = 1.0; 
b = 1.5;
A = [-(Cf+Cr)/(m*U), ((b*Cr-a*Cf)/(m*U))-1; -(a*Cf-b*Cr)/Iz, -(a^2*Cf+b^2*Cr)/(Iz*U)];
B = [Cf/(m*U); a*Cf/Iz]; 
C = [0, 1]; % We want the second state (Yaw Rate, r)
D = 0;
out1 = sim('Vehicle_Cornering2'); 
t_under = out1.tout;             
r_under = out1.simout;           

%% 2. NEUTRAL STEER SIMULATION (CG Centered)
a = 1.25; 
b = 1.25;
A = [-(Cf+Cr)/(m*U), ((b*Cr-a*Cf)/(m*U))-1; -(a*Cf-b*Cr)/Iz, -(a^2*Cf+b^2*Cr)/(Iz*U)];
B = [Cf/(m*U); a*Cf/Iz]; 
C = [0, 1]; 
D = 0;
out2 = sim('Vehicle_Cornering2');
t_neutral = out2.tout;
r_neutral = out2.simout;

%% 3. OVERSTEER SIMULATION (CG Rearward)
a = 1.5; 
b = 1.0;
A = [-(Cf+Cr)/(m*U), ((b*Cr-a*Cf)/(m*U))-1; -(a*Cf-b*Cr)/Iz, -(a^2*Cf+b^2*Cr)/(Iz*U)];
B = [Cf/(m*U); a*Cf/Iz]; 
C = [0, 1]; 
D = 0;
out3 = sim('Vehicle_Cornering2');
t_over = out3.tout;              
r_over = out3.simout;

%% 4. PLOTTING THE RESULTS
figure('Name', 'Handling Characteristics (Step Steer)', 'Color', 'w');
plot(t_under, r_under, 'b', 'LineWidth', 2); hold on;
plot(t_neutral, r_neutral, 'g', 'LineWidth', 2);
plot(t_over, r_over, 'r', 'LineWidth', 2);
grid on;

title('Vehicle Yaw Rate Response to Step Steer Input');
xlabel('Time (seconds)');
ylabel('Yaw Rate (rad/s)');
legend('Understeer (CG Forward)', 'Neutral Steer (CG Centered)', 'Oversteer (CG Rearward)', 'Location', 'NorthWest');

% Limit the y-axis so the oversteer spike doesn't ruin the scale
ylim([0, max(r_neutral)*2.5]); 
hold off;