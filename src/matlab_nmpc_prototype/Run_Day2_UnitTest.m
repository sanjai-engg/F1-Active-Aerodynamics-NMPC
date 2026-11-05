clear; clc;

%% NMPC SETUP
nx = 6;   % added integral state
ny = 4;   % added ey_int output
nu = 3;

nlobj = nlmpc(nx,ny,nu);

nlobj.Model.StateFcn  = "NMPC_StateFcn";
nlobj.Model.OutputFcn = "NMPC_OutputFcn";

nlobj.Ts = 0.05;
nlobj.PredictionHorizon = 30;
nlobj.ControlHorizon = 5;

%% INPUT CONSTRAINTS

% Steering
nlobj.MV(1).Min = -deg2rad(20);
nlobj.MV(1).Max =  deg2rad(20);

nlobj.MV(1).RateMin = -deg2rad(15);
nlobj.MV(1).RateMax =  deg2rad(15);

% Front wing
nlobj.MV(2).Min = 0;
nlobj.MV(2).Max = deg2rad(15);

% Rear wing
nlobj.MV(3).Min = 0;
nlobj.MV(3).Max = deg2rad(15);

%% WEIGHTS (INCLUDING INTEGRAL)

nlobj.Weights.OutputVariables = [500 300 150 800];

nlobj.Weights.ManipulatedVariables = [100 1 1];
nlobj.Weights.ManipulatedVariablesRate = [6 1 1];

disp("NMPC READY (WITH INTEGRAL)");

%% INITIAL STATE
x = [50; 0.5; 0; 0.1; 0; 0];   % added ey_int = 0

%% SIMULATION
Nsim = 100;
u = [0; 0; 0];

X_hist = zeros(nx,Nsim);
U_hist = zeros(nu,Nsim);

for k = 1:Nsim
    
    yref = [0 0 80 0];   % ey_int reference = 0
    
    u = nlmpcmove(nlobj, x, u, yref);
    
    xdot = NMPC_StateFcn(x,u);
    x = x + nlobj.Ts * xdot;
    
    X_hist(:,k) = x;
    U_hist(:,k) = u;
end

%% PLOTS
figure;

subplot(3,1,1)
plot(X_hist(1,:),'LineWidth',1.5)
title('Velocity vx'); grid on

subplot(3,1,2)
plot(X_hist(4,:),'LineWidth',1.5)
title('Cross-track error ey'); grid on

subplot(3,1,3)
plot(U_hist','LineWidth',1.5)
title('Inputs [delta, alpha_f, alpha_r]')
legend('\delta','\alpha_f','\alpha_r')
grid on