clear; clc;

nx = 5;   % [vx vy r ey epsi]
ny = 3;   % [ey epsi vx]
nu = 3;   % [delta alpha_f alpha_r]

nlobj = nlmpc(nx,ny,nu);

nlobj.Model.StateFcn = "NMPC_StateFcn";
nlobj.Model.OutputFcn = "NMPC_OutputFcn";

nlobj.Ts = 0.05;
nlobj.PredictionHorizon = 20;
nlobj.ControlHorizon = 5;

% --- INPUT LIMITS ---
nlobj.MV(1).Min = -deg2rad(30);
nlobj.MV(1).Max = deg2rad(30);

nlobj.MV(2).Min = 0;
nlobj.MV(2).Max = deg2rad(15);

nlobj.MV(3).Min = 0;
nlobj.MV(3).Max = deg2rad(15);

% --- COST FUNCTION ---
nlobj.Weights.OutputVariables = [200 100 -20];
nlobj.Weights.ManipulatedVariablesRate = [0.5 1 1];

disp("NMPC READY");