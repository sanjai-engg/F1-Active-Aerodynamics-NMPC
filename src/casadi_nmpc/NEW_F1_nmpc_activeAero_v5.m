% =========================================================================
% F1 CAR – NMPC CLOSED-LOOP SIMULATION  (v7 – Physics-Honest Speed)
% 130R Corner, Suzuka Circuit
%
% Root cause of v6 flat-speed bug:
%   Hard constraint  Vx >= 55 m/s  was ACTIVE and BINDING.
%   IPOPT simultaneously commanded -1500 Nm braking AND enforced Vx >= 55.
%   These are contradictory, so IPOPT relaxed the dynamics equality
%   (acceptable_tol=1e-2 allows this) and the speed appeared "stuck" at
%   200 km/h even though braking torque was at the limit.
%
% Fix: Remove hard Vx lower bound entirely.
%      Replace with a ONE-SIDED soft cost penalty (smooth hinge via fmax).
%      Now braking genuinely slows the car — speed is determined by physics,
%      not by a constraint wall.
%
% Summary of all changes from v6:
%  [v7-1] REMOVED hard  Vx >= 55  constraint — was fighting dynamics
%  [v7-2] Speed-track cost made ONE-SIDED — only penalises Vx < V_floor
%          (symmetric cost fought against -w_v*sum(Vx) reward)
%  [v7-3] V_floor = 55 m/s (198 km/h) as soft lower, V_target = 75 for reward
%  [v7-4] w_vfloor = 50 — strong soft penalty below 198 km/h
%  [v7-5] acceptable_tol tightened 1e-2 → 1e-3 — less dynamics relaxation
%  [v7-6] T_min_brake = -2500 Nm (now it will genuinely slow the car)
% =========================================================================

clear; close all; clc;
import casadi.*

%% ── 1. F1 VEHICLE PARAMETERS ─────────────────────────────────────────────
veh.m               = 800;
veh.Iz              = 900;
veh.lf              = 1.8;    veh.lr = 1.8;
veh.L               = 3.6;
veh.tf              = 1.6;    veh.tr = 1.5;
veh.h_cg            = 0.30;
veh.R_wheel         = 0.33;
veh.I_wheel         = 2.0;

veh.P_max           = 750000;          % 750 kW
veh.T_max_drive     = 2000;            % Nm
veh.T_min_brake     = -2500;           % [v7-6] Now that speed is physics-driven,
                                       %         braking will actually slow the car
veh.brake_bias      = 0.58;

veh.rho             = 1.225;
veh.A_wing          = 0.5;
veh.AR_f = 3.0;  veh.AR_r = 2.5;
veh.e_f  = 0.85; veh.e_r  = 0.85;
veh.A_body          = 1.5;
veh.CD_body         = 0.85;
veh.CL_body         = 2.2;
veh.CL_body_f_frac  = 0.40;
veh.Crr             = 0.010;
veh.roll_stiffness_f= 0.50;

tire.B = 12;  tire.C = 1.7;  tire.E = -0.5;
tire.mu_max = 1.85;  tire.mu_load_sens = 0.00003;

%% ── 2. POLYNOMIAL AERO FITS ──────────────────────────────────────────────
if ~isfile('SeligS1223.txt')
    aoa = (-5:2:22)';
    writematrix([aoa, max(-0.3,min(2.5,0.12*aoa)), ...
        0.012+0.003*aoa.^2/10, zeros(numel(aoa),4)], ...
        'SeligS1223.txt','Delimiter','\t');
end
if ~isfile('Eppler_E423.txt')
    aoa = (-5:2:25)';
    writematrix([aoa, max(-0.2,min(3.0,0.13*aoa)), ...
        0.010+0.0025*aoa.^2/10, zeros(numel(aoa),4)], ...
        'Eppler_E423.txt','Delimiter','\t');
end

fw_raw = load('SeligS1223.txt'); rw_raw = load('Eppler_E423.txt');
[fw_alpha,fi] = sort(fw_raw(:,1)); [rw_alpha,ri] = sort(rw_raw(:,1));
fw_CL = fw_raw(fi,2); fw_CD = fw_raw(fi,3);
rw_CL = rw_raw(ri,2); rw_CD = rw_raw(ri,3);

p_fw_CL = polyfit(fw_alpha, fw_CL, 2);  p_fw_CD = polyfit(fw_alpha, fw_CD, 2);
p_rw_CL = polyfit(rw_alpha, rw_CL, 2);  p_rw_CD = polyfit(rw_alpha, rw_CD, 2);

aero.p_fw_CL = p_fw_CL; aero.p_fw_CD = p_fw_CD;
aero.p_rw_CL = p_rw_CL; aero.p_rw_CD = p_rw_CD;
aero.fw_alpha_lim = [min(fw_alpha), max(fw_alpha)];  % degrees
aero.rw_alpha_lim = [min(rw_alpha), max(rw_alpha)];  % degrees

%% ── 3. TRACK DATA ────────────────────────────────────────────────────────
if isfile('130R_DoubleApex_Trajectory.mat')
    load('130R_DoubleApex_Trajectory.mat');
    s_ref = TrackData.Distance_s;  kappa_ref = TrackData.Curvature_k;
else
    s_ref     = (0:1.0:1200)';
    blend     = @(s,s0,w) 0.5*(1+tanh((s-s0)/w));
    kappa_ref = (1/130)*(blend(s_ref,100,25)-blend(s_ref,600,25));
end

%% ── 4. NMPC PROBLEM SETUP ────────────────────────────────────────────────
N  = 30;
dt = 0.05;

opti       = casadi.Opti();
X          = opti.variable(10, N+1);
U          = opti.variable(4,  N);
X0_par     = opti.parameter(10, 1);
kappa_par  = opti.parameter(1,  N);
U_prev_par = opti.parameter(4,  1);

% Slew rates — tight to prevent oscillation
slew_T  = 300;    % Nm/step
slew_d  = 0.015;  % rad/step
slew_af = 0.5;    % deg/step
slew_ar = 0.5;    % deg/step

% ── Cost weights ──────────────────────────────────────────────────────────
w_s      = 20.0;   % terminal arc-length reward
w_v      = 15.0;   % integral speed reward  (pushes Vx up everywhere)
w_ey     = 20;     % lateral tracking
w_epsi   = 8;      % heading error (low — prevents brake-to-correct)

% [v7-2,3,4] ONE-SIDED speed floor — only fires when Vx drops below V_floor
% This replaces the hard constraint AND the old symmetric w_vtrack cost.
% The car is FREE to brake below V_floor (the soft cost just discourages it),
% but IPOPT can no longer hide braking by relaxing dynamics.
V_floor  = 55.0;   % m/s = 198 km/h  — soft lower bound
w_vfloor = 50.0;   % penalty weight below V_floor

% [v7-2] Speed REWARD target — separate from the floor penalty
% -w_v*sum(Vx) already rewards high speed everywhere.
% No separate symmetric tracking cost needed — it fought w_v.

% W_dU — correct order [T_cmd, delta, af, ar]
W_dU = diag([80, 2.0, 2000, 2000]);

% Symbolic state/input extraction
s_sym    = X(1,:); ey_sym  = X(2,:); epsi_sym = X(3,:); Vx_sym = X(4,:);
T_sym    = U(1,:); d_sym   = U(2,:); af_sym   = U(3,:); ar_sym = U(4,:);

% ── Cost function ─────────────────────────────────────────────────────────
cost = - w_s * s_sym(N+1)           ...  % maximise final arc-length
       - w_v * sum(Vx_sym)          ...  % maximise speed at every step
       + w_ey   * sumsqr(ey_sym)    ...  % penalise lateral error
       + w_epsi * sumsqr(epsi_sym)  ...  % penalise heading error
       + 10*w_ey * ey_sym(N+1)^2;        % terminal lateral push

% [v7-1,2] ONE-SIDED soft speed floor via smooth hinge
%   fmax(0, V_floor - Vx)  is positive ONLY when Vx < V_floor
%   Squaring gives a smooth quadratic penalty that ramps up as speed drops
%   IPOPT can differentiate fmax cleanly — no kink in gradient
for k = 1:N+1
    speed_deficit = fmax(0, V_floor - Vx_sym(k));   % 0 when fast, positive when slow
    cost = cost + w_vfloor * speed_deficit^2;
end

% Slew-rate cost and hard rate constraints
for k = 0:N-1
    if k == 0
        dU = U(:,1) - U_prev_par;
    else
        dU = U(:,k+1) - U(:,k);
    end
    cost = cost + dU' * W_dU * dU;
    opti.subject_to(-slew_T  <= dU(1) <= slew_T);
    opti.subject_to(-slew_d  <= dU(2) <= slew_d);
    opti.subject_to(-slew_af <= dU(3) <= slew_af);
    opti.subject_to(-slew_ar <= dU(4) <= slew_ar);
end
opti.minimize(cost);

% ── RK2 Dynamics (equality constraints) ──────────────────────────────────
for k = 1:N
    kap = kappa_par(k);
    k1  = calc_dynamics(X(:,k),          U(:,k), kap, veh, tire, aero);
    k2  = calc_dynamics(X(:,k)+dt/2*k1,  U(:,k), kap, veh, tire, aero);
    opti.subject_to(X(:,k+1) == X(:,k) + dt*k2);
    opti.subject_to(1 - kappa_par(k)*ey_sym(k) >= 0.1);
end

% ── Box constraints ───────────────────────────────────────────────────────
opti.subject_to(-6.5  <= ey_sym  <= 6.5);

% [v7-1] NO LOWER BOUND on Vx — removed hard Vx >= 55 constraint
%         Speed floor is now enforced only via the soft cost above.
%         This means braking genuinely changes Vx — physics is honest.
opti.subject_to(Vx_sym  <= 100.0);            % only upper bound remains

opti.subject_to(-0.30 <= d_sym   <= 0.30);
opti.subject_to(veh.T_min_brake <= T_sym <= veh.T_max_drive);
opti.subject_to(aero.fw_alpha_lim(1) <= af_sym <= aero.fw_alpha_lim(2));
opti.subject_to(aero.rw_alpha_lim(1) <= ar_sym <= aero.rw_alpha_lim(2));

% Power constraint (prevents over-torque at high speed)
for k = 1:N
    opti.subject_to(T_sym(k)*(X(4,k)/veh.R_wheel) <= veh.P_max);
end

opti.subject_to(X(:,1) == X0_par);

% ── Solver options ────────────────────────────────────────────────────────
p_opts = struct('expand',true,'print_time',false);
s_opts = struct('max_iter',100, ...
                'print_level',0, ...
                'tol',5e-4, ...
                'acceptable_tol',1e-3, ...   % [v7-5] tighter than v6 (was 1e-2)
                'acceptable_iter',5, ...     % fewer iterations at acceptable tol
                'warm_start_init_point','yes', ...
                'mu_init',1e-2, ...
                'nlp_scaling_method','gradient-based');
opti.solver('ipopt', p_opts, s_opts);

%% ── 5. COMPILE NLP ONCE ──────────────────────────────────────────────────
fprintf('Compiling NLP (N=%d)... ', N);
tic;
nmpc_fn = opti.to_function('nmpc_fn', ...
    {X0_par,  kappa_par,  U_prev_par,  X,  U}, ...
    {U,       X});
fprintf('done in %.2f s\n\n', toc);

%% ── 6. CLOSED-LOOP SIMULATION ────────────────────────────────────────────
sim_steps = 300;
X_hist = zeros(10, sim_steps+1);
U_hist = zeros(4,  sim_steps);

V0  = 77.0;               % m/s = 277 km/h — realistic 130R entry speed
om0 = V0 / veh.R_wheel;
x0  = [s_ref(1); 0; 0; V0; 0; 0; om0; om0; om0; om0];
X_hist(:,1) = x0;

% Warm-start with wings deployed at realistic cornering angle
U_prev_applied = [1200; 0; 10; 12];  % [T_cmd, delta, af_deg, ar_deg]
X_guess = repmat(x0, 1, N+1);
U_guess = repmat(U_prev_applied, 1, N);

cur_X = x0;
consecutive_fails = 0;

fprintf('Running closed-loop simulation (%d steps)...\n', sim_steps);
fprintf('  Step | s (m)  |  V km/h  |  af °  |  ar °  |  T Nm\n');
fprintf('  -----|--------|----------|--------|--------|-------\n');

for i = 1:sim_steps
    s_now = cur_X(1);
    v_now = max(cur_X(4), 20.0);
    s_end = s_now + max(v_now*(N*dt), 60.0);

    kappa_vals = interp1(s_ref, kappa_ref, ...
                         linspace(s_now, s_end, N), 'nearest','extrap');

    try
        [U_val, X_val] = nmpc_fn(cur_X, kappa_vals, U_prev_applied, X_guess, U_guess);
        U_opt = full(U_val);
        X_opt = full(X_val);
        consecutive_fails = 0;
    catch ME
        consecutive_fails = consecutive_fails + 1;
        warning('Step %d solver failed (%d consec): %s', i, consecutive_fails, ME.message);
        U_opt = U_guess;
        X_opt = X_guess;
        if consecutive_fails >= 3
            % Reset guess to current physical state — prevents drift
            X_guess = repmat(cur_X, 1, N+1);
            U_guess = repmat(U_prev_applied, 1, N);
            consecutive_fails = 0;
            fprintf('  Guess reset at step %d.\n', i);
        end
    end

    U_hist(:,i)    = U_opt(:,1);
    U_prev_applied = U_opt(:,1);
    cur_X          = X_opt(:,2);
    X_hist(:,i+1)  = cur_X;

    % Shift optimal trajectory — warm-start next step
    X_guess = [X_opt(:,2:end), X_opt(:,end)];
    U_guess = [U_opt(:,2:end), U_opt(:,end)];

    if mod(i,20)==0 || i==1
        fprintf('  %4d | %6.1f | %8.1f | %6.1f | %6.1f | %6.0f\n', ...
            i, cur_X(1), cur_X(4)*3.6, U_opt(3,1), U_opt(4,1), U_opt(1,1));
    end
end

%% ── 7. PLOTS ─────────────────────────────────────────────────────────────
t_ctrl = (0:sim_steps-1)*dt;
t_hist = (0:sim_steps)*dt;
s_hist = X_hist(1,:);
V_hist = X_hist(4,:);

[~, apex_ref_idx] = max(abs(kappa_ref));
s_apex = s_ref(apex_ref_idx);

psi_r  = cumtrapz(s_ref, kappa_ref);
X_ref  = cumtrapz(s_ref, cos(psi_r));
Y_ref  = cumtrapz(s_ref, sin(psi_r));
psi_s  = interp1(s_ref, psi_r, s_hist,'linear','extrap');
X_car  = interp1(s_ref, X_ref, s_hist,'linear','extrap') - X_hist(2,:).*sin(psi_s);
Y_car  = interp1(s_ref, Y_ref, s_hist,'linear','extrap') + X_hist(2,:).*cos(psi_s);

figure('Name','F1 NMPC v7','Position',[50 50 1400 800]);
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

% 1. Speed profile
nexttile;
plot(s_hist, V_hist*3.6, 'b','LineWidth',2); hold on;
yline(V_floor*3.6,'r:',sprintf('Soft floor %d km/h', round(V_floor*3.6)), ...
      'LabelHorizontalAlignment','left','LineWidth',1.5);
yline(330,'r--','330 km/h cap','LabelHorizontalAlignment','left');
xline(s_apex,'k:','Apex');
title('Speed Profile'); xlabel('s (m)'); ylabel('km/h'); grid on;
ylim([150 340]);
% Annotate min speed
[min_V, min_idx] = min(V_hist);
text(s_hist(min_idx), min_V*3.6 - 8, sprintf('%.0f km/h', min_V*3.6), ...
     'Color','b','FontSize',9,'HorizontalAlignment','center');

% 2. Wing AoA
nexttile;
plot(t_ctrl, U_hist(3,:),'r','LineWidth',2); hold on;
plot(t_ctrl, U_hist(4,:),'g','LineWidth',2);
yline(aero.fw_alpha_lim(2),'r:','Max front');
yline(aero.rw_alpha_lim(2),'g:','Max rear');
yline(0,'k--');
title('Active Wing AoA'); xlabel('t (s)'); ylabel('deg');
legend('Front wing','Rear wing','Location','best'); grid on;

% 3. Steering angle
nexttile;
plot(t_ctrl, rad2deg(U_hist(2,:)),'k','LineWidth',2); hold on;
yline( rad2deg(0.30),'r--','Max'); yline(-rad2deg(0.30),'r--');
title('Steering Angle'); xlabel('t (s)'); ylabel('deg'); grid on;
ylim([-20 20]);

% 4. Lateral deviation
nexttile;
plot(t_hist, X_hist(2,:),'m','LineWidth',2); hold on;
yline( 6.5,'r--','Track edge'); yline(-6.5,'r--');
title('Lateral Deviation'); xlabel('t (s)'); ylabel('m'); grid on;

% 5. Torque
nexttile;
plot(t_ctrl, U_hist(1,:),'Color',[0.85 0.33 0.1],'LineWidth',2); hold on;
yline(veh.T_max_drive,'r--',sprintf('T_{max}=%d Nm', veh.T_max_drive));
yline(veh.T_min_brake,'b--',sprintf('T_{brake}=%d Nm', veh.T_min_brake));
yline(0,'k--'); grid on;
title('Torque Command'); xlabel('t (s)'); ylabel('Nm');

% 6. Trajectory coloured by speed
nexttile;
plot(X_ref-6.5*sin(psi_r), Y_ref+6.5*cos(psi_r),'k-','LineWidth',1.5); hold on;
plot(X_ref+6.5*sin(psi_r), Y_ref-6.5*cos(psi_r),'k-','LineWidth',1.5);
plot(X_ref, Y_ref,'k--','LineWidth',1);
scatter(X_car, Y_car, 12, V_hist*3.6, 'filled');
cb = colorbar; cb.Label.String = 'km/h';
clim([min(V_hist)*3.6-10, 320]);
axis equal; grid on;
title('Vehicle Trajectory (colour = speed)'); xlabel('X (m)'); ylabel('Y (m)');

sgtitle('F1 NMPC v7  |  130R Suzuka  |  Physics-Honest Speed', ...
        'FontSize',13,'FontWeight','bold');

% ── Summary ───────────────────────────────────────────────────────────────
fprintf('\n=== Simulation Summary ===\n');
fprintf('  Entry speed  : %.1f km/h\n', V_hist(1)*3.6);
fprintf('  Minimum speed: %.1f km/h  (physics limit, not constraint)\n', min(V_hist)*3.6);
fprintf('  Exit speed   : %.1f km/h\n', V_hist(end)*3.6);
fprintf('  Max |ey|     : %.2f m\n',   max(abs(X_hist(2,:))));
fprintf('  Max |delta|  : %.2f deg\n', max(abs(rad2deg(U_hist(2,:)))));
fprintf('  Min torque   : %.0f Nm\n',  min(U_hist(1,:)));
fprintf('  Max wing AoA : %.1f deg (front)  %.1f deg (rear)\n', ...
        max(U_hist(3,:)), max(U_hist(4,:)));

%% =========================================================================
%  LOCAL FUNCTIONS
% =========================================================================
function Xdot = calc_dynamics(X, U, kappa, veh, tire, aero)
    import casadi.*
    ey=X(2); epsi=X(3); Vx=X(4); Vy=X(5); r=X(6);
    w_fl=X(7); w_fr=X(8); w_rl=X(9); w_rr=X(10);
    T_cmd=U(1); delta=U(2); a_f=U(3); a_r=U(4);

    Vx_s = fmax(Vx, 1.0);
    s_dot    = (Vx*cos(epsi) - Vy*sin(epsi)) / (1 - kappa*ey);
    ey_dot   =  Vx*sin(epsi) + Vy*cos(epsi);
    epsi_dot =  r - kappa*s_dot;

    p = aero.p_fw_CL; CL2_f = p(1)*a_f^2 + p(2)*a_f + p(3);
    p = aero.p_fw_CD; CD2_f = p(1)*a_f^2 + p(2)*a_f + p(3);
    p = aero.p_rw_CL; CL2_r = p(1)*a_r^2 + p(2)*a_r + p(3);
    p = aero.p_rw_CD; CD2_r = p(1)*a_r^2 + p(2)*a_r + p(3);

    CL_f = CL2_f*veh.AR_f / (veh.AR_f + 2);
    CL_r = CL2_r*veh.AR_r / (veh.AR_r + 2);
    CD_f = CD2_f + CL_f^2 / (pi*veh.AR_f*veh.e_f);
    CD_r = CD2_r + CL_r^2 / (pi*veh.AR_r*veh.e_r);

    q      = 0.5*veh.rho*Vx_s^2;
    F_df   = q*veh.A_wing*CL_f + veh.CL_body_f_frac    *q*veh.A_body*veh.CL_body;
    F_dr   = q*veh.A_wing*CL_r + (1-veh.CL_body_f_frac)*q*veh.A_body*veh.CL_body;
    F_drag = q*(veh.A_wing*(CD_f+CD_r) + veh.A_body*veh.CD_body) + veh.Crr*veh.m*9.81;

    Fz_sf = veh.m*9.81*veh.lr / veh.L;
    Fz_sr = veh.m*9.81*veh.lf / veh.L;
    ax_e  = (T_cmd/veh.R_wheel - F_drag) / veh.m;
    ay_e  = Vx_s * r;
    dWx   = (veh.h_cg/veh.L)  * veh.m * ax_e;
    dWyf  = (veh.h_cg/veh.tf) * veh.m * ay_e *  veh.roll_stiffness_f;
    dWyr  = (veh.h_cg/veh.tr) * veh.m * ay_e * (1 - veh.roll_stiffness_f);

    Fz_fl = fmax(Fz_sf - dWx/2 + dWyf + F_df/2, 10);
    Fz_fr = fmax(Fz_sf - dWx/2 - dWyf + F_df/2, 10);
    Fz_rl = fmax(Fz_sr + dWx/2 + dWyr + F_dr/2, 10);
    Fz_rr = fmax(Fz_sr + dWx/2 - dWyr + F_dr/2, 10);

    al_fl = atan((Vy + veh.lf*r)/(Vx_s - veh.tf/2*r)) - delta;
    al_fr = atan((Vy + veh.lf*r)/(Vx_s + veh.tf/2*r)) - delta;
    al_rl = atan((Vy - veh.lr*r)/(Vx_s - veh.tr/2*r));
    al_rr = atan((Vy - veh.lr*r)/(Vx_s + veh.tr/2*r));

    den   = Vx_s + 0.1;
    SR_fl = (w_fl*veh.R_wheel - Vx_s) / den;
    SR_fr = (w_fr*veh.R_wheel - Vx_s) / den;
    SR_rl = (w_rl*veh.R_wheel - Vx_s) / den;
    SR_rr = (w_rr*veh.R_wheel - Vx_s) / den;

    [Fx_fl,Fy_fl] = calc_pacejka(Fz_fl, al_fl, SR_fl, tire);
    [Fx_fr,Fy_fr] = calc_pacejka(Fz_fr, al_fr, SR_fr, tire);
    [Fx_rl,Fy_rl] = calc_pacejka(Fz_rl, al_rl, SR_rl, tire);
    [Fx_rr,Fy_rr] = calc_pacejka(Fz_rr, al_rr, SR_rr, tire);

    cd = cos(delta); sd = sin(delta);
    Fx_flb = Fx_fl*cd - Fy_fl*sd;  Fy_flb = Fx_fl*sd + Fy_fl*cd;
    Fx_frb = Fx_fr*cd - Fy_fr*sd;  Fy_frb = Fx_fr*sd + Fy_fr*cd;

    Fx_tot = Fx_flb + Fx_frb + Fx_rl + Fx_rr;
    Fy_tot = Fy_flb + Fy_frb + Fy_rl + Fy_rr;

    Mz = veh.lf*(Fy_flb + Fy_frb) - veh.lr*(Fy_rl + Fy_rr) ...
       + (veh.tf/2)*(Fx_frb - Fx_flb) + (veh.tr/2)*(Fx_rr - Fx_rl);

    Vx_dot = (Fx_tot - F_drag)/veh.m + Vy*r;
    Vy_dot =  Fy_tot/veh.m           - Vx*r;
    r_dot  =  Mz / veh.Iz;

    T_drv = fmax(T_cmd, 0);
    T_brk = fmin(T_cmd, 0);
    w_fl_dot = (T_brk*veh.brake_bias/2              - Fx_fl*veh.R_wheel) / veh.I_wheel;
    w_fr_dot = (T_brk*veh.brake_bias/2              - Fx_fr*veh.R_wheel) / veh.I_wheel;
    w_rl_dot = (T_drv/2 + T_brk*(1-veh.brake_bias)/2 - Fx_rl*veh.R_wheel) / veh.I_wheel;
    w_rr_dot = (T_drv/2 + T_brk*(1-veh.brake_bias)/2 - Fx_rr*veh.R_wheel) / veh.I_wheel;

    Xdot = [s_dot; ey_dot; epsi_dot; Vx_dot; Vy_dot; r_dot;
            w_fl_dot; w_fr_dot; w_rl_dot; w_rr_dot];
end

function [Fx, Fy] = calc_pacejka(Fz, alpha, SR, tire)
    mu    = tire.mu_max - tire.mu_load_sens*Fz;
    D     = mu * Fz;
    sigma = sqrt(SR^2 + tan(alpha)^2 + 1e-4);
    F     = D * sin(tire.C * atan(tire.B*sigma - tire.E*(tire.B*sigma - atan(tire.B*sigma))));
    Fx    =  F * SR         / sigma;
    Fy    = -F * tan(alpha) / sigma;
end