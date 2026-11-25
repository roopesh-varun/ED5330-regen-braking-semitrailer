function S = scenario_setup(sc)
%SCENARIO_SETUP  Everything a run needs before the first time step.
%  Shared by run_sim (script) and sl_init (Simulink), so both use identical settings.
%  sc fields: type ('bit' | 'lc'), mode, mu, payload, payload_ctrl, sensors ('r_phi' | 'r'),
%             seed, Pbat (optional battery limit override, W)
S.sc = sc;
S.P  = params_vehicle(sc.payload);          % true vehicle
S.Pc = params_vehicle(sc.payload_ctrl);     % vehicle assumed by the controller
if isfield(sc, 'Pbat') && ~isempty(sc.Pbat), S.P.Pbat_max = sc.Pbat; S.Pc.Pbat_max = sc.Pbat; end
S.mu = sc.mu; g = S.P.g;
S.dt_c = 0.01; S.nsub = 2;
if exist('OCTAVE_VERSION', 'builtin')
  rand('seed', sc.seed); randn('seed', sc.seed);   % Octave generator
else
  rng(sc.seed, 'twister');                        % MATLAB generator (noise differs from Octave)
end

switch sc.type
  case 'bit'   % brake in turn at constant steering (test of Erdinc et al. 2023)
    S.Rturn = 72; ay0 = min(0.5*S.mu*g, 2.5);
    S.v0 = sqrt(ay0 * S.Rturn); S.delta0 = S.P.L1 / S.Rturn; S.da = 0; S.T = 0;
    S.adem = 0.35*S.mu*g; S.t_b = 2.0; S.t_end = 16;
  case 'lc'    % single lane change with braking
    S.v0 = 70/3.6; S.T = 3; S.t_b = 1.0; S.t_end = 8; S.delta0 = 0;
    [A, B] = lin_model(S.v0, S.P); xs = -A \ B(:,3);
    S.da = 0.3*S.mu*g / (S.v0 * xs(2));
    S.adem = 0.25*S.mu*g;
  otherwise
    error('unknown scenario type');
end
S.mtot = S.P.m1 + S.P.m2;

% controller and estimator
S.G = lqr_schedule(S.Pc);
sig = [0.003 0.005];                        % yaw-rate gyro (rad/s), articulation sensor (rad)
if strcmp(sc.sensors, 'r')
  S.KF.C = [0 1 0 0]; S.KF.R = sig(1)^2; S.KF.nd = 0;     % augmented model not detectable
  S.KF.Qd = diag([0.05 0.01 0.03 0.001].^2);
else
  S.KF.C = [0 1 0 0; 0 0 0 1]; S.KF.R = diag(sig.^2); S.KF.nd = 2;
  S.KF.Qd = diag([0.02 0.005 0.005 0.0005 2e3 2e3].^2);   % last two: yaw-moment disturbances (N m)
end

% initial state: linear steady state at v0
[A, B] = lin_model(S.v0, S.P); x0 = -A \ (B(:,3) * delta_at(S, 0));
S.s0  = [0; 0; 0; x0(4); S.v0; x0(1); x0(2); 0];
S.xh0 = [x0; zeros(S.KF.nd, 1)];
S.Pk0 = diag([0.1 0.01 0.01 0.01 5e3*ones(1, S.KF.nd)].^2);
end
