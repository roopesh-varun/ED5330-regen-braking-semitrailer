function [R, log] = run_sim(sc)
%RUN_SIM  Simulate one scenario of the tractor + electric semitrailer.
%  sc fields: type ('bit' brake-in-turn | 'lc' lane change with braking), mode (allocator mode),
%             mu, payload (true), payload_ctrl (assumed by the controller), sensors ('r_phi'|'r'),
%             seed, Pbat (battery limit override, W, optional)
%  R   : scalar metrics,  log : time histories at the control rate
S = scenario_setup(sc);
P = S.P; Pc = S.Pc; mu = S.mu; dt_c = S.dt_c; nsub = S.nsub; dt = dt_c / nsub;
v0 = S.v0; t_b = S.t_b; t_end = S.t_end; adem = S.adem; mtot = S.mtot;
G = S.G; KF = S.KF; delta_fn = @(t) delta_at(S, t);
s = S.s0; xh = S.xh0; Pk = S.Pk0;
ax_prev = 0; Fxt_prev = 0; ax_m = 0; Fw = zeros(1,6); Fm = zeros(1,6); Mdem = [0 0]; Mapp = [0 0];

N = round(t_end / dt_c);
L = struct('t', nan(N,1), 'X', nan(N,1), 'Y', nan(N,1), 'psi1', nan(N,1), 'phi', nan(N,1), ...
  'v', nan(N,1), 'r1', nan(N,1), 'xh', nan(N,6), 'xr', nan(N,4), 'alpha', nan(N,3), ...
  'ay1', nan(N,1), 'ay2', nan(N,1), 'ax', nan(N,1), 'Mdem', nan(N,2), 'Mreal', nan(N,2), ...
  'Fw', nan(N,6), 'Fm', nan(N,6), 'Pbat', nan(N,1), 'Pbrk', nan(N,1), 'Fz', nan(N,3), 'delta', nan(N,1), ...
  'vy1', nan(N,1), 'phid', nan(N,1));
stop_reason = 'time';
for k = 1:N
  t = (k-1)*dt_c;
  delta = delta_fn(t);
  Fz = axle_loads(P, ax_prev, Fxt_prev);
  [~, a] = plant_deriv(s, Fw, delta, mu, Fz, P);
  % ---- sensors ----
  v_meas = a.vx1;
  y = KF.C * [a.vy1; a.r1; s(8); s(4)] + sqrt(diag(KF.R)) .* randn(size(KF.C,1), 1);
  ay_meas = v_meas * y(1);
  ax_m = 0.8*ax_m + 0.2*a.ax1;
  % ---- estimator ----
  % input = yaw moments actually commanded at the wheels over the last period
  [xh, Pk] = kf_step(xh, Pk, [Mapp(1); Mapp(2); delta], y, v_meas, dt_c, Pc, KF);
  % ---- controller ----
  xr = ref_state(v_meas, delta, mu, Pc);
  if strcmp(sc.mode, 'emax')
    Mdem = [0 0];
  else
    K = gain_at(G, v_meas);
    Mdem = (-K * (xh(1:4) - xr))';
    Mdem = max(min(Mdem, 60e3), -60e3);
  end
  if t < t_b
    D = 0; Fdrive = max(mtot * 1.0 * (v0 - v_meas), 0);
  else
    D = mtot * adem; Fdrive = 0;
  end
  Fz_est = axle_loads(Pc, ax_m, Fxt_prev);
  [Fw, Fm, info] = allocator(sc.mode, D, Fdrive, Mdem, v_meas, Fz_est, mu, ay_meas, Pc);
  hw = P.w/2; Mapp = [hw*(Fw(2)-Fw(1)) + hw*(Fw(4)-Fw(3)), hw*(Fw(6)-Fw(5))];
  % ---- energy bookkeeping with the forces the plant can actually transmit ----
  Fzw = [Fz(1) Fz(1) Fz(2) Fz(2) Fz(3) Fz(3)] / 2;
  Fwc = max(min(Fw, mu*Fzw), -mu*Fzw);
  vw  = [a.vx1 a.vx1 a.vx1 a.vx1 a.vx2 a.vx2];
  Bw  = max(-Fwc, 0);                                % braking magnitude per wheel
  mreg = min(max(Fm, 0), Bw);                       % motor braking actually transmitted
  mtrac = max(-Fm, 0) .* (Fwc > 0);                 % motor traction (semitrailer moments)
  Pbat = P.eta * sum(mreg .* vw) - sum(mtrac .* vw) / P.eta;
  Pbrk = sum(Bw .* vw);
  % ---- log ----
  L.t(k) = t; L.X(k) = s(1); L.Y(k) = s(2); L.psi1(k) = s(3); L.phi(k) = s(4);
  L.v(k) = a.vx1; L.r1(k) = a.r1; L.xh(k,1:numel(xh)) = xh'; L.xr(k,:) = xr';
  L.alpha(k,:) = a.alpha; L.ay1(k) = a.ay1; L.ay2(k) = a.ay2; L.ax(k) = a.ax1;
  L.Mdem(k,:) = Mdem; L.Mreal(k,:) = a.Mz; L.Fw(k,:) = Fwc; L.Fm(k,:) = Fm;
  L.Pbat(k) = Pbat; L.Pbrk(k) = Pbrk; L.Fz(k,:) = Fz'; L.delta(k) = delta; L.vy1(k) = a.vy1; L.phid(k) = s(8);
  % ---- integrate the plant over one control period (RK4) ----
  for j = 1:nsub
    k1 = plant_deriv(s, Fw, delta, mu, Fz, P);
    k2 = plant_deriv(s + dt/2*k1, Fw, delta, mu, Fz, P);
    k3 = plant_deriv(s + dt/2*k2, Fw, delta, mu, Fz, P);
    k4 = plant_deriv(s + dt*k3, Fw, delta, mu, Fz, P);
    s = s + dt/6*(k1 + 2*k2 + 2*k3 + k4);
  end
  ax_prev = a.ax1; Fxt_prev = a.Fxt;
  if t > t_b && a.vx1 < 3, stop_reason = 'stopped'; break; end
  if abs(s(4)) > deg2rad(60), stop_reason = 'jackknife'; break; end
end
fn = fieldnames(L);
for i = 1:numel(fn), L.(fn{i}) = L.(fn{i})(1:k, :); end
log = L;

% ---- metrics ----
ib = find(L.t >= t_b, 1);
R.mode = sc.mode; R.type = sc.type; R.mu = mu; R.payload = sc.payload; R.payload_ctrl = sc.payload_ctrl;
R.sensors = sc.sensors; R.v0 = v0; R.stop_reason = stop_reason;
R.E_bat  = sum(L.Pbat(ib:end)) * dt_c;              % J into the battery
R.E_brk  = sum(L.Pbrk(ib:end)) * dt_c;              % J dissipated by braking at the wheels
R.regen_share = R.E_bat / max(R.E_brk, 1);
R.dalpha_r = rad2deg(max(abs(L.alpha(ib:end,2) - L.alpha(ib,2))));
R.dalpha_t = rad2deg(max(abs(L.alpha(ib:end,3) - L.alpha(ib,3))));
R.unstable = (R.dalpha_r > 5) || (R.dalpha_t > 3) || strcmp(stop_reason, 'jackknife');
R.phi_err_max = rad2deg(max(abs(L.phi(ib:end) - L.xr(ib:end,4))));
R.r_err_rms = sqrt(mean((L.r1(ib:end) - L.xr(ib:end,2)).^2));
R.mean_decel = (L.v(ib) - L.v(end)) / max(L.t(end) - L.t(ib), dt_c);
R.RWA = max(abs(L.ay2)) / max(max(abs(L.ay1)), 1e-6);
R.est_err_phid = sqrt(mean((L.xh(ib:end,3) - L.phid(ib:end)).^2));
R.est_err_v1 = sqrt(mean((L.xh(ib:end,1) - L.vy1(ib:end)).^2));
R.Mpeak = max(abs(L.Mdem(:)));
end
