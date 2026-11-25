function out = sl_emulate(sc)
%SL_EMULATE  Run the Simulink block wrappers in the model's block order, without Simulink.
%  Used to check that the block decomposition reproduces run_sim exactly
%  (and in Octave, where Simulink is not available).
global P1SIM
sl_init(sc); S = P1SIM;
dt = S.dt_c / S.nsub; N = round(S.t_end / S.dt_c);
s = S.s0; Fw_prev = zeros(6,1); Mapp_prev = [0;0]; axFxt_prev = [0;0];
E_bat = 0; E_brk = 0; started = false; t_log = []; phi_log = []; v_log = [];
for k = 1:N
  t = (k-1)*S.dt_c;
  delta = sl_steer(t);
  Fz = sl_loads(axFxt_prev(1), axFxt_prev(2), 0);
  aux = sl_truth(s, Fw_prev, delta, Fz);
  [y, v, ay, ax_m] = sl_sensors(aux);
  [D, Fdrive] = sl_scenario(t, v);
  xh = sl_kf(Mapp_prev, delta, y, v);
  Mdem = sl_lqr(xh, v, delta);
  Fz_est = sl_loads(ax_m, axFxt_prev(2), 1);
  [Fw, Fm] = sl_alloc(D, Fdrive, Mdem, v, Fz_est, ay);
  [Pbat, Pbrk, Mapp] = sl_energy(Fw, Fm, Fz, aux);
  if t >= S.t_b, E_bat = E_bat + Pbat*S.dt_c; E_brk = E_brk + Pbrk*S.dt_c; end
  t_log(end+1) = t; phi_log(end+1) = s(4); v_log(end+1) = aux(1); %#ok<AGROW>
  for j = 1:S.nsub
    k1 = sl_plant(s, Fw, delta, Fz); k2 = sl_plant(s + dt/2*k1, Fw, delta, Fz);
    k3 = sl_plant(s + dt/2*k2, Fw, delta, Fz); k4 = sl_plant(s + dt*k3, Fw, delta, Fz);
    s = s + dt/6*(k1 + 2*k2 + 2*k3 + k4);
  end
  Fw_prev = Fw; Mapp_prev = Mapp; axFxt_prev = [aux(6); aux(9)];
  if t > S.t_b && aux(1) < 3, break; end
  if abs(s(4)) > deg2rad(60), break; end
end
out.E_bat = E_bat; out.E_brk = E_brk; out.regen_share = E_bat / max(E_brk, 1);
out.t = t_log; out.phi = phi_log; out.v = v_log;
end
