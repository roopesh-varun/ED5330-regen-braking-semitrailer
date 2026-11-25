function [Mdem, xr] = sl_lqr(xh, v_meas, delta)
%SL_LQR  Block "Reference model + gain-scheduled LQR" (Lecture 23): Mdem = -K(u)(x_hat - x_ref).
global P1SIM
S = P1SIM;
xr = ref_state(v_meas, delta, S.mu, S.Pc);
if strcmp(S.sc.mode, 'emax')
  Mdem = [0; 0];
else
  K = gain_at(S.G, v_meas);
  Mdem = -K * (xh(1:4) - xr);
  Mdem = max(min(Mdem, 60e3), -60e3);
end
end
