function xh = sl_kf(Mapp_prev, delta, y, v_meas)
%SL_KF  Block "Kalman filter" (Lecture 21), augmented with yaw-moment disturbance states.
%  Mapp_prev: yaw moments applied at the wheels over the last period [M1; M2] (N m)
global P1SIM
persistent x P
S = P1SIM;
if isempty(x), x = S.xh0; P = S.Pk0; end
ny = size(S.KF.C, 1);
[x, P] = kf_step(x, P, [Mapp_prev(1); Mapp_prev(2); delta], y(1:ny), v_meas, S.dt_c, S.Pc, S.KF);
xh = zeros(6, 1); xh(1:numel(x)) = x;
end
