function [xh, Pk] = kf_step(xh, Pk, uin, y, u, dt, P, KF)
%KF_STEP  One predict-correct step of the discrete Kalman filter (Lecture 21),
%  augmented with two disturbance states (Lecture 20-22 observer ideas).
%  State xa = [v1 r1 phid phi dM1 dM2]: the last two are unknown yaw moments on the
%  tractor and semitrailer (random walks). They absorb effects the linear model omits,
%  mainly the yaw coupling created by unequal braking of the two units at an angle.
%  With yaw rate as the only measurement the augmented model is not detectable,
%  so KF.nd = 0 (plain 4-state filter) is used for that sensor set.
%  uin = [M1; M2; delta] applied inputs; y = measurements selected by KF.C (4-state part).
[A, B] = lin_model(max(u, 3), P);
m = 3; nd = KF.nd; na = 4 + nd;          % nd = 2 (disturbance states) or 0 (plain filter)
if nd > 0
  Aa = [A, B(:,1:2); zeros(2, 6)];
else
  Aa = A;
end
Ba = [B; zeros(nd, 3)];
E = expm([Aa Ba; zeros(m, na+m)] * dt);
Ad = E(1:na, 1:na); Bd = E(1:na, na+1:end);
% predict
xh = Ad*xh + Bd*uin;
Pk = Ad*Pk*Ad' + KF.Qd;
% correct
C = [KF.C, zeros(size(KF.C,1), nd)];
S = C*Pk*C' + KF.R;
L = Pk*C' / S;
xh = xh + L*(y - C*xh);
Pk = (eye(na) - L*C) * Pk;
end
