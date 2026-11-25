function W = lqr_weights()
%LQR_WEIGHTS  Bryson's-rule limits: Q_ii = 1/xmax_i^2, R_jj = 1/umax_j^2.
%  States x = [v1 r1 phid phi]: lateral velocity error 0.3 m/s, yaw-rate error
%  0.02 rad/s, articulation-rate error 0.03 rad/s, articulation error 0.01 rad.
%  Inputs: tractor and semitrailer yaw moments up to 30 kN m each.
W.xmax = [0.3 0.02 0.03 0.01];
W.umax = [30e3 30e3];
end
