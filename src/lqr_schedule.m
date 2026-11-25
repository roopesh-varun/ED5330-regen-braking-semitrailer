function G = lqr_schedule(P, W)
%LQR_SCHEDULE  LQR gains for the yaw-moment inputs [M1 M2] on a speed grid.
%  Gains are designed for the payload in P (payload scheduling = calling this
%  function with the estimated payload). W holds the Bryson-rule limits.
if nargin < 2, W = lqr_weights(); end
G.u = 3:1:30;
G.K = zeros(2, 4, numel(G.u));
Q = diag(1 ./ W.xmax.^2);
R = diag(1 ./ W.umax.^2);
for k = 1:numel(G.u)
  [A, B] = lin_model(G.u(k), P);
  G.K(:,:,k) = lqr_care(A, B(:,1:2), Q, R);
end
G.Q = Q; G.R = R;
end
