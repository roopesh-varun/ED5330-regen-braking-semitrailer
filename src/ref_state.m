function xr = ref_state(u, delta, mu, P)
%REF_STATE  Desired state: steady-state response of the linear model to the steer
%  angle at the current speed, scaled down so that |u*r1| <= 0.85*mu*g.
u = max(u, 3);
[A, B] = lin_model(u, P);
xr = -A \ (B(:,3) * delta);
rmax = 0.85 * mu * P.g / u;
if abs(xr(2)) > rmax, xr = xr * (rmax / abs(xr(2))); end
end
