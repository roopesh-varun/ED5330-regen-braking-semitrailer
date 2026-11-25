function [sd, aux] = plant_deriv(s, Fw, delta, mu, Fz, P)
%PLANT_DERIV  Nonlinear tractor-semitrailer plant: state derivative.
%  s     = [X; Y; psi1; phi; Xd; Yd; psi1d; phid]
%  Fw    = [fL fR dL dR tL tR] longitudinal wheel forces (N, wheel frame, negative = braking)
%          (front steer, tractor drive, semitrailer axle; each axle lumped, left/right wheels)
%  delta = front steer angle (rad),  mu = road friction,  Fz = [front; drive; trailer] axle loads (N)
%  Wheel forces are clipped to the friction limit of each wheel (slip-controlled brakes).
q = s(1:4); qd = s(5:8);
psi1 = q(3); phi = q(4); psi2 = psi1 - phi;
r1 = qd(3); r2 = qd(3) - qd(4);
% body-frame velocities
c = cos(psi1); sn = sin(psi1);
vx1 =  c*qd(1) + sn*qd(2);
vy1 = -sn*qd(1) + c*qd(2);
% semitrailer CoG velocity (world) from the kinematics p2 = p1 - c1 e1 - d2 e2
c2 = cos(psi2); s2 = sin(psi2);
p2d = [qd(1) + P.c1*sn*r1 + P.d2*s2*r2; qd(2) - P.c1*c*r1 - P.d2*c2*r2];
vx2 =  c2*p2d(1) + s2*p2d(2);
vy2 = -s2*p2d(1) + c2*p2d(2);

Fzw = [Fz(1) Fz(1) Fz(2) Fz(2) Fz(3) Fz(3)] / 2;
Fw  = max(min(Fw(:)', mu*Fzw), -mu*Fzw);

% slip angles (rad)
vfb = [vx1; vy1 + P.a1*r1];
vxw =  vfb(1)*cos(delta) + vfb(2)*sin(delta);
vyw = -vfb(1)*sin(delta) + vfb(2)*cos(delta);
alpha_f = atan2(vyw, max(vxw, 0.5));
alpha_r = atan2(vy1 - P.b1*r1, max(vx1, 0.5));
alpha_t = atan2(vy2 - P.e2*r2, max(vx2, 0.5));

Fy = tyre_lat([alpha_f alpha_f alpha_r alpha_r alpha_t alpha_t], Fzw, Fw, mu, P.Cn);

% axle forces in body frames
Fxfw = Fw(1) + Fw(2); Fyfw = Fy(1) + Fy(2);
Fxf = Fxfw*cos(delta) - Fyfw*sin(delta);
Fyf = Fxfw*sin(delta) + Fyfw*cos(delta);
Fxr = Fw(3) + Fw(4); Fyr = Fy(3) + Fy(4);
Fxt = Fw(5) + Fw(6); Fyt = Fy(5) + Fy(6);
h = P.w/2;
Mz1 = h*(Fw(2) - Fw(1)) + h*(Fw(4) - Fw(3));   % right minus left
Mz2 = h*(Fw(6) - Fw(5));

[M, rhs] = eom_nonlinear(q, qd, [Fxf Fyf Fxr Fyr Fxt Fyt Mz1 Mz2], P);
qdd = M \ rhs;
sd = [qd; qdd];

if nargout > 1
  % accelerations in body frames
  ax1w = qdd(1); ay1w = qdd(2);
  a1b = [c*ax1w + sn*ay1w; -sn*ax1w + c*ay1w];
  r2d = qdd(3) - qdd(4);
  p2dd = [qdd(1) + P.c1*(c*r1^2 + sn*qdd(3)) + P.d2*(c2*r2^2 + s2*r2d); ...
          qdd(2) + P.c1*(sn*r1^2 - c*qdd(3)) + P.d2*(s2*r2^2 - c2*r2d)];
  aux.ax1 = a1b(1); aux.ay1 = a1b(2);
  aux.ay2 = -s2*p2dd(1) + c2*p2dd(2);
  aux.vx1 = vx1; aux.vy1 = vy1; aux.r1 = r1; aux.vx2 = vx2; aux.vy2 = vy2; aux.r2 = r2;
  aux.alpha = [alpha_f alpha_r alpha_t];
  aux.Fw = Fw; aux.Fy = Fy; aux.Fxt = Fxt; aux.Mz = [Mz1 Mz2];
end
end
