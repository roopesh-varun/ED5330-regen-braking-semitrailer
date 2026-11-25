function [Fw, Fm, info] = allocator(mode, D, Fdrive, Mdem, v, Fz, mu, ay, P)
%ALLOCATOR  Split the driver's brake demand and the yaw-moment demands over the wheels.
%  mode   : 'fric_esc' (B1) friction-only braking in proportion to axle load + LQR yaw moments
%           'emax'     (B2) regenerative braking maximised, tractor axle first, no yaw control
%           'emax_lqr' (B3) as B2 plus LQR yaw moments (ablation)
%           'prop'          proposed: stability-capped regen, semitrailer first, + LQR yaw moments
%  D      : total brake demand (N, >= 0)       Fdrive : propulsion force (N, >= 0)
%  Mdem   : [M1 M2] yaw-moment demands (N m)    v : speed (m/s)
%  Fz     : estimated [front; drive; trailer] axle loads (N)
%  mu     : friction estimate,  ay : lateral acceleration estimate (m/s^2)
%  Fw     : [fL fR dL dR tL tR] longitudinal wheel forces (N, negative = braking)
%  Fm     : motor part of each wheel force (N, braking positive, traction negative)
w = P.w; g = P.g; vv = max(v, 1);
Fd_lim = min(P.Fd_max, P.Pd_max / vv);          % tractor e-axle (both wheels)
Ft_lim = min(P.Ft_max, P.Pt_max / vv);          % each semitrailer motor
Fbat   = P.Pbat_max / vv;                        % battery limit as a force
cap_full = mu * Fz;                              % full friction capacity per axle
Ba = zeros(3,1); Bm_d = 0; Bm_t = 0;

switch mode
  case 'fric_esc'
    Ba = D * Fz / sum(Fz);
  case {'emax', 'emax_lqr'}
    Bm_d = min([D, Fd_lim, cap_full(2), Fbat]);
    Bm_t = min([D - Bm_d, 2*Ft_lim, cap_full(3), Fbat - Bm_d]);
    Ba = [0; Bm_d; Bm_t];
    Ba = fill_friction(Ba, D - Bm_d - Bm_t, Fz, cap_full);
  case 'prop'
    Fyreq = Fz * abs(ay) / g;                    % lateral force needed per axle (quasi-static)
    s = [P.s_front; P.s_drive; P.s_trail];
    cap = sqrt(max((s .* mu .* Fz).^2 - Fyreq.^2, 0));
    Bm_t = min([D, 2*Ft_lim, cap(3), Fbat]);
    Bm_d = min([D - Bm_t, Fd_lim, cap(2), Fbat - Bm_t]);
    Ba = [0; Bm_d; Bm_t];
    rest = D - Bm_d - Bm_t;
    % friction in stability order: steer axle, semitrailer, drive axle, up to the caps
    for a = [1 3 2]
      add = min(rest, max(cap(a) - Ba(a), 0));
      Ba(a) = Ba(a) + add; rest = rest - add;
    end
    if rest > 0                                   % emergency: driver demand has priority
      Ba = fill_friction(Ba, rest, Fz, cap_full);
    end
  otherwise
    error('allocator: unknown mode %s', mode);
end

% yaw moments: B1, B3 and proposed use the LQR demand, B2 none
if strcmp(mode, 'emax'), M1 = 0; M2 = 0; else, M1 = Mdem(1); M2 = Mdem(2); end
kf = Fz(1) / (Fz(1) + Fz(2));                    % tractor moment split by axle load
Mf = kf*M1; Md = (1-kf)*M1;
% M = (w/2)(F_R - F_L) with F = -B  =>  B_L - B_R = 2M/w
B = [Ba(1)/2 + Mf/w, Ba(1)/2 - Mf/w, Ba(2)/2 + Md/w, Ba(2)/2 - Md/w, Ba(3)/2 + M2/w, Ba(3)/2 - M2/w];
Fzw = [Fz(1) Fz(1) Fz(2) Fz(2) Fz(3) Fz(3)] / 2;
B = min(B, mu * Fzw);

Fm = zeros(1,6);
if ~strcmp(mode, 'fric_esc')
  % tractor e-axle: one motor, open differential -> equal motor force on both wheels
  md = max(min([Bm_d/2, B(3), B(4)]), 0);
  Fm(3:4) = md;
  % semitrailer: independent left/right motors, may also drive to create a moment
  mt = max(min(B(5:6), Ft_lim), -Ft_lim);
  if strcmp(mode, 'emax'), mt = min([Bm_t/2 Bm_t/2], [B(5) B(6)]); end
  Fm(5:6) = mt;
  % battery limit on net charging power
  preg = sum(max(Fm, 0)) * vv;
  if preg > P.Pbat_max, Fm = min(Fm, 0) + max(Fm, 0) * (P.Pbat_max / preg); end
end
% friction brakes cannot propel: wheels without motor traction stay >= 0
B(1:2) = max(B(1:2), 0); B(3:4) = max(B(3:4), Fm(3:4)); B(5:6) = max(B(5:6), min(Fm(5:6), 0));
Fw = -B;
Fw(3:4) = Fw(3:4) + Fdrive/2;
info.Ba = Ba; info.Mreal = [w/2*((B(1)-B(2)) + (B(3)-B(4))), w/2*(B(5)-B(6))];
end

function Ba = fill_friction(Ba, rest, Fz, capf)
% distribute the remaining demand in proportion to axle load, within friction limits
for pass = 1:3
  if rest <= 1e-6, break; end
  room = max(capf - Ba, 0);
  if sum(room) <= 0, break; end
  share = Fz .* (room > 0); share = share / sum(share);
  add = min(rest * share, room);
  Ba = Ba + add; rest = rest - sum(add);
end
end
