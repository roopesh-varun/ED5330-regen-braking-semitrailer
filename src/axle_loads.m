function [Fz, Fcz] = axle_loads(P, ax, Fxt)
%AXLE_LOADS  Quasi-static vertical axle loads with longitudinal load transfer.
%  ax  : longitudinal acceleration of the combination (m/s^2, negative when braking)
%  Fxt : total longitudinal force on the semitrailer axle (N, negative when braking)
%  Fz  : [front; drive; trailer] axle loads (N),  Fcz : vertical coupling load (N)
%  Side-view moment balances about the semitrailer axle and the tractor front axle.
g = P.g;
Fcx = P.m2*ax - Fxt;                                    % coupling force on trailer, x
Fcz = (P.e2*P.m2*g - P.h2*P.m2*ax + P.hc*Fcx) / P.l2c; % coupling load on trailer (up)
Fzt = P.m2*g - Fcz;
Fzr = (P.a1*P.m1*g + P.h1*P.m1*ax + P.l1c*Fcz + P.hc*Fcx) / P.L1;
Fzf = P.m1*g + Fcz - Fzr;
Fz  = max([Fzf; Fzr; Fzt], 0);
end
