%CHECK_MODEL  Course checks on the design model (Lectures 13-23):
%  open-loop and closed-loop eigenvalues vs speed and payload, controllability and
%  observability ranks for three sensor sets, and a linear vs nonlinear comparison.
%  Writes results/poles.csv and results/model_checks.txt.
here = fileparts(mfilename('fullpath')); addpath(here);
outdir = fullfile(here, '..', 'results'); if ~exist(outdir, 'dir'), mkdir(outdir); end
fid = fopen(fullfile(outdir, 'poles.csv'), 'w');
fprintf(fid, 'payload,u,loop,re,im\n');
ft = fopen(fullfile(outdir, 'model_checks.txt'), 'w');
pays = {'light', 'rated', 'over'};
for p = 1:3
  P = params_vehicle(pays{p}); G = lqr_schedule(P);
  fprintf(ft, '%s: m2 = %.0f kg, static axle loads [front drive trailer] = [%.1f %.1f %.1f] kN\n', ...
    pays{p}, P.m2, P.Fz0/1e3);
  for u = 5:1:30
    [A, B] = lin_model(u, P); Bu = B(:,1:2);
    K = gain_at(G, u);
    e1 = eig(A); e2 = eig(A - Bu*K);
    for k = 1:4, fprintf(fid, '%s,%d,open,%.5f,%.5f\n', pays{p}, u, real(e1(k)), imag(e1(k))); end
    for k = 1:4, fprintf(fid, '%s,%d,closed,%.5f,%.5f\n', pays{p}, u, real(e2(k)), imag(e2(k))); end
    if any(u == [10 20 30])
      Co = [Bu A*Bu A^2*Bu A^3*Bu];
      Cr = [0 1 0 0]; Crp = [0 1 0 0; 0 0 0 1]; Cay = [0 1 0 0; A(1,:) + [0 u 0 0]];
      ob = @(C) rank([C; C*A; C*A^2; C*A^3]);
      fprintf(ft, '  u=%2d m/s  rank ctrb[M1 M2]=%d  rank ctrb[M2 only]=%d  rank obsv: r=%d, r+phi=%d, r+ay=%d\n', ...
        u, rank(Co), rank([Bu(:,2) A*Bu(:,2) A^2*Bu(:,2) A^3*Bu(:,2)]), ob(Cr), ob(Crp), ob(Cay));
      fprintf(ft, '     open-loop eig: %s\n     closed-loop eig: %s\n', num2str(e1.', '%8.3f'), num2str(e2.', '%8.3f'));
    end
  end
  fprintf(ft, '  lateral-drift eigenvalue -Cn*g/u at u=20: %.3f (unobservable from r and phi, stable => detectable)\n', -P.Cn*P.g/20);
end
fclose(fid);
% linear vs nonlinear at small steer
P = params_vehicle('light'); u = 15; d = 0.005; [A, B] = lin_model(u, P);
xl = zeros(4,1); s = [0;0;0;0;u;0;0;0]; dt = 0.002; Fz = P.Fz0; z6 = zeros(1,6);
for k = 1:1500
  xl = xl + dt*(A*xl + B(:,3)*d);
  k1 = plant_deriv(s, z6, d, 0.9, Fz, P); k2 = plant_deriv(s + dt/2*k1, z6, d, 0.9, Fz, P);
  k3 = plant_deriv(s + dt/2*k2, z6, d, 0.9, Fz, P); k4 = plant_deriv(s + dt*k3, z6, d, 0.9, Fz, P);
  s = s + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
[~, a] = plant_deriv(s, z6, d, 0.9, Fz, P);
fprintf(ft, 'Linear vs nonlinear, 15 m/s, 0.005 rad steer, t = 3 s: r1 %.5f vs %.5f rad/s, phi %.5f vs %.5f rad\n', xl(2), a.r1, xl(4), s(4));
P = params_vehicle('light'); [A, B] = lin_model(5, P); xs = -A \ (B(:,3)*0.05);
fprintf(ft, 'Kinematic check, 5 m/s, delta 0.05 rad: r1 = %.4f rad/s vs u*delta/L1 = %.4f\n', xs(2), 5*0.05/P.L1);
fclose(ft);
type(fullfile(outdir, 'model_checks.txt'));
