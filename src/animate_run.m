function animate_run(mu, payload, modes)
%ANIMATE_RUN  Live demo: top view of the tractor and semitrailer for several controllers.
%  animate_run                 % mu 0.3, light payload, max-regen vs proposed
%  animate_run(0.3, 'light', {'fric_esc','emax','emax_lqr','prop'})
if nargin < 1, mu = 0.3; end
if nargin < 2, payload = 'light'; end
if nargin < 3, modes = {'emax', 'prop'}; end
names = struct('fric_esc', 'B1 Friction-only ESC', 'emax', 'B2 Max-regen', 'emax_lqr', 'B3 Max-regen + LQR', 'prop', 'Proposed');
cols  = struct('fric_esc', [0.54 0.54 0.53], 'emax', [0.92 0.41 0.20], 'emax_lqr', [0.11 0.69 0.48], 'prop', [0.16 0.47 0.84]);
P = params_vehicle(payload); n = numel(modes); Ls = cell(1, n); Rs = cell(1, n);
for i = 1:n
  sc = struct('type', 'bit', 'mode', modes{i}, 'mu', mu, 'payload', payload, 'payload_ctrl', payload, ...
              'sensors', 'r_phi', 'seed', 1, 'Pbat', []);
  [Rs{i}, Ls{i}] = run_sim(sc);
end
fig = figure('Color', 'w', 'Name', 'Brake in turn');
hp = cell(1, n);
allX = []; allY = []; for i = 1:n, allX = [allX; Ls{i}.X]; allY = [allY; Ls{i}.Y]; end %#ok<AGROW>
for i = 1:n
  subplot(1, n, i); hold on; axis equal; grid on;
  xlim([min(allX) - 15, max(allX) + 15]); ylim([min(allY) - 15, max(allY) + 15]);
  plot(Ls{i}.X, Ls{i}.Y, ':', 'Color', [0.8 0.8 0.8]);
  title(sprintf('%s  (regen %.0f%%)', names.(modes{i}), 100*Rs{i}.regen_share), 'Color', cols.(modes{i}));
  hp{i}.st = patch(nan, nan, cols.(modes{i}), 'FaceAlpha', 0.25, 'EdgeColor', cols.(modes{i}));
  hp{i}.tr = patch(nan, nan, cols.(modes{i}), 'FaceAlpha', 0.6, 'EdgeColor', cols.(modes{i}));
  hp{i}.txt = text(0.03, 0.95, '', 'Units', 'normalized', 'VerticalAlignment', 'top', 'FontSize', 9);
end
kmax = max(cellfun(@(l) numel(l.t), Ls));
for k = 1:5:kmax
  if ~ishandle(fig), return; end
  for i = 1:n
    L = Ls{i}; kk = min(k, numel(L.t));
    [tr, st] = outline(P, L.X(kk), L.Y(kk), L.psi1(kk), L.phi(kk));
    set(hp{i}.tr, 'XData', tr(:,1), 'YData', tr(:,2)); set(hp{i}.st, 'XData', st(:,1), 'YData', st(:,2));
    set(hp{i}.txt, 'String', sprintf('t = %.1f s\nv = %.0f km/h\narticulation %.0f deg\nbattery %.0f kW', ...
      L.t(kk), 3.6*L.v(kk), rad2deg(L.phi(kk)), L.Pbat(kk)/1e3));
  end
  drawnow; pause(0.03);
end
end

function [tr, st] = outline(P, X, Y, p1, ph)
W = 2.5; p2 = p1 - ph;
e1 = [cos(p1) sin(p1)]; n1 = [-sin(p1) cos(p1)]; e2 = [cos(p2) sin(p2)]; n2 = [-sin(p2) cos(p2)];
c = [X Y];
tr = [c + (P.a1+1.4)*e1 + W/2*n1; c + (P.a1+1.4)*e1 - W/2*n1; c - (P.b1+0.6)*e1 - W/2*n1; c - (P.b1+0.6)*e1 + W/2*n1];
H = c - P.c1*e1;
st = [H + 1.2*e2 + W/2*n2; H + 1.2*e2 - W/2*n2; H - (P.l2c+1.5)*e2 - W/2*n2; H - (P.l2c+1.5)*e2 + W/2*n2];
end
