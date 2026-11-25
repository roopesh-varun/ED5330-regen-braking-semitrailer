%RUN_ALL  Full run matrix for ED5330 project P1 (about 80 runs, a few minutes).
%  Writes results/metrics.csv and results/logs_*.mat (time histories of selected runs).
here = fileparts(mfilename('fullpath')); addpath(here);
outdir = fullfile(here, '..', 'results'); if ~exist(outdir, 'dir'), mkdir(outdir); end
modes = {'fric_esc', 'emax', 'emax_lqr', 'prop'};
pays  = {'light', 'rated', 'over'};
runs = {};
mk = @(type, mode, mu, pay, payc, sens, seed, pbat, tag) struct('type', type, 'mode', mode, 'mu', mu, ...
  'payload', pay, 'payload_ctrl', payc, 'sensors', sens, 'seed', seed, 'Pbat', pbat, 'tag', tag);
% A: brake in turn, 3 payloads x 3 friction x 4 controllers
for p = 1:3, for mu = [0.3 0.6 0.9], for m = 1:4
  runs{end+1} = mk('bit', modes{m}, mu, pays{p}, pays{p}, 'r_phi', 1, [], 'A'); end, end, end
% B: lane change with braking, 3 payloads x 2 friction x 4 controllers
for p = 1:3, for mu = [0.5 0.9], for m = 1:4
  runs{end+1} = mk('lc', modes{m}, mu, pays{p}, pays{p}, 'r_phi', 1, [], 'B'); end, end, end
% C1: payload error (controller assumes the wrong semitrailer mass), proposed, brake in turn
for mu = [0.3 0.6 0.9]
  runs{end+1} = mk('bit', 'prop', mu, 'over', 'light', 'r_phi', 1, [], 'C1');
  runs{end+1} = mk('bit', 'prop', mu, 'light', 'over', 'r_phi', 1, [], 'C1');
end
% C2: yaw-rate sensor only (articulation not measured), proposed
for mu = [0.3 0.6 0.9]
  runs{end+1} = mk('bit', 'prop', mu, 'light', 'light', 'r', 1, [], 'C2'); end
% C3: high state of charge, battery limit 100 kW, proposed and energy-max
for mu = [0.3 0.6 0.9], for m = [2 4]
  runs{end+1} = mk('bit', modes{m}, mu, 'light', 'light', 'r_phi', 1, 100e3, 'C3'); end, end
% C4: sensor-noise seeds, proposed at mu = 0.3
for seed = 2:3
  runs{end+1} = mk('bit', 'prop', 0.3, 'light', 'light', 'r_phi', seed, [], 'C4'); end

keep = {}; % logs kept for figures and the animation
fid = fopen(fullfile(outdir, 'metrics.csv'), 'w');
fprintf(fid, 'tag,type,mode,mu,payload,payload_ctrl,sensors,seed,Pbat_kW,v0,stop_reason,E_bat_MJ,E_brk_MJ,regen_share,dalpha_r_deg,dalpha_t_deg,unstable,phi_err_max_deg,r_err_rms,mean_decel,RWA,Mpeak_kNm\n');
t0 = tic;
for i = 1:numel(runs)
  sc = runs{i};
  [R, L] = run_sim(sc);
  pb = sc.Pbat; if isempty(pb), pb = 500e3; end
  fprintf(fid, '%s,%s,%s,%.2f,%s,%s,%s,%d,%.0f,%.3f,%s,%.4f,%.4f,%.4f,%.3f,%.3f,%d,%.3f,%.5f,%.3f,%.3f,%.2f\n', ...
    sc.tag, sc.type, sc.mode, sc.mu, sc.payload, sc.payload_ctrl, sc.sensors, sc.seed, pb/1e3, R.v0, R.stop_reason, ...
    R.E_bat/1e6, R.E_brk/1e6, R.regen_share, R.dalpha_r, R.dalpha_t, R.unstable, R.phi_err_max, R.r_err_rms, ...
    R.mean_decel, R.RWA, R.Mpeak/1e3);
  if any(strcmp(sc.tag, {'A', 'B'})) && strcmp(sc.payload, 'light')
    nm = sprintf('%s_%s_mu%02d_%s', sc.type, sc.mode, round(100*sc.mu), sc.payload);
    keep{end+1} = nm; S.(nm) = L; %#ok<SAGROW>
  end
  fprintf('%3d/%d %-4s %-3s %-9s mu=%.1f %-5s regen=%5.1f%% unstable=%d\n', i, numel(runs), sc.tag, sc.type, sc.mode, sc.mu, sc.payload, 100*R.regen_share, R.unstable);
end
fclose(fid);
save(fullfile(outdir, 'logs_light.mat'), '-struct', 'S', '-v7');
fprintf('done in %.0f s\n', toc(t0));
