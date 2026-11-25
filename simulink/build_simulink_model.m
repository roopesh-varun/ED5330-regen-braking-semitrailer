function build_simulink_model(sc)
%BUILD_SIMULINK_MODEL  Create P1_tractor_etrailer.slx (run in MATLAB with Simulink).
%  Each control block is a MATLAB Function block that calls a wrapper sl_*.m as an
%  extrinsic function, so the model runs exactly the code verified by sl_emulate.m.
%  Usage:  build_simulink_model                      % default: brake in turn, mu 0.3, proposed
%          sim('P1_tractor_etrailer'); sl_results    % run and summarise
%  To run another case: sl_init(struct(...)) then sim again (no rebuild needed).
if nargin < 1, sc = []; end
here = fileparts(mfilename('fullpath')); addpath(here); addpath(fullfile(here, '..', 'src'));
if isempty(sc), sl_init(); else, sl_init(sc); end
mdl = 'P1_tractor_etrailer';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(fullfile(here, [mdl '.slx']), 'file'), delete(fullfile(here, [mdl '.slx'])); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'Solver', 'ode4', 'FixedStep', 'P1_dt', 'StopTime', 'P1_tend', ...
  'SaveTime', 'off', 'SaveOutput', 'off', 'ReturnWorkspaceOutputs', 'off');
NL = char(10);

% ---------- sources, plant integrator, rate transitions ----------
add_block('simulink/Sources/Clock', [mdl '/Clock'], 'Position', [30 40 60 70]);
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/ZOH t'], 'SampleTime', 'P1_Ts', 'Position', [100 40 130 70]);
add_block('simulink/Continuous/Integrator', [mdl '/Vehicle states'], 'InitialCondition', 'P1_s0', ...
  'Position', [1180 520 1230 560]);
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/ZOH s'], 'SampleTime', 'P1_Ts', 'Position', [100 520 130 550]);
add_block('simulink/Discrete/Unit Delay', [mdl '/Delay Fw'], 'SampleTime', 'P1_Ts', 'InitialCondition', 'zeros(6,1)', 'Position', [1100 300 1130 330]);
add_block('simulink/Discrete/Unit Delay', [mdl '/Delay Mapp'], 'SampleTime', 'P1_Ts', 'InitialCondition', 'zeros(2,1)', 'Position', [1100 380 1130 410]);
add_block('simulink/Discrete/Unit Delay', [mdl '/Delay aux'], 'SampleTime', 'P1_Ts', 'InitialCondition', 'zeros(13,1)', 'Position', [560 620 590 650]);
add_block('simulink/Sinks/Stop Simulation', [mdl '/Stop'], 'Position', [700 40 730 70]);

% ---------- MATLAB Function blocks (extrinsic wrappers) ----------
fb(mdl, 'Driver steering', [180 30 300 80], ['function delta = f(t)' NL 'coder.extrinsic(''sl_steer'');' NL ...
  'delta = 0;' NL 'delta = sl_steer(t);']);
fb(mdl, 'True axle loads', [180 600 320 660], ['function Fz = f(aux_prev)' NL 'coder.extrinsic(''sl_loads'');' NL ...
  'Fz = zeros(3,1);' NL 'Fz = sl_loads(aux_prev(6), aux_prev(9), 0);']);
fb(mdl, 'Vehicle sensors (truth)', [180 470 330 560], ['function aux = f(s, Fw_prev, delta, Fz)' NL ...
  'coder.extrinsic(''sl_truth'');' NL 'aux = zeros(13,1);' NL 'aux = sl_truth(s, Fw_prev, delta, Fz);']);
fb(mdl, 'Sensors', [380 470 500 560], ['function [y, v, ay, axm] = f(aux)' NL 'coder.extrinsic(''sl_sensors'');' NL ...
  'y = zeros(2,1); v = 0; ay = 0; axm = 0;' NL '[y, v, ay, axm] = sl_sensors(aux);']);
fb(mdl, 'Driver brake pedal', [560 120 680 180], ['function [D, Fdrive] = f(t, v)' NL 'coder.extrinsic(''sl_scenario'');' NL ...
  'D = 0; Fdrive = 0;' NL '[D, Fdrive] = sl_scenario(t, v);']);
fb(mdl, 'Kalman filter', [560 300 680 400], ['function xh = f(Mapp_prev, delta, y, v)' NL 'coder.extrinsic(''sl_kf'');' NL ...
  'xh = zeros(6,1);' NL 'xh = sl_kf(Mapp_prev, delta, y, v);']);
fb(mdl, 'Reference + LQR', [740 300 860 380], ['function [Mdem, xr] = f(xh, v, delta)' NL 'coder.extrinsic(''sl_lqr'');' NL ...
  'Mdem = zeros(2,1); xr = zeros(4,1);' NL '[Mdem, xr] = sl_lqr(xh, v, delta);']);
fb(mdl, 'Axle load estimate', [740 470 880 530], ['function Fz_est = f(axm, aux_prev)' NL 'coder.extrinsic(''sl_loads'');' NL ...
  'Fz_est = zeros(3,1);' NL 'Fz_est = sl_loads(axm, aux_prev(9), 1);']);
fb(mdl, 'Brake and regen allocator', [920 200 1060 380], ['function [Fw, Fm] = f(D, Fdrive, Mdem, v, Fz_est, ay)' NL ...
  'coder.extrinsic(''sl_alloc'');' NL 'Fw = zeros(6,1); Fm = zeros(6,1);' NL '[Fw, Fm] = sl_alloc(D, Fdrive, Mdem, v, Fz_est, ay);']);
fb(mdl, 'Energy and applied moments', [920 420 1060 520], ['function [Pbat, Pbrk, Mapp] = f(Fw, Fm, Fz, aux)' NL ...
  'coder.extrinsic(''sl_energy'');' NL 'Pbat = 0; Pbrk = 0; Mapp = zeros(2,1);' NL '[Pbat, Pbrk, Mapp] = sl_energy(Fw, Fm, Fz, aux);']);
fb(mdl, 'Tractor + e-semitrailer (nonlinear)', [1000 560 1140 640], ['function sd = f(s, Fw, delta, Fz)' NL ...
  'coder.extrinsic(''sl_plant'');' NL 'sd = zeros(8,1);' NL 'sd = sl_plant(s, Fw, delta, Fz);']);
fb(mdl, 'Stop check', [560 30 660 80], ['function stop = f(t, aux)' NL 'coder.extrinsic(''sl_stop'');' NL ...
  'stop = 0;' NL 'stop = sl_stop(t, aux);']);

% ---------- logging ----------
tw(mdl, 'log t', 'P1_t', [380 20 440 40]);
tw(mdl, 'log aux', 'P1_aux', [380 600 440 620]);
tw(mdl, 'log Pbat', 'P1_Pbat', [1120 430 1180 450]);
tw(mdl, 'log Pbrk', 'P1_Pbrk', [1120 460 1180 480]);
tw(mdl, 'log Mdem', 'P1_Mdem', [920 140 980 160]);
tw(mdl, 'log xh', 'P1_xh', [740 420 800 440]);
tw(mdl, 'log s', 'P1_s', [180 700 240 720]);

% ---------- wiring ----------
L = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
L('Clock/1', 'ZOH t/1');
L('ZOH t/1', 'Driver steering/1'); L('ZOH t/1', 'Driver brake pedal/1'); L('ZOH t/1', 'Stop check/1'); L('ZOH t/1', 'log t/1');
L('Vehicle states/1', 'ZOH s/1'); L('Vehicle states/1', 'Tractor + e-semitrailer (nonlinear)/1');
L('ZOH s/1', 'Vehicle sensors (truth)/1'); L('ZOH s/1', 'log s/1');
L('Delay Fw/1', 'Vehicle sensors (truth)/2');
L('Driver steering/1', 'Vehicle sensors (truth)/3'); L('Driver steering/1', 'Kalman filter/2');
L('Driver steering/1', 'Reference + LQR/3'); L('Driver steering/1', 'Tractor + e-semitrailer (nonlinear)/3');
L('True axle loads/1', 'Vehicle sensors (truth)/4'); L('True axle loads/1', 'Energy and applied moments/3');
L('True axle loads/1', 'Tractor + e-semitrailer (nonlinear)/4');
L('Vehicle sensors (truth)/1', 'Sensors/1'); L('Vehicle sensors (truth)/1', 'Delay aux/1');
L('Vehicle sensors (truth)/1', 'Energy and applied moments/4'); L('Vehicle sensors (truth)/1', 'Stop check/2');
L('Vehicle sensors (truth)/1', 'log aux/1');
L('Delay aux/1', 'True axle loads/1'); L('Delay aux/1', 'Axle load estimate/2');
L('Sensors/1', 'Kalman filter/3'); L('Sensors/2', 'Kalman filter/4'); L('Sensors/2', 'Driver brake pedal/2');
L('Sensors/2', 'Reference + LQR/2'); L('Sensors/2', 'Brake and regen allocator/4');
L('Sensors/3', 'Brake and regen allocator/6'); L('Sensors/4', 'Axle load estimate/1');
L('Delay Mapp/1', 'Kalman filter/1');
L('Kalman filter/1', 'Reference + LQR/1'); L('Kalman filter/1', 'log xh/1');
L('Driver brake pedal/1', 'Brake and regen allocator/1'); L('Driver brake pedal/2', 'Brake and regen allocator/2');
L('Reference + LQR/1', 'Brake and regen allocator/3'); L('Reference + LQR/1', 'log Mdem/1');
L('Axle load estimate/1', 'Brake and regen allocator/5');
L('Brake and regen allocator/1', 'Delay Fw/1'); L('Brake and regen allocator/1', 'Energy and applied moments/1');
L('Brake and regen allocator/1', 'Tractor + e-semitrailer (nonlinear)/2');
L('Brake and regen allocator/2', 'Energy and applied moments/2');
L('Energy and applied moments/1', 'log Pbat/1'); L('Energy and applied moments/2', 'log Pbrk/1');
L('Energy and applied moments/3', 'Delay Mapp/1');
L('Tractor + e-semitrailer (nonlinear)/1', 'Vehicle states/1');
L('Stop check/1', 'Stop/1');
% unused second output of Reference + LQR -> terminator
add_block('simulink/Sinks/Terminator', [mdl '/Term xr'], 'Position', [900 360 910 370]);
L('Reference + LQR/2', 'Term xr/1');

save_system(mdl, fullfile(here, [mdl '.slx']));
fprintf('Built %s.slx. Run: sim(''%s''); sl_results\n', mdl, mdl);
end

function fb(mdl, name, pos, code)
% add a MATLAB Function block and set its code
blk = [mdl '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = code;
end

function tw(mdl, name, var, pos)
add_block('simulink/Sinks/To Workspace', [mdl '/' name], 'VariableName', var, ...
  'SaveFormat', 'Array', 'Position', pos);
end
