function sl_init(sc)
%SL_INIT  Prepare a scenario for the Simulink model P1_tractor_etrailer.slx.
%  Example:  sl_init(struct('type','bit','mode','prop','mu',0.3,'payload','light', ...
%                           'payload_ctrl','light','sensors','r_phi','seed',1,'Pbat',[]));
%  Sets the global P1SIM used by the block wrappers, resets their memory, and puts the
%  initial state, sample times and stop time in the base workspace for the block dialogs.
global P1SIM
here = fileparts(mfilename('fullpath')); addpath(here); addpath(fullfile(here, '..', 'src'));
if nargin < 1
  sc = struct('type','bit','mode','prop','mu',0.3,'payload','light','payload_ctrl','light', ...
              'sensors','r_phi','seed',1,'Pbat',[]);
end
P1SIM = scenario_setup(sc);
clear sl_sensors sl_kf                       % reset persistent memory of the stateful blocks
assignin('base', 'P1_s0', P1SIM.s0);
assignin('base', 'P1_Ts', P1SIM.dt_c);
assignin('base', 'P1_dt', P1SIM.dt_c / P1SIM.nsub);
assignin('base', 'P1_tend', P1SIM.t_end);
fprintf('P1 scenario ready: %s, mode %s, mu %.1f, payload %s (controller assumes %s)\n', ...
  sc.type, sc.mode, sc.mu, sc.payload, sc.payload_ctrl);
end
