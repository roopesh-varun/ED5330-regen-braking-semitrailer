%MAIN  ED5330 project P1: stability-constrained regenerative braking allocation for a
%  battery-electric tractor with an electrified semitrailer.
%  Run this file in MATLAB (R2019b or later) from the project folder.
%  Steps (comment out what you do not need):
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'src')); addpath(fullfile(here, 'simulink'));

%% 1. Course checks on the design model (eigenvalues, controllability, observability)
run(fullfile(here, 'src', 'check_model.m'));

%% 2. Full run matrix (77 runs, a few minutes) -> results/metrics.csv
run(fullfile(here, 'src', 'run_all.m'));

%% 3. Live demo: max-regen jackknifes, proposed holds the curve (mu = 0.3)
animate_run(0.3, 'light', {'emax', 'prop'});

%% 4. Simulink model: build, run the default case, compare with the script
build_simulink_model;
sim('P1_tractor_etrailer');
sl_results;
% another case without rebuilding, e.g. max-regen at mu 0.3:
% sl_init(struct('type','bit','mode','emax','mu',0.3,'payload','light','payload_ctrl','light','sensors','r_phi','seed',1,'Pbat',[]));
% sim('P1_tractor_etrailer'); sl_results;
