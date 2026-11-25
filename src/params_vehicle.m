function P = params_vehicle(payload)
%PARAMS_VEHICLE  Tractor-semitrailer parameters for ED5330 project P1.
%  payload: 'light' | 'rated' | 'over'
%
%  Geometry, tractor mass/inertia, published semitrailer mass/inertia, CoG heights,
%  track width and normalised cornering stiffness (6 per rad) are taken from
%  Erdinc, Jonasson, Sadeghi Kati, Jacobson, Fredriksson, Laine, "Modelling of
%  articulated heavy vehicles for computation of accurate safe operating envelope
%  for yaw stability", Technology Convergence 2023, Appendix table
%  (https://research.chalmers.se/publication/538592).
%  Heavier payloads scale the semitrailer mass and yaw inertia together with the
%  CoG kept in place (REPRESENTATIVE assumption).
%  Electric drive and battery limits are REPRESENTATIVE values chosen for this study.

if nargin < 1, payload = 'light'; end
P.g = 9.81;

% --- tractor (published) ---
P.m1  = 10250;          % kg
P.I1  = 48758;          % kg m^2, yaw
P.L1  = 4.085;          % m, wheelbase
P.a1  = 1.534;          % m, CoG to front axle
P.b1  = P.L1 - P.a1;    % m, CoG to drive axle
P.l1c = 3.7725;         % m, front axle to coupling
P.c1  = P.l1c - P.a1;   % m, CoG to coupling (behind CoG)
P.h1  = 0.92;           % m, CoG height
P.hc  = 1.013;          % m, coupling height

% --- semitrailer (published at 13.5 t) ---
m2pub = 13500; I2pub = 83913;
switch payload
  case 'light', P.m2 = m2pub;
  case 'rated', P.m2 = 30000;     % representative rated load
  case 'over',  P.m2 = 39000;     % rated + 30 %, representative overloading
  otherwise, error('unknown payload %s', payload);
end
P.payload = payload;
P.I2  = I2pub * P.m2 / m2pub;   % inertia scaled with mass
P.l2c = 7.05;           % m, coupling to semitrailer axle
P.e2  = 1.9315;         % m, semitrailer CoG to axle
P.d2  = P.l2c - P.e2;   % m, coupling to semitrailer CoG
P.h2  = 0.92;           % m, CoG height (published value kept for all payloads)

% --- tyres ---
P.w   = 2.0;            % m, track width
P.Cn  = 6;              % 1/rad, cornering stiffness normalised by axle load
[Fz0, Fc0] = axle_loads(P, 0, 0);
P.Fz0 = Fz0;            % static axle loads [front drive trailer] (N)
P.Fc0 = Fc0;            % static coupling load (N)
P.Cf = P.Cn * Fz0(1); P.Cr = P.Cn * Fz0(2); P.Ct = P.Cn * Fz0(3);

% --- electric drive (REPRESENTATIVE) ---
P.Pd_max  = 350e3;      % W, tractor e-axle regen power
P.Fd_max  = 50e3;       % N, tractor e-axle wheel force limit (both wheels)
P.Pt_max  = 120e3;      % W, each semitrailer motor (left and right)
P.Ft_max  = 20e3;       % N, each semitrailer motor wheel force
P.Pbat_max = 500e3;     % W, battery charge-power limit (shared)
P.eta     = 0.85;       % regen efficiency, wheel to battery

% --- allocator safety factors for the proposed controller ---
P.s_front = 0.90; P.s_drive = 0.70; P.s_trail = 0.85;
end
