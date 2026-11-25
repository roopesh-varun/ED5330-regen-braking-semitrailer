function [y, v_meas, ay_meas, ax_m] = sl_sensors(aux)
%SL_SENSORS  Block "Sensors": noisy yaw rate (and articulation angle), wheel speed,
%  lateral acceleration estimate and filtered longitudinal acceleration.
%  y is always 2x1; with the yaw-rate-only sensor set the second entry is unused (0).
global P1SIM
persistent axf
if isempty(axf), axf = 0; end
KF = P1SIM.KF;
x4 = [aux(2); aux(3); aux(4); aux(5)];             % [vy1 r1 phid phi]
ym = KF.C * x4 + sqrt(diag(KF.R)) .* randn(size(KF.C, 1), 1);
y = zeros(2, 1); y(1:numel(ym)) = ym;
v_meas = aux(1);
ay_meas = v_meas * y(1);
axf = 0.8*axf + 0.2*aux(6);
ax_m = axf;
end
