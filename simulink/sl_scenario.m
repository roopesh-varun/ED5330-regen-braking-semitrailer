function [D, Fdrive] = sl_scenario(t, v)
%SL_SCENARIO  Block "Driver brake pedal": brake demand after t_b, speed hold before it.
global P1SIM
S = P1SIM;
if t < S.t_b
  D = 0; Fdrive = max(S.mtot * 1.0 * (S.v0 - v), 0);
else
  D = S.mtot * S.adem; Fdrive = 0;
end
end
