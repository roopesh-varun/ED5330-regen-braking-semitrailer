function Fy = tyre_lat(alpha, Fz, Fx, mu, Cn)
%TYRE_LAT  Lateral force of one wheel: tanh law with friction-circle combined slip.
%  Fy0 = -mu*Fz*tanh(Cn*alpha/mu)  (initial slope -Cn*Fz)
%  Fy  = Fy0 * sqrt(1 - (Fx/(mu*Fz))^2)
%  Follows the tyre-model structure of Erdinc et al. (2023).
Fmax = mu .* max(Fz, 1);
Fy0  = -Fmax .* tanh(Cn .* alpha ./ mu);
use  = min(abs(Fx) ./ Fmax, 1);
Fy   = Fy0 .* sqrt(1 - use.^2);
end
