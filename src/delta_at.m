function d = delta_at(S, t)
%DELTA_AT  Driver steering input (rad) at time t for the scenario in S.
if strcmp(S.sc.type, 'bit')
  d = S.delta0;
else
  d = S.da * sin(2*pi*(t - 1)/S.T) * (t >= 1 && t <= 1 + S.T);
end
end
