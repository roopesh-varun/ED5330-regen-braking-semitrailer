function delta = sl_steer(t)
%SL_STEER  Block "Driver steering": front steer angle (rad) at time t.
global P1SIM
delta = delta_at(P1SIM, t);
end
