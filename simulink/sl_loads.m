function Fz = sl_loads(ax, Fxt, use_controller_model)
%SL_LOADS  Block "Axle loads": quasi-static vertical loads with longitudinal load transfer.
%  use_controller_model = 1 for the controller's estimate (assumed payload), 0 for the plant.
global P1SIM
if use_controller_model, P = P1SIM.Pc; else, P = P1SIM.P; end
Fz = axle_loads(P, ax, Fxt);
end
