function sd = sl_plant(s, Fw, delta, Fz)
%SL_PLANT  Block "Tractor + e-semitrailer (nonlinear)": state derivative for the Integrator.
global P1SIM
sd = plant_deriv(s(:), Fw(:)', delta, P1SIM.mu, Fz(:), P1SIM.P);
end
