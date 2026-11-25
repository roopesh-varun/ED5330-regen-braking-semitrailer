function aux = sl_truth(s, Fw_prev, delta, Fz)
%SL_TRUTH  Block "Vehicle sensors (true values)": quantities a sensor would see, from the
%  plant state and the wheel forces applied over the last period.
%  aux = [vx1 vy1 r1 phid phi ax1 ay1 ay2 Fxt vx2 alpha_f alpha_r alpha_t]'
global P1SIM
[~, a] = plant_deriv(s(:), Fw_prev(:)', delta, P1SIM.mu, Fz(:), P1SIM.P);
aux = [a.vx1; a.vy1; a.r1; s(8); s(4); a.ax1; a.ay1; a.ay2; a.Fxt; a.vx2; a.alpha(:)];
end
