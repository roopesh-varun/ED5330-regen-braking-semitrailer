function R = sl_results()
%SL_RESULTS  Summarise the last Simulink run and compare it with the script (run_sim).
global P1SIM
S = P1SIM;
t = evalin('base', 'P1_t'); Pbat = evalin('base', 'P1_Pbat'); Pbrk = evalin('base', 'P1_Pbrk');
aux = evalin('base', 'P1_aux');
ib = find(t >= S.t_b, 1);
R.E_bat = sum(Pbat(ib:end)) * S.dt_c;
R.E_brk = sum(Pbrk(ib:end)) * S.dt_c;
R.regen_share = R.E_bat / max(R.E_brk, 1);
R.peak_articulation_deg = rad2deg(max(abs(aux(:,5))));
Rs = run_sim(S.sc);
fprintf('Simulink: regen share %.4f, battery energy %.4f MJ, peak articulation %.2f deg\n', ...
  R.regen_share, R.E_bat/1e6, R.peak_articulation_deg);
fprintf('Script  : regen share %.4f, battery energy %.4f MJ\n', Rs.regen_share, Rs.E_bat/1e6);
fprintf('Difference in regen share: %.2e (expected near zero; same code, same block order)\n', ...
  R.regen_share - Rs.regen_share);
end
