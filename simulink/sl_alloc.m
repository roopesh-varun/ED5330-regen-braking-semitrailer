function [Fw, Fm] = sl_alloc(D, Fdrive, Mdem, v_meas, Fz_est, ay_meas)
%SL_ALLOC  Block "Brake and regen allocator" (the proposed method or a baseline, by mode).
global P1SIM
S = P1SIM;
[fw, fm] = allocator(S.sc.mode, D, Fdrive, Mdem(:)', v_meas, Fz_est(:), S.mu, ay_meas, S.Pc);
Fw = fw(:); Fm = fm(:);
end
