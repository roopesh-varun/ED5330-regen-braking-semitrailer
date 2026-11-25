function [Pbat, Pbrk, Mapp] = sl_energy(Fw, Fm, Fz, aux)
%SL_ENERGY  Block "Energy and applied moments": battery charging power, braking power,
%  and the yaw moments the wheel forces apply (fed back to the Kalman filter).
global P1SIM
P = P1SIM.P; mu = P1SIM.mu;
Fw = Fw(:)'; Fm = Fm(:)';
Fzw = [Fz(1) Fz(1) Fz(2) Fz(2) Fz(3) Fz(3)] / 2;
Fwc = max(min(Fw, mu*Fzw), -mu*Fzw);
vw  = [aux(1) aux(1) aux(1) aux(1) aux(10) aux(10)];
Bw  = max(-Fwc, 0);
mreg = min(max(Fm, 0), Bw);
mtrac = max(-Fm, 0) .* (Fwc > 0);
Pbat = P.eta * sum(mreg .* vw) - sum(mtrac .* vw) / P.eta;
Pbrk = sum(Bw .* vw);
hw = P.w/2;
Mapp = [hw*(Fw(2)-Fw(1)) + hw*(Fw(4)-Fw(3)); hw*(Fw(6)-Fw(5))];
end
