function stop = sl_stop(t, aux)
%SL_STOP  Block "Stop": end the run when the truck has stopped or has jackknifed (|phi| > 60 deg).
global P1SIM
stop = double((t > P1SIM.t_b && aux(1) < 3) || abs(aux(5)) > deg2rad(60));
end
