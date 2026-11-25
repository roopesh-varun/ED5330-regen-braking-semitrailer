function K = gain_at(G, u)
%GAIN_AT  Linear interpolation of the scheduled LQR gain at speed u.
u = min(max(u, G.u(1)), G.u(end));
k = find(G.u <= u, 1, 'last');
if k == numel(G.u), K = G.K(:,:,k); return; end
lam = (u - G.u(k)) / (G.u(k+1) - G.u(k));
K = (1-lam)*G.K(:,:,k) + lam*G.K(:,:,k+1);
end
