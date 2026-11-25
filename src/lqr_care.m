function [K, S] = lqr_care(A, B, Q, R)
%LQR_CARE  Continuous-time LQR gain via the Hamiltonian matrix (no toolbox needed).
%  Solves A'S + SA - S B R^-1 B' S + Q = 0 and returns K = R^-1 B' S.
%  Equivalent to MATLAB's lqr(A,B,Q,R) for stabilisable/detectable problems.
n = size(A,1);
H = [A, -B*(R\B'); -Q, -A'];
[V, D] = eig(H);
ev = diag(D);
idx = find(real(ev) < 0);
if numel(idx) ~= n, error('lqr_care: Hamiltonian has %d stable eigenvalues, expected %d', numel(idx), n); end
X1 = V(1:n, idx); X2 = V(n+1:end, idx);
S = real(X2 / X1);
S = (S + S')/2;
K = R \ (B' * S);
end
