"""Report figures and demo GIF from results/ (Python 3, numpy, pandas, scipy, matplotlib)."""
import numpy as np, pandas as pd, scipy.io as sio, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation, PillowWriter

import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = os.path.join(ROOT, 'results') + os.sep; FIG = os.path.join(ROOT, 'figs') + os.sep
INK, INK2, GRID, SURF = '#0b0b0b', '#52514e', '#e5e4df', '#fcfcfb'
COL = {'fric_esc': '#8a8986', 'emax': '#eb6834', 'emax_lqr': '#1baf7a', 'prop': '#2a78d6'}
NAME = {'fric_esc': 'B1 Friction-only ESC', 'emax': 'B2 Max-regen', 'emax_lqr': 'B3 Max-regen + LQR', 'prop': 'Proposed'}
ORDER = ['fric_esc', 'emax', 'emax_lqr', 'prop']
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.edgecolor': INK2, 'axes.labelcolor': INK,
                     'xtick.color': INK2, 'ytick.color': INK2, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8,
                     'axes.spines.top': False, 'axes.spines.right': False, 'figure.facecolor': 'white',
                     'axes.facecolor': 'white', 'legend.frameon': False, 'axes.titleweight': 'bold', 'axes.titlesize': 11})
# geometry (Erdinc et al. 2023 parameters)
a1, L1, l1c = 1.534, 4.085, 3.7725; b1 = L1 - a1; c1 = l1c - a1; l2c, e2 = 7.05, 1.9315; d2 = l2c - e2; W = 2.5

L = sio.loadmat(RES + 'logs_light.mat', squeeze_me=True, struct_as_record=False)
def lg(name): return L[name]
d = pd.read_csv(RES + 'metrics.csv')

def outlines(X, Y, p1, ph):
    p2 = p1 - ph
    e1 = np.array([np.cos(p1), np.sin(p1)]); n1 = np.array([-np.sin(p1), np.cos(p1)])
    e2v = np.array([np.cos(p2), np.sin(p2)]); n2 = np.array([-np.sin(p2), np.cos(p2)])
    P1 = np.array([X, Y])
    tr = [P1 + (a1 + 1.4) * e1 + W/2*n1, P1 + (a1 + 1.4) * e1 - W/2*n1, P1 - (b1 + 0.6) * e1 - W/2*n1, P1 - (b1 + 0.6) * e1 + W/2*n1]
    H = P1 - c1 * e1
    st = [H + 1.2 * e2v + W/2*n2, H + 1.2 * e2v - W/2*n2, H - (l2c + 1.5) * e2v - W/2*n2, H - (l2c + 1.5) * e2v + W/2*n2]
    return np.array(tr), np.array(st)

# ---------- Fig 1: paths and final poses, brake in turn mu = 0.3 ----------
fig, ax = plt.subplots(figsize=(7.2, 5.6))
for m in ORDER:
    g = lg(f'bit_{m}_mu30_light')
    ax.plot(g.X, g.Y, color=COL[m], lw=2, label=NAME[m] + (' (jackknife)' if m == 'emax' else ''))
for m in ['emax', 'prop']:
    g = lg(f'bit_{m}_mu30_light')
    for k in ([len(g.t) - 1] if m == 'emax' else [int(len(g.t) * 0.55), len(g.t) - 1]):
        tr, st = outlines(g.X[k], g.Y[k], g.psi1[k], g.phi[k])
        ax.add_patch(Polygon(st, closed=True, fc=COL[m], alpha=0.18, ec=COL[m], lw=1.2))
        ax.add_patch(Polygon(tr, closed=True, fc=COL[m], alpha=0.45, ec=COL[m], lw=1.2))
gb = lg('bit_prop_mu30_light'); ib = np.argmax(gb.t >= 2.0)
ax.plot(gb.X[ib], gb.Y[ib], 'o', color=INK, ms=6); ax.annotate('braking starts', (gb.X[ib], gb.Y[ib]), (gb.X[ib] - 2, gb.Y[ib] + 6), color=INK2, fontsize=9, arrowprops=dict(arrowstyle='-', color=INK2))
ax.set_aspect('equal'); ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
ax.set_title('Snow (μ = 0.3): max-regen jackknifes, proposed holds the curve', loc='left', fontsize=10.5)
ax.legend(loc='lower right', fontsize=8.5)
fig.tight_layout(); fig.savefig(FIG + 'fig1_paths_mu03.png', dpi=200); plt.close(fig)

# ---------- Fig 2: time histories mu = 0.3 ----------
fig, axs = plt.subplots(3, 1, figsize=(7.2, 7.0), sharex=True)
for m in ORDER:
    g = lg(f'bit_{m}_mu30_light')
    axs[0].plot(g.t, np.degrees(g.phi), color=COL[m], lw=2, label=NAME[m])
    axs[1].plot(g.t, np.degrees(g.alpha[:, 1]), color=COL[m], lw=2)
    axs[2].plot(g.t, g.Pbat / 1e3, color=COL[m], lw=2)
axs[0].set_ylabel('Articulation angle (deg)'); axs[1].set_ylabel('Drive-axle side-slip (deg)'); axs[2].set_ylabel('Battery charging (kW)')
axs[0].set_ylim(-5, 65); axs[1].set_ylim(-60, 10)
for a_ in axs: a_.axvline(2.0, color=INK2, lw=1, ls='--')
axs[0].text(2.1, 55, 'braking starts', color=INK2, fontsize=9)
axs[2].set_xlabel('Time (s)')
axs[0].legend(loc='upper right', fontsize=8.5, ncol=2)
axs[0].set_title('Max-regen saturates the drive axle; the proposed split keeps it gripping', loc='left', fontsize=10.5)
fig.tight_layout(); fig.savefig(FIG + 'fig2_timehist_mu03.png', dpi=200); plt.close(fig)

# ---------- Fig 3: regen share by case, unstable marked ----------
A = d[d.tag == 'A'].copy()
cases = [(p, mu) for p in ['light', 'rated', 'over'] for mu in [0.3, 0.6, 0.9]]
fig, ax = plt.subplots(figsize=(8.6, 4.4))
bw = 0.2
for j, m in enumerate(ORDER):
    xs, ys, uns = [], [], []
    for i, (p, mu) in enumerate(cases):
        r = A[(A['mode'] == m) & (A.payload == p) & (A.mu == mu)].iloc[0]
        xs.append(i + (j - 1.5) * bw); ys.append(100 * r.regen_share); uns.append(r.unstable)
    ax.bar(xs, ys, bw * 0.92, color=COL[m], label=NAME[m], hatch=None)
    for x, y, u in zip(xs, ys, uns):
        if u:
            ax.bar(x, y, bw * 0.92, color='white', edgecolor=COL[m], hatch='////', lw=1)
            ax.text(x, y + 1.5, '×', ha='center', color=INK, fontsize=11, fontweight='bold')
ax.set_xticks(range(len(cases))); ax.set_xticklabels([f'{p}\nμ={mu}' for p, mu in cases], fontsize=8.5)
ax.set_ylabel('Braking energy into battery (%)'); ax.set_ylim(0, 100)
pr = A[A['mode']=='prop'].regen_share*100; nun = int(A[A['mode']=='emax'].unstable.sum())
ax.set_title(f'Proposed recovers {pr.min():.0f}–{pr.max():.0f}% with no instability; max-regen fails {nun} of 9 (×)', loc='left', fontsize=10.5)
ax.text(-0.42, 2, 'B1: 0% by design', fontsize=7.5, color=INK2, rotation=90, va='bottom')
ax.legend(fontsize=8.5, ncol=4, loc='upper right')
ax.text(8.6, -22, 'Hatched bar with × = side-slip limit crossed (5° drive axle / 3° semitrailer axle)', ha='right', fontsize=8, color=INK2)
fig.tight_layout(); fig.savefig(FIG + 'fig3_regen_by_case.png', dpi=200); plt.close(fig)

# ---------- Fig 4: trade-off scatter ----------
fig, ax = plt.subplots(figsize=(7.2, 4.8))
mk = {0.3: 'o', 0.6: 's', 0.9: '^'}
for m in ORDER:
    s = A[A['mode'] == m]
    for mu in [0.3, 0.6, 0.9]:
        q = s[s.mu == mu]
        ax.scatter(100 * q.regen_share, q.phi_err_max_deg, s=46, marker=mk[mu], color=COL[m], edgecolor='white', lw=0.8, zorder=3,
                   label=NAME[m] if mu == 0.3 else None)
ax.set_yscale('log'); ax.set_xlabel('Braking energy into battery (%)'); ax.set_ylabel('Peak articulation-angle error (deg, log)')
ax.axhline(10, color=INK2, lw=1, ls='--'); ax.text(30, 12, 'jackknife region', color=INK2, fontsize=9)
ax.set_title('Proposed sits in the safe, high-energy corner', loc='left', fontsize=10.5)
ax.text(0.99, 0.02, 'marker: ● μ 0.3   ■ 0.6   ▲ 0.9', transform=ax.transAxes, ha='right', fontsize=8.5, color=INK2)
ax.legend(fontsize=8.5, loc='lower left', bbox_to_anchor=(0.02, 0.25))
fig.tight_layout(); fig.savefig(FIG + 'fig4_tradeoff.png', dpi=200); plt.close(fig)

# ---------- Fig 5: poles vs speed ----------
pl = pd.read_csv(RES + 'poles.csv')
fig, ax = plt.subplots(figsize=(7.2, 4.6))
for loop, c, lab in [('open', '#8a8986', 'Open loop'), ('closed', '#2a78d6', 'Closed loop (LQR)')]:
    q = pl[(pl.payload == 'light') & (pl.loop == loop) & (pl.re > -12)]
    sc = ax.scatter(q.re, q.im, c=c, s=10 + q.u, alpha=0.75, label=lab, edgecolor='none')
ax.set_xlabel('Real part (1/s)'); ax.set_ylabel('Imaginary part (rad/s)')
q = pl[(pl.payload == 'light') & (pl.loop == 'open') & (pl.u == 30) & (pl.im > 0)].iloc[0]
ax.annotate('open-loop sway pair at 30 m/s, ζ ≈ 0.39', (q.re, q.im), (-8.6, 3.1), fontsize=9, color=INK2, arrowprops=dict(arrowstyle='-', color=INK2))
q = pl[(pl.payload == 'light') & (pl.loop == 'closed') & (pl.u == 30) & (pl.im > 0)].iloc[0]
ax.annotate('with LQR, ζ ≈ 0.80', (q.re, q.im), (-8.6, 2.55), fontsize=9, color=INK2, arrowprops=dict(arrowstyle='-', color=INK2))
ax.set_title('LQR doubles sway-mode damping at 30 m/s (light payload)', loc='left', fontsize=10.5)
ax.text(0.30, 0.10, 'speeds 5–30 m/s; larger dot = faster', transform=ax.transAxes, ha='left', fontsize=8.5, color=INK2)
ax.legend(fontsize=8.5, loc='lower left')
ax.text(0.30, 0.04, 'closed-loop fast pole near −35 1/s not shown', transform=ax.transAxes, ha='left', fontsize=8.5, color=INK2)
fig.tight_layout(); fig.savefig(FIG + 'fig5_poles.png', dpi=200); plt.close(fig)

# ---------- Fig 6: Kalman filter estimate ----------
g = lg('bit_prop_mu30_light'); dt = 0.01
phid_true = g.phid
fig, axs = plt.subplots(2, 1, figsize=(7.2, 4.8), sharex=True)
axs[0].plot(g.t, np.degrees(g.phi), color='#8a8986', lw=2.5, label='true')
axs[0].plot(g.t, np.degrees(g.xh[:, 3]), color='#2a78d6', lw=1.2, label='Kalman estimate')
axs[1].plot(g.t, np.degrees(phid_true), color='#8a8986', lw=2.5, label='true (not measured)')
axs[1].plot(g.t, np.degrees(g.xh[:, 2]), color='#2a78d6', lw=1.2, label='Kalman estimate')
axs[0].set_ylabel('Articulation angle (deg)'); axs[1].set_ylabel('Articulation rate (deg/s)'); axs[1].set_xlabel('Time (s)')
axs[0].legend(fontsize=8.5); axs[1].legend(fontsize=8.5)
ib6 = np.argmax(g.t >= 2.0); rms6 = np.degrees(np.sqrt(np.mean((g.xh[ib6:, 2] - g.phid[ib6:])**2)))
axs[0].set_title(f'Articulation-rate estimate: unbiased, noisy (RMS {rms6:.1f}°/s)', loc='left', fontsize=10.5)
fig.tight_layout(); fig.savefig(FIG + 'fig6_kalman.png', dpi=200); plt.close(fig)

# ---------- Animation ----------
ge, gp = lg('bit_emax_mu30_light'), lg('bit_prop_mu30_light')
n = max(len(ge.t), len(gp.t)); frames = list(range(0, n, 8))
fig, axs = plt.subplots(1, 2, figsize=(9, 4.8))
allx = np.concatenate([ge.X, gp.X]); ally = np.concatenate([ge.Y, gp.Y])
for a_, g, m in [(axs[0], ge, 'emax'), (axs[1], gp, 'prop')]:
    a_.set_xlim(allx.min() - 15, allx.max() + 15); a_.set_ylim(ally.min() - 15, ally.max() + 15); a_.set_aspect('equal')
    a_.set_title(NAME[m], color=COL[m]); a_.plot(g.X, g.Y, color=GRID, lw=1)
pt = {}
for a_, g, m in [(axs[0], ge, 'emax'), (axs[1], gp, 'prop')]:
    pt[m] = (a_.add_patch(Polygon(np.zeros((4, 2)), fc=COL[m], alpha=0.25, ec=COL[m])),
             a_.add_patch(Polygon(np.zeros((4, 2)), fc=COL[m], alpha=0.6, ec=COL[m])),
             a_.plot([], [], color=COL[m], lw=2)[0], a_.text(0.03, 0.95, '', transform=a_.transAxes, fontsize=9, va='top'))
fig.suptitle('Brake-in-turn on snow, μ = 0.3, light payload', fontsize=11)
def upd(k):
    for g, m in [(ge, 'emax'), (gp, 'prop')]:
        kk = min(k, len(g.t) - 1)
        tr, st = outlines(g.X[kk], g.Y[kk], g.psi1[kk], g.phi[kk])
        pt[m][0].set_xy(st); pt[m][1].set_xy(tr); pt[m][2].set_data(g.X[:kk], g.Y[:kk])
        pt[m][3].set_text(f't = {g.t[kk]:.1f} s\nv = {3.6*g.v[kk]:.0f} km/h\narticulation {np.degrees(g.phi[kk]):.0f}°\nbattery {g.Pbat[kk]/1e3:.0f} kW')
    return []
ani = FuncAnimation(fig, upd, frames=frames, blit=False)
ani.save(FIG + 'anim_brake_in_turn_mu03.gif', writer=PillowWriter(fps=12), dpi=90)
print('figs done')
