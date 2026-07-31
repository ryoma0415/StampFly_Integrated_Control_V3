#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""20260731_182025 解析図: 誤差時系列 / リプレイ比較 / b_m軌跡 / RMS80アーチファクト."""
import json
import math
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

SP = '/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad/a0731b'
PL = SP + '/plots'

installed = {f.name for f in fm.fontManager.ttflist}
for jp in ('Hiragino Sans', 'Hiragino Maru Gothic Pro', 'YuGothic', 'Noto Sans CJK JP'):
    if jp in installed:
        plt.rcParams['font.family'] = jp
        break
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.size'] = 9

wrap = lambda a: (np.asarray(a) + 180.0) % 360.0 - 180.0

df = pd.read_csv(SP + '/log_corrected.csv')
t = df['elapsed_time'].values
truth = df['yaw_true_corrected_deg'].values
state = df['tlm_state'].values
t_fly = t[np.argmax(state == 4)]
t_land = t[np.argmax(state == 5)]
flight_w = (t >= t_fly) & (t < t_land)
est = {
    'EKF1':     np.degrees(df['tlm_yaw_est_rad'].values),
    'EKF2':     np.degrees(df['tlm_ekf2_yaw_rad'].values),
    'gyro_int': np.degrees(df['tlm_yaw_gyro_int_rad'].values),
    'Madgwick': np.degrees(df['tlm_yaw_rad'].values),
}
C = {'EKF1': 'tab:red', 'EKF2': 'tab:purple', 'gyro_int': 'tab:blue',
     'Madgwick': 'tab:green'}
stats = json.load(open(SP + '/yaw_eval_stats.json'))
TSPANS = stats['_windows']['turn_spans']

def shade(ax):
    for a, b in TSPANS:
        ax.axvspan(a, b, color='orange', alpha=0.10, lw=0)
    ax.axvline(t_land, color='gray', ls=':', lw=0.8)

# ============ Fig 1: 4系統誤差時系列 ============
fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True,
                        gridspec_kw={'height_ratios': [2, 2, 1.2]})
ax = axs[0]
for name in est:
    err = wrap(est[name] - truth)
    off = err[(t >= 0.17) & (t <= 2.0)].mean()
    ax.plot(t, wrap(err - off), color=C[name], lw=1.0,
            label=f"{name} (RMS {stats[name]['adj']['rms']:.1f}°)")
ax.axhline(0, color='k', lw=0.5)
shade(ax)
ax.set_ylim(-30, 25)
ax.set_ylabel('ヨー誤差 [deg] (地上オフセット除去)')
ax.legend(ncol=4, loc='lower right', fontsize=8)
ax.set_title('20260731_182025 ヨー誤差 vs 復元真値 (橙=回頭マニューバ) — '
             'EKF2はt=20sの磁気ソフト再捕捉で回復、ヨー観測は全飛行未融合')
ax = axs[1]
ax.plot(t, df['tlm_bm_x_ut'], 'r-', lw=1, label='EKF1 b_m x')
ax.plot(t, df['tlm_bm_y_ut'], 'r--', lw=1, label='EKF1 b_m y')
ax.plot(t, df['tlm_ekf2_bm_x_ut'], 'm-', lw=0.8, label='EKF2 b_m x')
ax.plot(t, df['tlm_ekf2_bm_y_ut'], 'm--', lw=0.8, label='EKF2 b_m y')
shade(ax)
ax.set_ylabel('b_m [µT]')
ax.legend(ncol=4, fontsize=8, loc='lower left')
ax = axs[2]
u = np.unwrap(np.radians(truth))
tr_rate = np.gradient(u, t) * 180/np.pi
tr_rate = pd.Series(tr_rate).rolling(9, center=True, min_periods=1).mean().values
ax.plot(t, truth, 'k-', lw=0.9, label='truth ψ [deg]')
ax.plot(t, tr_rate, 'c-', lw=0.7, label='truth rate [deg/s]')
shade(ax)
ax.set_ylabel('deg | deg/s')
ax.set_xlabel('t [s]')
ax.legend(ncol=2, fontsize=8)
fig.tight_layout()
fig.savefig(PL + '/01_yaw_error_4sys.png', dpi=140)
plt.close(fig)

# ============ Fig 2: リプレイ比較 ============
z = np.load(SP + '/replay_series.npz', allow_pickle=True)
rt = z['R0_noyaw_t_s']
rtr = rt - rt[0]
rtruth = z['truth_deg']
rstate = z['state']
rfly = rstate == 4
FRAME = 90.434
rstats = json.load(open(SP + '/replay_stats.json'))

fig, axs = plt.subplots(2, 2, figsize=(13.5, 8.6))
fig.subplots_adjust(hspace=0.34, wspace=0.22, top=0.90)

# P1: 軌跡 (truth系へ揃える: sent系リプレイは+90.434)
ax = axs[0, 0]
ax.plot(rtr, rtruth, 'k--', lw=1.6, label='MoCap真値(復元)')
ax.plot(rtr, z['psi_ekf2_log'], color='tab:purple', lw=1.4,
        label='EKF2実機 (融合0%)')
ax.plot(rtr, np.degrees(z['R1_deadlock_psi_ekf2_rad']), color='tab:orange',
        lw=1.1, label='リプレイ(a) 実送信基準+現行ゲート → デッドロック再現')
ax.plot(rtr, wrap(np.degrees(z['R3b_fix_clean_psi_ekf2_rad']) + FRAME),
        color='tab:green', lw=1.1, label='リプレイ(b) 初回整列改造 (+90.4°表示)')
ax.plot(rtr, np.degrees(z['R2_correct_psi_ekf2_rad']), color='tab:blue',
        lw=1.1, label='リプレイ 正解基準')
ax.set_title('ヨー軌跡: デッドロック再現と初回整列による救済', fontsize=10)
ax.set_xlabel('t [s]'); ax.set_ylabel('ψ [deg]')
ax.legend(fontsize=7, loc='lower left')

# P2: 誤差 (それぞれの基準フレームで)
ax = axs[0, 1]
e_log2 = wrap(z['psi_ekf2_log'] - rtruth)
e_r2 = wrap(np.degrees(z['R2_correct_psi_ekf2_rad']) - rtruth)
e_r3a = wrap(np.degrees(z['R3a_fix_asis_psi_ekf2_rad']) - (rtruth - FRAME))
e_r3b = wrap(np.degrees(z['R3b_fix_clean_psi_ekf2_rad']) - (rtruth - FRAME))
ax.axhspan(-2, 2, color='#e7efe9', zorder=0)
ax.plot(rtr[rfly], e_log2[rfly], color='tab:purple', lw=1.2,
        label=f'EKF2実機 RMS {np.sqrt(np.mean(e_log2[rfly]**2)):.1f}°')
ax.plot(rtr[rfly], e_r3a[rfly], color='#c49000', lw=1.0,
        label=f"(b\') 整列のみ・実mocap列 RMS {rstats['R3a_fix_asis']['err_vs_ref']['raw_rms']:.1f}° (t=25sグリッチでラッチ)")
ax.plot(rtr[rfly], e_r3b[rfly], color='tab:green', lw=1.0,
        label=f"(b) 整列+グリッチ除去 RMS {rstats['R3b_fix_clean']['err_vs_ref']['raw_rms']:.1f}°")
ax.plot(rtr[rfly], e_r2[rfly], color='tab:blue', lw=1.0,
        label=f"正解基準 RMS {rstats['R2_correct']['err_vs_ref']['raw_rms']:.1f}°")
la = rstats['R3a_fix_asis']['latch_t'] - rt[0]
ax.axvline(la, color='#c49000', ls=':', lw=1.2)
ax.text(la + 0.3, -16, f'融合停止ラッチ\nt={la:.1f}s (グリッチ84連続棄却)',
        color='#c49000', fontsize=7)
ax.set_title('真値に対するヨー誤差 (飛行中・各自の基準フレーム)', fontsize=10)
ax.set_xlabel('t [s]'); ax.set_ylabel('誤差 [deg]')
ax.set_ylim(-22, 12)
ax.legend(fontsize=7, loc='lower right')

# P3: イノベーション
ax = axs[1, 0]
ax.axhspan(-30, 30, color='#e7efe9', zorder=0)
iv1 = np.degrees(z['R1_deadlock_yaw_innov_rad'])
iv3a = np.degrees(z['R3a_fix_asis_yaw_innov_rad'])
iv2 = np.degrees(z['R2_correct_yaw_innov_rad'])
ax.plot(rtr, iv1, color='tab:orange', lw=1.0, label='(a) 実送信: innov≈−90° 全棄却')
ax.plot(rtr, iv3a, color='#c49000', lw=0.9,
        label="(b\') 整列後は≈0、グリッチで+95°スパイク")
ax.plot(rtr, iv2, color='tab:blue', lw=0.9, label='正解基準: innov RMS 1.6°')
ax.axhline(30, color='k', lw=0.8, ls=':'); ax.axhline(-30, color='k', lw=0.8, ls=':')
la1 = rstats['R1_deadlock']['latch_t'] - rt[0]
ax.annotate(f'25連続棄却→ラッチ t={la1:.2f}s (実機 1.03s)', xy=(la1, -89),
            xytext=(6, -60), fontsize=8, color='tab:orange',
            arrowprops=dict(arrowstyle='->', color='tab:orange'))
ax.set_title('ヨー観測イノベーションと±30°ゲート', fontsize=10)
ax.set_xlabel('t [s]'); ax.set_ylabel('innov [deg]')
ax.legend(fontsize=7, loc='center right')

# P4: 融合受理状態
ax = axs[1, 1]
for i, (key, col, lab) in enumerate([
        ('R1_deadlock', 'tab:orange', "(a) 実送信+現行実装: 融合 0%"),
        ('R3a_fix_asis', '#c49000', "(b\') 初回整列のみ: 43% (t=25.1sで恒久停止)"),
        ('R3b_fix_clean', 'tab:green', "(b) 整列+グリッチ除去: 100%"),
        ('R2_correct', 'tab:blue', "正解基準+現行実装: 100%")]):
    acc = z[f'{key}_obs_accept']
    y0 = 3 - i
    m = acc == 1
    ax.plot(rtr[m], np.full(m.sum(), y0 + 0.18), '|', color=col, ms=7)
    stp = z[f'{key}_obs_stopped'] == 1
    if stp.any():
        ax.plot(rtr[stp], np.full(stp.sum(), y0 - 0.13), '|', color='r', ms=5)
    ax.text(0.3, y0 + 0.32, lab, fontsize=8, color=col)
ax.set_ylim(-0.6, 3.9)
ax.set_yticks([])
ax.set_xlabel('t [s]')
ax.set_title('ヨー観測の受理(上段ティック)と融合停止ラッチ(赤)', fontsize=10)
fig.suptitle('EKF2リプレイ: ブートストラップ・デッドロックの再現と「初回整列」改修の効果実証 '
             f"(正解基準RMS {rstats['R2_correct']['err_vs_ref']['raw_rms']:.2f}° / "
             f"実機EKF1 {stats['EKF1']['raw']['rms']:.1f}°)", fontsize=12)
fig.savefig(PL + '/02_replay_matrix.png', dpi=140)
plt.close(fig)

# ============ Fig 3: b_m 軌跡 ============
mb = json.load(open(SP + '/magbias_20260731_1820.json'))
db = mb['delta_b_ut']
fig, axs = plt.subplots(1, 3, figsize=(13.5, 4.4))
ax = axs[0]
bx1 = df['tlm_bm_x_ut'].values; by1 = df['tlm_bm_y_ut'].values
bx2 = df['tlm_ekf2_bm_x_ut'].values; by2 = df['tlm_ekf2_bm_y_ut'].values
ax.plot(t, np.hypot(bx1, by1), 'r-', lw=1, label='|b_m| EKF1')
ax.plot(t, np.hypot(bx2, by2), 'm-', lw=1, label='|b_m| EKF2')
rbx = z['R2_correct_bm_x_ut']; rby = z['R2_correct_bm_y_ut']
ax.plot(rtr, np.hypot(rbx, rby), 'b-', lw=1, label='|b_m| リプレイ(正解基準融合)')
ax.axhline(np.hypot(db[0], db[1]), color='k', ls='--', lw=0.9,
           label=f'magbiasフィット |Δb|={np.hypot(db[0], db[1]):.1f}µT')
ax.axhspan(13, 16, color='#f3e2dc', zorder=0)
ax.text(1, 14.0, '7/27残差帯 13–16µT', fontsize=7, color='#a04a2a')
shade(ax)
ax.set_xlabel('t [s]'); ax.set_ylabel('|b_m| [µT]')
ax.legend(fontsize=7); ax.set_title('ハードアイアン残差ノルムの収束', fontsize=10)
ax = axs[1]
w = flight_w
ax.plot(bx1[w], by1[w], '-', c='mistyrose', lw=0.6)
s1 = ax.scatter(bx1[w], by1[w], s=3, c=t[w], cmap='Reds', label='EKF1')
ax.plot(bx2[w], by2[w], '-', c='thistle', lw=0.6)
ax.scatter(bx2[w], by2[w], s=3, c=t[w], cmap='Purples', label='EKF2')
ax.scatter([db[0]], [db[1]], marker='*', s=140, c='k', zorder=5,
           label=f'magbias Δb ({db[0]:+.1f},{db[1]:+.1f})')
ax.scatter([-0.41], [-7.64], marker='*', s=140, facecolor='none',
           edgecolor='k', label='11:19 Δb (−0.4,−7.6)')
ax.scatter([rbx[np.isfinite(rbx)][-1]], [rby[np.isfinite(rby)][-1]],
           marker='x', s=70, c='b', label='リプレイ最終')
ax.axis('equal'); ax.legend(fontsize=7)
ax.set_xlabel('b_m x [µT]'); ax.set_ylabel('b_m y [µT]')
ax.set_title('b_m平面軌跡 (飛行中・色=時刻)', fontsize=10)
ax = axs[2]
glitch = df['mocap_glitch'].values.astype(int)
zx = df['tlm_mag_lev_x_ut'].values; zy = df['tlm_mag_lev_y_ut'].values
yawr = np.radians(truth)
cw = flight_w & (glitch == 0)
c_, s_ = np.cos(yawr), np.sin(yawr)
# 4パラメタフィットの再構成残差: z - R(psi)B0h' (B0h'はfitから)
fit_b0 = None
try:
    # fit b0h from magbias internals not saved -> refit quickly
    A = np.zeros((2*cw.sum(), 4))
    A[0::2, 0] = c_[cw]; A[0::2, 1] = -s_[cw]; A[0::2, 2] = 1
    A[1::2, 0] = s_[cw]; A[1::2, 1] = c_[cw]; A[1::2, 3] = 1
    b = np.empty(2*cw.sum()); b[0::2] = zx[cw]; b[1::2] = zy[cw]
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    fit_b0 = sol
    resx = zx - (c_*sol[0] - s_*sol[1])
    resy = zy - (s_*sol[0] + c_*sol[1])
    sc = ax.scatter(truth[cw], resx[cw], s=3, c='tab:blue', label='残差x')
    ax.scatter(truth[cw], resy[cw], s=3, c='tab:orange', label='残差y')
    ax.axhline(sol[2], color='tab:blue', ls='--', lw=1)
    ax.axhline(sol[3], color='tab:orange', ls='--', lw=1)
    ax.set_xlabel('truth ψ [deg]'); ax.set_ylabel('mag_lev − R(ψ)B0h\' [µT]')
    ax.set_title(f'ヘディング一周残差: Δb=({sol[2]:+.1f},{sol[3]:+.1f})µT '
                 f'(励振239°)', fontsize=10)
    ax.legend(fontsize=7)
except Exception as e:
    print('fit panel skip', e)
fig.tight_layout()
fig.savefig(PL + '/03_bm_heading.png', dpi=140)
plt.close(fig)

# ============ Fig 4: RMS80 アーチファクト分解 ============
mc = df['mocap_yaw_true_deg'].values
e_cont = wrap(est['EKF1'] - mc)
e_corr = wrap(est['EKF1'] - truth)
dcheck = wrap(mc - truth)
cls_flip = np.abs(np.abs(dcheck) - 180.0) < 20
cls_glitch = (np.abs(dcheck) >= 20) & ~cls_flip
fig, axs = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
ax = axs[0]
ax.plot(t, mc, '.', ms=2, c='0.6', label='mocap_yaw_true(ロガー列=汚染)')
ax.plot(t, truth, 'k-', lw=1.0, label='復元真値')
ax.plot(t[cls_flip], mc[cls_flip], '.', ms=3, c='tab:red', label='誤180°флип補正 (376行)')
ax.plot(t[cls_glitch], mc[cls_glitch], '.', ms=3, c='tab:orange', label='+90°別解グリッチ')
ax.set_ylabel('ψ [deg]'); ax.legend(fontsize=8, ncol=2)
ax.set_title('「EKF1 vs mocap RMS 80.4°」の正体: ロガー列の90°別解グリッチ+誤180°補正', fontsize=10)
ax = axs[1]
ax.plot(t, e_cont, c='0.6', lw=0.7, label=f'EKF1−ロガー列 (飛行RMS {stats["ekf1_rms80_decomposition"]["vs_contaminated_flight_rms"]:.1f}°)')
ax.plot(t, e_corr, 'r-', lw=1.0, label=f'EKF1−復元真値 (RMS {stats["ekf1_rms80_decomposition"]["vs_corrected_flight_rms"]:.1f}°)')
ax.set_xlabel('t [s]'); ax.set_ylabel('誤差 [deg]')
ax.legend(fontsize=8)
fig.tight_layout()
fig.savefig(PL + '/04_rms80_artifact.png', dpi=140)
plt.close(fig)
print('plots done', fit_b0)
