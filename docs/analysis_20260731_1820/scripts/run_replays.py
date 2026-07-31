#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""20260731_182025 EKF2リプレイ5ケース: デッドロック再現 / 正解基準 / 初回整列改造."""
import json
import math
import sys
import numpy as np

SP = '/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad/a0731b'
REPO = '/Users/ryoma_nishimura/Code-Projects/StampFly-Project-Develop/Developments/StampFly_MoCap_System_v2/StampFly_Integrated_Control_V3'
sys.path.insert(0, SP)
from ekf2_replay_mod import load_flight_log, run_replay, wrap_pi  # noqa: E402

FRAME_OFF_DEG = 90.434  # sent = truth - 90.434 (truth_fit.json)

data = load_flight_log(__import__('pathlib').Path(SP + '/log_fixed.csv'))
ff_profile = json.loads(open(
    REPO + '/pc_server/data/ff_profiles/研究室_DroneX_20260713.json',
    encoding='utf-8').read())

wrapd = lambda a: (np.asarray(a) + 180.0) % 360.0 - 180.0

cases = {
    'R0_noyaw':      dict(yaw_obs_mode='none'),
    'R1_deadlock':   dict(yaw_obs_mode='yaw_ref_sent_rad', align_yaw_obs=False),
    'R2_correct':    dict(yaw_obs_mode='mocap', align_yaw_obs=False),
    'R3a_fix_asis':  dict(yaw_obs_mode='yaw_ref_sent_rad', align_yaw_obs=False,
                          initial_align=True),
    'R3b_fix_clean': dict(yaw_obs_mode='yaw_obs_sent_clean_rad',
                          align_yaw_obs=False, initial_align=True),
}

out = {}
results = {}
for name, kw in cases.items():
    res = run_replay(data, ff_profile=ff_profile, **kw)
    results[name] = res
    fr = res['frames']
    t = res['t_s']
    state = data['tlm_state'][fr]
    fly = state == 4
    truth_deg = np.degrees(res['mocap_true_rad'])  # log_fixed: mocap col = 正解truth
    psi_deg = np.degrees(res['psi_ekf2_rad'])
    # 評価フレーム系: 送信系ケースは truth-90.434 が基準
    sent_frame = 'sent' in kw.get('yaw_obs_mode', '')
    ref = truth_deg - FRAME_OFF_DEG if sent_frame else truth_deg
    err = wrapd(psi_deg - ref)
    m = fly & np.isfinite(err)
    off = math.degrees(math.atan2(np.mean(np.sin(np.radians(err[m]))),
                                  np.mean(np.cos(np.radians(err[m])))))
    err_sh = wrapd(err - off)
    obs = np.isfinite(res['obs_accept'])
    acc = res['obs_accept'] == 1.0
    stopped = res['obs_stopped'] == 1.0
    innov = np.degrees(res['yaw_innov_rad'])
    st = np.nan_to_num(res['status'], nan=0.0).astype(int)
    anchor_fired = (st & 0x04) != 0
    stats = dict(
        mode=res['mode'],
        n_frames=int(len(fr)), n_obs=int(obs.sum()),
        fused_pct=float(100.0 * acc.sum() / max(obs.sum(), 1)),
        latch_t=float(t[np.argmax(stopped)]) if stopped.any() else None,
        latch_release_ts=[],
        align_innov_deg=(math.degrees(res['align_innov_rad'])
                         if res['align_innov_rad'] is not None else None),
        err_vs_ref=dict(frame='truth-90.434' if sent_frame else 'truth',
                        raw_rms=float(np.sqrt(np.mean(err[m]**2))),
                        raw_mean=float(err[m].mean()),
                        const_off_deg=float(off),
                        shape_rms=float(np.sqrt(np.mean(err_sh[m]**2))),
                        shape_max=float(np.abs(err_sh[m]).max())),
        vs_logged_ekf2=dict(
            raw_rms=float(np.sqrt(np.mean(wrapd(
                psi_deg - np.degrees(res['psi_ekf2_log_rad']))[m]**2)))),
        innov=dict(
            n_finite=int(np.isfinite(innov).sum()),
            rms_when_accepted=float(np.sqrt(np.mean(innov[acc]**2))) if acc.any() else None,
            max_when_accepted=float(np.abs(innov[acc]).max()) if acc.any() else None),
        bm_final=[float(res['bm_x_ut'][np.isfinite(res['bm_x_ut'])][-1]),
                  float(res['bm_y_ut'][np.isfinite(res['bm_y_ut'])][-1])],
        gate_reject_ratio=res['stats']['gate_reject_ratio'],
        flight_anchor_t=float(t[np.argmax(anchor_fired)]) if anchor_fired.any() else None,
        warnings=res['warnings'],
    )
    # 停止→(改造ではreseedで解除は初回のみ) 停止区間
    if stopped.any():
        d = np.diff(stopped.astype(int))
        st_on = np.where(d == 1)[0] + 1
        st_off = np.where(d == -1)[0] + 1
        stats['stopped_spans'] = [
            [round(float(t[a]), 2),
             round(float(t[b]), 2) if len(st_off) else None]
            for a, b in zip(st_on, list(st_off) + [len(t)-1]*len(st_on))][:6]
        stats['stopped_frac_of_obs'] = float(stopped.sum() / max(obs.sum(), 1))
    out[name] = stats
    print(name, json.dumps(stats, ensure_ascii=False)[:600])

# ---- Code Identity 照合 (R0 vs 実機EKF2ログ) ----
r0 = results['R0_noyaw']
fr = r0['frames']
t = r0['t_s']
psi_r = np.degrees(r0['psi_ekf2_rad'])
psi_l = np.degrees(r0['psi_ekf2_log_rad'])
d = wrapd(psi_r - psi_l)
m = np.isfinite(d)
off = d[m & (t < 1.0)].mean() if (m & (t < 1.0)).any() else 0.0
out['code_identity_R0'] = dict(
    raw_rms=float(np.sqrt(np.mean(d[m]**2))),
    raw_max=float(np.abs(d[m]).max()),
    init_off=float(off),
    offrm_rms=float(np.sqrt(np.mean(wrapd(d[m]-off)**2))),
    bm_final_replay=out['R0_noyaw']['bm_final'],
    bm_final_logged=[float(data['tlm_ekf2_bm_x_ut'][fr][-1]),
                     float(data['tlm_ekf2_bm_y_ut'][fr][-1])],
    # ゲートビット一致率 (mag観測フレームのみ)
    gate_match_pct=float(100.0 * np.mean(
        np.nan_to_num(r0['gate'], nan=-1.0).astype(int)[np.isfinite(r0['nis'])] ==
        np.nan_to_num(data['tlm_ekf2_gate'][fr], nan=-2.0).astype(int)[np.isfinite(r0['nis'])])),
)
# ΔB̂再構成の一致 (FFパスのCode Identity)
dbr = r0['db_hat_x_ut']
mdb = np.isfinite(dbr)
dbl = data['tlm_db_hat_x_ut'][fr]
out['code_identity_R0']['db_hat_x_match_rms_ut'] = float(
    np.sqrt(np.nanmean((dbr[mdb] - dbl[mdb])**2)))
dbry = r0['db_hat_y_ut']
dbly = data['tlm_db_hat_y_ut'][fr]
out['code_identity_R0']['db_hat_y_match_rms_ut'] = float(
    np.sqrt(np.nanmean((dbry[mdb] - dbly[mdb])**2)))
out['code_identity_R0']['db_hat_scale_ratio_x'] = float(
    np.nanmedian(dbl[mdb] / np.where(np.abs(dbr[mdb]) > 1, dbr[mdb], np.nan)))

# R1 が実機と同一挙動か (デッドロック再現)
r1 = out['R1_deadlock']
out['deadlock_reproduction'] = dict(
    replay_fused_pct=r1['fused_pct'], flight_fused_pct=0.0,
    replay_latch_t=r1['latch_t'], flight_latch_t=1.031,
    innov_first=float(np.degrees(results['R1_deadlock']['yaw_innov_rad'][
        np.isfinite(results['R1_deadlock']['yaw_innov_rad'])][0])))

# 保存
np.savez(SP + '/replay_series.npz',
         **{f'{n}_{k}': results[n][k] for n in results
            for k in ('t_s', 'psi_ekf2_rad', 'bm_x_ut', 'bm_y_ut',
                      'yaw_innov_rad', 'obs_accept', 'obs_stopped', 'status')},
         frames=results['R0_noyaw']['frames'],
         truth_deg=np.degrees(results['R0_noyaw']['mocap_true_rad']),
         psi_ekf1_log=np.degrees(results['R0_noyaw']['psi_ekf1_log_rad']),
         psi_ekf2_log=np.degrees(results['R0_noyaw']['psi_ekf2_log_rad']),
         state=data['tlm_state'][results['R0_noyaw']['frames']])
with open(SP + '/replay_stats.json', 'w') as f:
    json.dump(out, f, indent=1, ensure_ascii=False)
print(json.dumps(out, indent=1, ensure_ascii=False)[:4000])
