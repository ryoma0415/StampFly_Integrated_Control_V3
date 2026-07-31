"""フロー較正(純回転 2×2 フィット)の回復精度・受入判定・適用往復・記録ゲート
(FLIGHT_ANALYSIS_20260731.md §5 / core/flowcal.py)。

合成データの作り方はファーム機上式の逆(flow_hub.cpp と同一規約):
真の K_true でセンサーカウント c = K_true·[q; −p] を作り、機体が適用中の
K_cur で復元したレート f = K_cur⁻¹·c から v_x=(f_x−q)·d / v_y=(f_y+p)·d
をテレメトリ値として与える。フィットは K_true を回復するはず。
"""

from __future__ import annotations

import json
import math
import random

import pytest

import stampfly_protocol as proto

from core.flowcal import (
    FLOWCAL_DEFAULT_SCALE, FLOWCAL_MIN_SAMPLES,
    flowcal_fit, flowcal_live_metrics, flowcal_matrix_fw_valid,
    flowcal_sample_valid,
)
from core.session import MODE_EXPERIMENT

from conftest import halt_supervisor
from fakes import FakeDroneResponder, wait_until

FLOW_OK_STATUS = (proto.TlmState.FLOW_STATUS_SENSOR_OK
                  | proto.TlmState.FLOW_STATUS_BURST_OK
                  | proto.TlmState.FLOW_STATUS_VEL_VALID
                  | proto.TlmState.FLOW_STATUS_RANGE_VALID
                  | proto.TlmState.FLOW_STATUS_SQUAL_OK)

K_CUR_DEFAULT = (FLOWCAL_DEFAULT_SCALE, 0.0, 0.0, FLOWCAL_DEFAULT_SCALE)


def diag_rot_matrix(kx: float, ky: float, phi_deg: float
                    ) -> tuple[float, float, float, float]:
    """K = diag(kx,ky)·R(φ0)(取付回転込みの真値行列)。"""
    phi = math.radians(phi_deg)
    c, s = math.cos(phi), math.sin(phi)
    return (kx * c, -kx * s, ky * s, ky * c)


def make_samples(k_true, k_cur=K_CUR_DEFAULT, *,
                 n=750, rate_hz=25.0, amp_q=0.6, amp_p=0.5,
                 d=0.3, squal=80, status=FLOW_OK_STATUS,
                 noise_rad_s=0.02, seed=1234):
    """純回転ロッキングの合成 TLM_STATE 行(dict)を作る。

    q/p は互いに非整数比の周波数の正弦(相関を避ける)。noise はセンサー
    レート換算 [rad/s] の白色ノイズ(7/31 実測 σ_v≈0.075 m/s / d=0.3m
    ≈ 0.25 rad/s は純回転収集ではもっと小さい — 既定 0.02 は保守的)。
    """
    rng = random.Random(seed)
    t00, t01, t10, t11 = k_true
    k00, k01, k10, k11 = k_cur
    det = k00 * k11 - k01 * k10
    i00, i01, i10, i11 = (k11 / det, -k01 / det, -k10 / det, k00 / det)
    samples = []
    for i in range(n):
        t = i / rate_hz
        q = amp_q * math.sin(2.0 * math.pi * 0.9 * t)
        p = amp_p * math.sin(2.0 * math.pi * 0.53 * t + 1.0)
        gx, gy = q, -p
        # 真のセンサーカウントレート+ノイズ(counts/s)
        cx = t00 * gx + t01 * gy + rng.gauss(0.0, noise_rad_s) * t00
        cy = t10 * gx + t11 * gy + rng.gauss(0.0, noise_rad_s) * t11
        # 機上処理: f = K_cur⁻¹·c → v = (f − [q; −p])·d
        fx = i00 * cx + i01 * cy
        fy = i10 * cx + i11 * cy
        samples.append({
            "q": q, "p": p,
            "vx": (fx - q) * d, "vy": (fy + p) * d,
            "d": d, "squal": squal, "status": status,
        })
    return samples


def assert_matrix_close(got, want, scale_tol=0.02, phi_tol_deg=1.0):
    """回復精度: 含意スケール ±2%・含意回転 ±1°(要件)。"""
    gkx = math.hypot(got[0], got[2])
    gky = math.hypot(got[1], got[3])
    wkx = math.hypot(want[0], want[2])
    wky = math.hypot(want[1], want[3])
    assert gkx == pytest.approx(wkx, rel=scale_tol)
    assert gky == pytest.approx(wky, rel=scale_tol)
    gphi = 0.5 * (math.degrees(math.atan2(-got[1], got[0]))
                  + math.degrees(math.atan2(got[2], got[3])))
    wphi = 0.5 * (math.degrees(math.atan2(-want[1], want[0]))
                  + math.degrees(math.atan2(want[2], want[3])))
    assert gphi == pytest.approx(wphi, abs=phi_tol_deg)


class TestFit:
    def test_recovers_diag_scale(self):
        """純スケール(回転なし)の回復。"""
        k_true = (500.0, 0.0, 0.0, 480.0)
        fit = flowcal_fit(make_samples(k_true), K_CUR_DEFAULT)
        assert fit["ok"], fit["warnings"]
        assert_matrix_close(fit["matrix"], k_true)
        assert fit["r2x"] >= 0.98 and fit["r2y"] >= 0.98

    def test_recovers_rotation(self):
        """7/31 実測相当(スケール 1.11/1.26 倍・φ0=−9.4°)の回復。"""
        k_true = diag_rot_matrix(450.0 * 1.11, 450.0 * 1.26, -9.4)
        fit = flowcal_fit(make_samples(k_true), K_CUR_DEFAULT)
        assert fit["ok"], fit["warnings"]
        assert_matrix_close(fit["matrix"], k_true)
        assert fit["phi0_deg"] == pytest.approx(-9.4, abs=1.0)

    def test_recovers_with_nondiag_k_cur(self):
        """K_cur が既に非対角(2 回目以降の較正)でも真値を回復する。"""
        k_cur = diag_rot_matrix(500.0, 470.0, -8.0)
        k_true = diag_rot_matrix(495.0, 465.0, -9.5)
        fit = flowcal_fit(make_samples(k_true, k_cur=k_cur), k_cur)
        assert fit["ok"], fit["warnings"]
        assert_matrix_close(fit["matrix"], k_true)

    def test_outlier_rejection(self):
        """並進スパイク(7/31 実測: 単発 0.4〜0.9 m/s)5% 注入でも回復する。"""
        k_true = diag_rot_matrix(500.0, 480.0, -9.4)
        samples = make_samples(k_true, n=1000)
        rng = random.Random(99)
        spiked = 0
        for s in samples:
            if rng.random() < 0.05:
                s["vx"] += rng.choice((-1.0, 1.0)) * rng.uniform(0.4, 0.9)
                s["vy"] += rng.choice((-1.0, 1.0)) * rng.uniform(0.4, 0.9)
                spiked += 1
        assert spiked > 20
        fit = flowcal_fit(samples, K_CUR_DEFAULT)
        assert fit["ok"], fit["warnings"]
        assert fit["n_rejected"] > 0
        assert_matrix_close(fit["matrix"], k_true)

    def test_recovers_with_tilt_compensated_altitude(self):
        """機上 d(生スラント Range)と TLM altitude_tof(チルト補正後の
        鉛直高度 ≈ Range·cosφ·cosθ。sensor.cpp tof_tilt_comp_enabled=true
        既定)の不一致を、記録行の roll/pitch からのスラント復元
        d/(cosφ·cosθ) で吸収する(±25° ロッキングで補正なしだと非対角に
        ≈8%・φ0 に ≈0.6° の系統誤差が乗る)。"""
        k_true = diag_rot_matrix(450.0 * 1.11, 450.0 * 1.26, -9.4)
        t00, t01, t10, t11 = k_true
        rng = random.Random(77)
        h = 0.30                       # 手持ちの鉛直高度(一定)
        amp = math.radians(25.0)       # ±25° ロッキング
        w1, w2 = 2.0 * math.pi * 0.9, 2.0 * math.pi * 1.3
        samples = []
        for i in range(1000):
            t = i / 25.0
            # 角度と角速度が整合する正弦(φ=A sin(w1 t) → p=Aw1 cos(w1 t))
            roll = amp * math.sin(w1 * t)
            p = amp * w1 * math.cos(w1 * t)
            pitch = amp * math.sin(w2 * t + 0.7)
            q = amp * w2 * math.cos(w2 * t + 0.7)
            cc = math.cos(roll) * math.cos(pitch)
            d_fw = h / cc              # 機上が使う生スラント距離
            gx, gy = q, -p
            cx = t00 * gx + t01 * gy + rng.gauss(0.0, 0.02) * t00
            cy = t10 * gx + t11 * gy + rng.gauss(0.0, 0.02) * t11
            fx = cx / FLOWCAL_DEFAULT_SCALE   # K_cur=diag(450,450) の逆
            fy = cy / FLOWCAL_DEFAULT_SCALE
            samples.append({
                "q": q, "p": p,
                "vx": (fx - q) * d_fw, "vy": (fy + p) * d_fw,
                "d": d_fw * cc,        # TLM はチルト補正後の鉛直高度
                "roll": roll, "pitch": pitch,
                "squal": 80, "status": FLOW_OK_STATUS,
            })
        fit = flowcal_fit(samples, K_CUR_DEFAULT)
        assert fit["ok"], fit["warnings"]
        # 復元により行列成分レベルで一致する(補正なしだと m01/m10 が
        # +6〜7 counts/rad ずれ、この許容を超える)
        for got, want in zip(fit["matrix"], k_true):
            assert got == pytest.approx(want, abs=max(abs(want) * 0.015, 4.0))
        assert fit["phi0_deg"] == pytest.approx(-9.4, abs=0.3)

    def test_invalid_rows_excluded(self):
        """vel_valid/range_valid/squal_ok が欠ける行はフィットに使わない。"""
        k_true = (500.0, 0.0, 0.0, 480.0)
        samples = make_samples(k_true, n=600)
        # 後半 300 行を無効化+速度をゼロ化(ファームは無効時 0 を送る)
        for s in samples[300:]:
            s["status"] = (proto.TlmState.FLOW_STATUS_SENSOR_OK
                           | proto.TlmState.FLOW_STATUS_BURST_OK)
            s["vx"] = s["vy"] = 0.0
        assert not flowcal_sample_valid(samples[-1])
        fit = flowcal_fit(samples, K_CUR_DEFAULT)
        assert fit["n_valid"] == 300
        assert fit["valid_ratio"] == pytest.approx(0.5)
        assert fit["ok"], fit["warnings"]
        assert_matrix_close(fit["matrix"], k_true)


class TestAcceptance:
    def test_too_few_samples(self):
        fit = flowcal_fit(make_samples((450.0, 0.0, 0.0, 450.0), n=100),
                          K_CUR_DEFAULT)
        assert fit["ok"] is False
        assert any("サンプル不足" in w for w in fit["warnings"])

    def test_insufficient_excitation(self):
        """揺すり不足(std < 0.30 rad/s)は不合格。"""
        fit = flowcal_fit(
            make_samples((450.0, 0.0, 0.0, 450.0), amp_q=0.1, amp_p=0.1,
                         noise_rad_s=0.005),
            K_CUR_DEFAULT)
        assert fit["ok"] is False
        assert any("揺すり不足" in w for w in fit["warnings"])

    def test_scale_out_of_pass_range(self):
        """含意スケールが 300..600 の外なら不合格(ファーム受理内でも)。"""
        fit = flowcal_fit(make_samples((700.0, 0.0, 0.0, 700.0)),
                          K_CUR_DEFAULT)
        assert fit["ok"] is False
        assert any("合格範囲外" in w for w in fit["warnings"])

    def test_scale_ratio_exceeded(self):
        """x/y 比 >1.2 は不合格。"""
        fit = flowcal_fit(make_samples((580.0, 0.0, 0.0, 420.0)),
                          K_CUR_DEFAULT)
        assert fit["ok"] is False
        assert any("乖離" in w for w in fit["warnings"])

    def test_rotation_exceeded(self):
        """|φ0| > 15° は不合格。"""
        fit = flowcal_fit(
            make_samples(diag_rot_matrix(450.0, 450.0, 20.0)),
            K_CUR_DEFAULT)
        assert fit["ok"] is False
        assert any("φ0" in w for w in fit["warnings"])

    def test_noisy_data_fails_r2(self):
        """相関が壊れるほどのノイズ(並進混入相当)は r² で不合格。"""
        fit = flowcal_fit(
            make_samples((450.0, 0.0, 0.0, 450.0), noise_rad_s=1.0),
            K_CUR_DEFAULT)
        assert fit["ok"] is False

    def test_fw_valid_bounds(self):
        assert flowcal_matrix_fw_valid(450.0, 36.0, -33.0, 447.0)
        assert not flowcal_matrix_fw_valid(50.0, 0.0, 0.0, 450.0)
        assert not flowcal_matrix_fw_valid(450.0, 2500.0, 0.0, 450.0)
        # det < 10⁴(K⁻¹ 不安定)は拒否
        assert not flowcal_matrix_fw_valid(450.0, 500.0, 500.0, 450.0)
        assert not flowcal_matrix_fw_valid(120.0, 110.0, 110.0, 120.0)
        assert not flowcal_matrix_fw_valid(float("nan"), 0.0, 0.0, 450.0)

    def test_live_metrics(self):
        samples = make_samples((450.0, 0.0, 0.0, 450.0), n=250)
        metrics = flowcal_live_metrics(samples)
        assert metrics["n_total"] == 250
        assert metrics["n_valid"] == 250
        assert metrics["valid_ratio"] == pytest.approx(1.0)
        assert metrics["std_q_rad_s"] > 0.3
        assert metrics["std_p_rad_s"] > 0.3
        assert metrics["squal"] == 80
        assert metrics["tof_m"] == pytest.approx(0.3)
        assert flowcal_live_metrics([]) is None


# ---------------------------------------------------------------------------
# マネージャ(記録ゲート+適用往復。FakeDroneResponder で CAL_GET 照合込み)
# ---------------------------------------------------------------------------

def make_flow_tlm(q=0.0, p=0.0, vx=0.0, vy=0.0, d=0.3,
                  squal=80, status=FLOW_OK_STATUS, flags=0) -> proto.TlmState:
    return proto.TlmState(state=2, flags=flags, q=q, p=p,
                          altitude_tof=d, flow_vx_mps=vx, flow_vy_mps=vy,
                          flow_squal=squal, flow_status=status)


@pytest.fixture
def flowcal_session(server_config, session_factory, tmp_path):
    server_config["failsafe"]["command_ack_timeout_s"] = 0.1
    responder = FakeDroneResponder()
    session, transport, clock = session_factory(responder=responder)
    session.connect("FAKE")
    halt_supervisor(session)
    assert session.set_mode(MODE_EXPERIMENT)
    mgr = session.flowcal
    mgr.state_path = tmp_path / "flowcal_state.json"
    # TLM_STATE を 1 フレーム届けておく(記録開始ゲートの鮮度要件)
    transport.push(proto.MsgType.TLM_STATE, make_flow_tlm().to_payload())
    assert wait_until(lambda: session._tlm_state_snapshot()[0] is not None)
    return session, transport, clock, responder, mgr


def feed_synthetic(mgr, k_true, k_cur, **kwargs):
    """合成サンプルを on_tlm_state 経由で供給する(RX スレッド相当)。"""
    for s in make_samples(k_true, k_cur=k_cur, **kwargs):
        mgr.on_tlm_state(make_flow_tlm(q=s["q"], p=s["p"],
                                       vx=s["vx"], vy=s["vy"], d=s["d"],
                                       squal=s["squal"], status=s["status"]))


class TestRecordGate:
    def test_rejects_while_motor_running(self, flowcal_session):
        session, transport, clock, responder, mgr = flowcal_session
        with session.experiment.lock:
            session.experiment.motor_running = True
        result = mgr.start_record()
        assert result["ok"] is False
        assert "モーター回転中" in result["message"]
        with session.experiment.lock:
            session.experiment.motor_running = False

    def test_rejects_while_flying(self, flowcal_session):
        session, transport, clock, responder, mgr = flowcal_session
        transport.push(proto.MsgType.TLM_STATE,
                       make_flow_tlm(flags=proto.TlmState.FLAG_FLYING,
                                     ).to_payload())
        assert wait_until(
            lambda: (session._tlm_state_snapshot()[0] or proto.TlmState()
                     ).flags & proto.TlmState.FLAG_FLYING)
        result = mgr.start_record()
        assert result["ok"] is False
        assert "飛行中" in result["message"]

    def test_rejects_double_start(self, flowcal_session):
        session, transport, clock, responder, mgr = flowcal_session
        assert mgr.start_record()["ok"], mgr.status()["message"]
        result = mgr.start_record()
        assert result["ok"] is False
        assert "既に記録中" in result["message"]
        mgr.clear()

    def test_start_snapshots_drone_matrix(self, flowcal_session):
        """記録開始時に CAL_GET で機体の現在行列を固定する。"""
        session, transport, clock, responder, mgr = flowcal_session
        responder.cal_data.valid_flags |= proto.TlmCalData.VALID_FLOWCAL
        responder.cal_data.flowcal_m00 = 470.0
        responder.cal_data.flowcal_m11 = 460.0
        responder.cal_data.flowcal_m01 = 10.0
        responder.cal_data.flowcal_m10 = -12.0
        result = mgr.start_record()
        assert result["ok"], result["message"]
        assert result["k_cur"]["source"] == "drone"
        assert result["k_cur"]["matrix"] == \
            pytest.approx([470.0, 10.0, -12.0, 460.0])
        # 機体側 flowcal 未設定なら既定 diag(450,450)
        mgr.clear()
        responder.cal_data.valid_flags &= ~proto.TlmCalData.VALID_FLOWCAL
        result = mgr.start_record()
        assert result["ok"], result["message"]
        assert result["k_cur"]["source"] == "default"
        assert result["k_cur"]["matrix"] == \
            pytest.approx([450.0, 0.0, 0.0, 450.0])
        mgr.clear()


class TestManagerRoundtrip:
    def test_record_fit_apply_verify(self, flowcal_session):
        session, transport, clock, responder, mgr = flowcal_session
        k_true = diag_rot_matrix(500.0, 480.0, -9.4)
        assert mgr.start_record()["ok"]
        feed_synthetic(mgr, k_true, K_CUR_DEFAULT)
        status = mgr.status()
        assert status["collecting"] is True
        assert status["sample_count"] == 750
        assert status["live"]["std_q_rad_s"] > 0.3

        result = mgr.stop_and_fit()
        assert result["ok"], result["fit"]
        fit = result["fit"]
        assert fit["ok"], fit["warnings"]
        assert_matrix_close(fit["matrix"], k_true)

        transport.clear_sent()
        result = mgr.apply()
        assert result["ok"], result["message"]
        assert result["verified"] is True
        # CMD_FLOWCAL_SET(17B 行優先)が送られ、機体状態へ反映されている
        frames = transport.frames_of_type(proto.MsgType.CMD_FLOWCAL_SET)
        assert len(frames) == 1
        msg = proto.CmdFlowcalSet.from_payload(frames[0].payload)
        assert msg.mode == proto.CmdFlowcalSet.MODE_SET
        assert (msg.m00, msg.m01, msg.m10, msg.m11) == \
            pytest.approx(tuple(fit["matrix"]), abs=1e-3)
        assert responder.cal_data.valid_flags & proto.TlmCalData.VALID_FLOWCAL
        assert responder.cal_data.flowcal_m00 == \
            pytest.approx(fit["matrix"][0], abs=1e-3)
        # flowcal_state.json に適用状態が残る
        state = json.loads(mgr.state_path.read_text(encoding="utf-8"))
        assert state["applied"]["verified"] is True
        assert state["applied"]["matrix"] == \
            pytest.approx(fit["matrix"], abs=1e-3)
        assert state["applied"]["forced"] is False
        assert mgr.status()["applied"]["verified"] is True

    def test_apply_requires_pass_unless_force(self, flowcal_session):
        """不合格フィット(スケール範囲外)は force のみ適用可。"""
        session, transport, clock, responder, mgr = flowcal_session
        k_true = (650.0, 0.0, 0.0, 650.0)   # 合格範囲(300..600)外・ファーム受理内
        assert mgr.start_record()["ok"]
        feed_synthetic(mgr, k_true, K_CUR_DEFAULT)
        result = mgr.stop_and_fit()
        assert result["ok"] is False
        result = mgr.apply()
        assert result["ok"] is False
        assert result.get("quality_warning") is True
        assert result["warnings"]
        result = mgr.apply(force=True)
        assert result["ok"], result["message"]
        assert responder.cal_data.flowcal_m00 == pytest.approx(650.0, rel=0.02)
        state = json.loads(mgr.state_path.read_text(encoding="utf-8"))
        assert state["applied"]["forced"] is True

    def test_apply_never_sends_fw_invalid(self, flowcal_session):
        """ファーム受理範囲外の行列は force でも送信しない。"""
        session, transport, clock, responder, mgr = flowcal_session
        with mgr._lock:
            mgr._fit = {"ok": False, "matrix": [50.0, 0.0, 0.0, 450.0],
                        "warnings": ["テスト"]}
        transport.clear_sent()
        result = mgr.apply(force=True)
        assert result["ok"] is False
        assert "受理範囲外" in result["message"]
        assert transport.frames_of_type(proto.MsgType.CMD_FLOWCAL_SET) == []

    def test_apply_rejected_ack_fails(self, flowcal_session):
        """機体 NACK(飛行中 bad_state 等)で適用失敗になる。"""
        session, transport, clock, responder, mgr = flowcal_session
        assert mgr.start_record()["ok"]
        feed_synthetic(mgr, diag_rot_matrix(500.0, 480.0, -9.4),
                       K_CUR_DEFAULT)
        assert mgr.stop_and_fit()["ok"]
        responder.ack_status_overrides[int(proto.MsgType.CMD_FLOWCAL_SET)] = \
            proto.TlmAck.STATUS_BAD_STATE
        result = mgr.apply()
        assert result["ok"] is False
        assert not (responder.cal_data.valid_flags
                    & proto.TlmCalData.VALID_FLOWCAL)

    def test_apply_without_fit_fails(self, flowcal_session):
        session, transport, clock, responder, mgr = flowcal_session
        result = mgr.apply()
        assert result["ok"] is False
        assert "フィット結果" in result["message"]

    def test_clear_resets_pc_state_only(self, flowcal_session):
        session, transport, clock, responder, mgr = flowcal_session
        assert mgr.start_record()["ok"]
        feed_synthetic(mgr, (500.0, 0.0, 0.0, 480.0), K_CUR_DEFAULT, n=250)
        assert mgr.stop_and_fit()["ok"]
        transport.clear_sent()
        result = mgr.clear()
        assert result["ok"]
        status = mgr.status()
        assert status["sample_count"] == 0
        assert status["fit"] is None
        # 機体 NVS には何も送らない(機体側クリアは /api/flow cal_clear)
        assert transport.frames_of_type(proto.MsgType.CMD_FLOWCAL_SET) == []

    def test_session_feeds_recorder_from_wire(self, flowcal_session):
        """ワイヤ経由の TLM_STATE が記録バッファへ入る(session 配線)。"""
        session, transport, clock, responder, mgr = flowcal_session
        assert mgr.start_record()["ok"]
        before = mgr.status()["sample_count"]
        transport.push(proto.MsgType.TLM_STATE,
                       make_flow_tlm(q=0.2, vx=0.05).to_payload())
        assert wait_until(
            lambda: mgr.status()["sample_count"] == before + 1)
        mgr.clear()


class TestApplyUnverified:
    """P2a: ACK 成功後の読み戻し失敗でも適用記録を必ず残す(applied_unverified)。

    旧実装は照合失敗で記録なしのまま return し、機体 NVS だけが書き換わって
    flowcal_state.json と乖離した。改修後は verified=False + verify_error で
    送信行列・時刻を必ず記録し、status() の verify_warning が不一致を警告する。
    """

    def _fit_ok(self, mgr):
        assert mgr.start_record()["ok"]
        feed_synthetic(mgr, diag_rot_matrix(500.0, 480.0, -9.4), K_CUR_DEFAULT)
        result = mgr.stop_and_fit()
        assert result["ok"], result["fit"]
        return result["fit"]

    def test_readback_timeout_records_applied_unverified(self, flowcal_session):
        session, transport, clock, responder, mgr = flowcal_session
        fit = self._fit_ok(mgr)
        session.calibration._cal_get_timeout_s = 0.1   # 待ち時間の短縮
        responder.silent_types.add(int(proto.MsgType.CMD_CAL_GET))

        result = mgr.apply()
        assert result["ok"] is False
        assert result["verified"] is False
        assert result.get("applied_unverified") is True
        # 機体 NVS は書き換わっている(ACK 成功 = 機体側受理)
        assert responder.cal_data.valid_flags & proto.TlmCalData.VALID_FLOWCAL
        # 状態ファイルに送信行列+時刻+verify_error が残る
        state = json.loads(mgr.state_path.read_text(encoding="utf-8"))
        applied = state["applied"]
        assert applied["verified"] is False
        assert applied["matrix"] == pytest.approx(fit["matrix"], abs=1e-3)
        assert applied["applied_at"] > 0
        assert "CAL_GET" in applied["verify_error"]
        # status が未照合を警告する
        status = mgr.status()
        assert status["applied"]["verified"] is False
        assert status["verify_warning"]

    def test_readback_mismatch_records_applied_unverified(self,
                                                          flowcal_session):
        session, transport, clock, responder, mgr = flowcal_session
        self._fit_ok(mgr)
        orig = responder._update_cal_state

        def corrupt(frame):
            orig(frame)
            if frame.type == proto.MsgType.CMD_FLOWCAL_SET:
                responder.cal_data.flowcal_m00 += 25.0   # NVS 書込み化けの模擬

        responder._update_cal_state = corrupt
        result = mgr.apply()
        assert result["ok"] is False
        assert result.get("applied_unverified") is True
        state = json.loads(mgr.state_path.read_text(encoding="utf-8"))
        assert state["applied"]["verified"] is False
        assert "m00" in state["applied"]["verify_error"]

    def test_ack_failure_still_not_recorded(self, flowcal_session):
        """ACK 失敗(NACK)は機体未受理 — 従来どおり記録しない。"""
        session, transport, clock, responder, mgr = flowcal_session
        self._fit_ok(mgr)
        responder.ack_status_overrides[int(proto.MsgType.CMD_FLOWCAL_SET)] = \
            proto.TlmAck.STATUS_BAD_STATE
        result = mgr.apply()
        assert result["ok"] is False
        assert not mgr.state_path.is_file()

    def test_status_warns_on_drone_mismatch(self, flowcal_session):
        """照合成功後でも、機体行列が変わったら status が警告する。"""
        session, transport, clock, responder, mgr = flowcal_session
        self._fit_ok(mgr)
        assert mgr.apply()["ok"]
        assert mgr.status()["verify_warning"] is None
        # 機体側で行列が変わった(別経路の書換え等)→ TLM_CAL_DATA 受信
        responder.cal_data.flowcal_m00 += 30.0
        target = responder.cal_data.flowcal_m00
        transport.push(proto.MsgType.TLM_CAL_DATA,
                       responder.cal_data.to_payload())
        assert wait_until(
            lambda: (lambda cal: cal is not None
                     and abs(cal.flowcal_m00 - target) < 1e-3)(
                         session.calibration.peek_cal_data()[0]))
        warning = mgr.status()["verify_warning"]
        assert warning and "一致しません" in warning

    def test_unverified_clears_after_matching_readback(self, flowcal_session):
        """未照合でも、後追いの TLM_CAL_DATA が4成分一致すれば警告は消える。"""
        session, transport, clock, responder, mgr = flowcal_session
        self._fit_ok(mgr)
        session.calibration._cal_get_timeout_s = 0.1
        responder.silent_types.add(int(proto.MsgType.CMD_CAL_GET))
        assert mgr.apply()["ok"] is False
        assert mgr.status()["verify_warning"]
        responder.silent_types.discard(int(proto.MsgType.CMD_CAL_GET))
        transport.push(proto.MsgType.TLM_CAL_DATA,
                       responder.cal_data.to_payload())
        assert wait_until(lambda: (mgr.status()["drone"] or {}).get("valid"))
        assert mgr.status()["verify_warning"] is None
