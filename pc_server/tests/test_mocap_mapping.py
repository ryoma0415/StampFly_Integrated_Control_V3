"""MoCap マッピング(AttitudeMapper / validate_mapping / 実行時変更 API)。

2026-07-27 の NatNet_RigidBody_Streamer 実測で確定した事実を契約化する:
- Motive は Y-up → Z-up 前提オイラーの yaw_rad は機首方位ではない
- 機首方位は前方軸ベクトルの制御座標方位(heading)で、位置マッピングと
  同じ軸マップから自動導出される
- マーカー対称性によるソルバの180°別解(機首軸まわり反転)が発生する
  → 上方軸ガード+ヨー継続性ガードで補正する
"""

from __future__ import annotations

import json
import math

import pytest

from core import config as cfg
from core.logger import COLUMNS, META_PASSTHROUGH_KEYS
from core.mocap import (
    DEFAULT_ATTITUDE_TRANSFORM,
    AttitudeMapper,
    CoordinateTransformer,
    MocapSource,
    validate_mapping,
)

DEG = math.pi / 180.0


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_about_y(theta):
    """Motive Y 軸(鉛直)まわり右ねじ theta の回転クォータニオン。"""
    return (0.0, math.sin(theta / 2.0), 0.0, math.cos(theta / 2.0))


FLIP_ABOUT_BODY_X = (1.0, 0.0, 0.0, 0.0)   # 機首軸まわり180°(ソルバ別解)


@pytest.fixture
def transformer() -> CoordinateTransformer:
    return CoordinateTransformer(None)   # 既定 x←z, y←−x, z←y


@pytest.fixture
def mapper() -> AttitudeMapper:
    return AttitudeMapper(None)          # 既定 +x 前方 / +y 上方


# ----------------------------------------------------------------------
# AttitudeMapper: ヨー導出
# ----------------------------------------------------------------------

def test_default_heading_matches_mirror_frame_identity(transformer, mapper):
    """既定マッピングの既知関係: heading = −θ − 90°(鏡映フレーム)。

    2026-07-27 実測(rb2.csv)で全行 0.0000° 一致を確認した関係式。
    """
    state = mapper.new_state()
    for theta_deg in (0.0, 45.0, 90.0, 135.0):
        q = quat_about_y(theta_deg * DEG)
        heading, _, _ = mapper.derive(q, transformer, state, 0.0)
        expected = -theta_deg - 90.0
        expected = (expected + 180.0) % 360.0 - 180.0
        assert math.degrees(heading) == pytest.approx(expected, abs=1e-6)


def test_yaw_sign_and_offset_recover_user_convention(transformer):
    """sign=+1, offset=+90° で「物理ヨー = yaw_true」に合わせられる。

    実測時のユーザ規約(0→90→180→−90)は heading + 90° に一致した。
    """
    mapper = AttitudeMapper({"yaw_sign": 1, "yaw_offset_deg": 90.0})
    state = mapper.new_state()
    # 物理ヨー +60°(ユーザ規約 = Motive Y まわり −60°: 実測と同じ逆符号)
    q = quat_about_y(-60.0 * DEG)
    _, yaw_true, _ = mapper.derive(q, transformer, state, 0.0)
    assert math.degrees(yaw_true) == pytest.approx(60.0, abs=1e-6)


def test_yaw_true_wraps_into_pi_range(transformer):
    mapper = AttitudeMapper({"yaw_offset_deg": 350.0})
    state = mapper.new_state()
    _, yaw_true, _ = mapper.derive((0.0, 0.0, 0.0, 1.0), transformer, state, 0.0)
    assert -math.pi < yaw_true <= math.pi


def test_forward_axis_selection(transformer):
    """前方軸を +z にすると、+z 軸の方位が heading になる。"""
    mapper = AttitudeMapper({"forward_axis": "+z", "up_axis": "+y"})
    state = mapper.new_state()
    # 恒等姿勢: ボディ+Z = Motive +Z → 制御系では x←z なので +x 方向
    heading, _, _ = mapper.derive((0.0, 0.0, 0.0, 1.0), transformer, state, 0.0)
    assert math.degrees(heading) == pytest.approx(0.0, abs=1e-6)


# ----------------------------------------------------------------------
# AttitudeMapper: フリップ補正
# ----------------------------------------------------------------------

def test_up_axis_guard_flags_inverted_solve(transformer, mapper):
    """機首軸まわり180°別解: heading 不変+FLIP_UP_INVERTED が立つ。"""
    state = mapper.new_state()
    q = quat_about_y(30.0 * DEG)
    h1, y1, f1 = mapper.derive(q, transformer, state, 0.00)
    assert f1 == 0
    q_flip = quat_mul(q, FLIP_ABOUT_BODY_X)
    h2, y2, f2 = mapper.derive(q_flip, transformer, state, 0.01)
    assert f2 & AttitudeMapper.FLIP_UP_INVERTED
    assert math.degrees(h2) == pytest.approx(math.degrees(h1), abs=1e-6)
    assert math.degrees(y2) == pytest.approx(math.degrees(y1), abs=1e-6)
    # 正立解に戻れば解除される
    _, _, f3 = mapper.derive(q, transformer, state, 0.02)
    assert f3 == 0


def test_up_axis_guard_holds_state_near_gate(transformer, mapper):
    """上方軸の鉛直成分が小さい(±90°姿勢)間は反転判定を保留する。"""
    state = mapper.new_state()
    mapper.derive(quat_about_y(0.0), transformer, state, 0.00)
    assert state["up_flipped"] is False
    # ボディ X まわり +90°(上方軸が水平になる): 判定保留 = False のまま
    q_roll90 = (math.sin(math.pi / 4.0), 0.0, 0.0, math.cos(math.pi / 4.0))
    _, _, flags = mapper.derive(q_roll90, transformer, state, 0.01)
    assert state["up_flipped"] is False
    assert not (flags & AttitudeMapper.FLIP_UP_INVERTED)


def test_yaw_continuity_guard_corrects_180_jump(transformer):
    """鉛直軸まわりの180°別解(上方軸では検出不能)を継続性で補正する。

    フレーム間で +180° 跳んだヨーは補正されて連続に保たれ、
    反転が解けたフレームで補正も自動解除される。
    """
    mapper = AttitudeMapper(None)
    state = mapper.new_state()
    q = quat_about_y(10.0 * DEG)
    _, y1, _ = mapper.derive(q, transformer, state, 0.00)
    # 鉛直軸(Motive Y)まわり180°の別解
    q_vflip = quat_mul(quat_about_y(math.pi), q)
    _, y2, f2 = mapper.derive(q_vflip, transformer, state, 0.01)
    assert f2 & AttitudeMapper.FLIP_YAW_JUMP
    assert math.degrees(y2) == pytest.approx(math.degrees(y1), abs=1e-6)
    # 別解が解けたフレーム: 再び180°跳ぶ → トグル解除で連続のまま
    _, y3, f3 = mapper.derive(q, transformer, state, 0.02)
    assert not (f3 & AttitudeMapper.FLIP_YAW_JUMP)
    assert math.degrees(y3) == pytest.approx(math.degrees(y1), abs=1e-6)


def test_yaw_continuity_guard_resets_after_gap(transformer):
    """受信ギャップ(>0.5s)を跨いだら継続性を主張しない(補正破棄)。"""
    mapper = AttitudeMapper(None)
    state = mapper.new_state()
    q = quat_about_y(10.0 * DEG)
    mapper.derive(q, transformer, state, 0.00)
    q_vflip = quat_mul(quat_about_y(math.pi), q)
    _, _, f2 = mapper.derive(q_vflip, transformer, state, 0.01)
    assert f2 & AttitudeMapper.FLIP_YAW_JUMP
    # 1.0s のギャップ後: 蓄積補正は破棄され、観測をそのまま信じる
    _, y3, f3 = mapper.derive(q_vflip, transformer, state, 1.01)
    assert not (f3 & AttitudeMapper.FLIP_YAW_JUMP)
    _, y_raw, _ = AttitudeMapper(None).derive(
        q_vflip, transformer, AttitudeMapper.new_state(), 0.0)
    assert math.degrees(y3) == pytest.approx(math.degrees(y_raw), abs=1e-6)


def test_flip_correction_disabled(transformer):
    mapper = AttitudeMapper({"flip_correction": False})
    state = mapper.new_state()
    q = quat_about_y(10.0 * DEG)
    mapper.derive(q, transformer, state, 0.00)
    q_vflip = quat_mul(quat_about_y(math.pi), q)
    _, y2, f2 = mapper.derive(q_vflip, transformer, state, 0.01)
    assert f2 == 0   # 補正なし: 跳んだまま(生の観測)
    assert state["last_yaw"] is None


# ----------------------------------------------------------------------
# validate_mapping
# ----------------------------------------------------------------------

VALID_TRANSFORM = {
    "x": {"axis": "z", "sign": 1},
    "y": {"axis": "x", "sign": -1},
    "z": {"axis": "y", "sign": 1},
}


def test_validate_accepts_current_config():
    assert validate_mapping(VALID_TRANSFORM, DEFAULT_ATTITUDE_TRANSFORM) is None


@pytest.mark.parametrize("mutate,phrase", [
    (lambda t: t["y"].update(axis="z"), "縮退"),
    (lambda t: t["y"].update(sign=0), "sign"),
    (lambda t: t["y"].update(axis="w"), "axis"),
    (lambda t: t.pop("z"), "z"),
])
def test_validate_rejects_bad_transform(mutate, phrase):
    transform = {k: dict(v) for k, v in VALID_TRANSFORM.items()}
    mutate(transform)
    error = validate_mapping(transform, {})
    assert error is not None and phrase in error


@pytest.mark.parametrize("attitude,phrase", [
    ({"forward_axis": "+w"}, "forward_axis"),
    ({"forward_axis": "+x", "up_axis": "-x"}, "同じ軸"),
    ({"yaw_sign": 2}, "yaw_sign"),
    ({"yaw_offset_deg": 400.0}, "yaw_offset_deg"),
    ({"flip_gate": 1.5}, "flip_gate"),
    ({"flip_correction": "yes"}, "flip_correction"),
])
def test_validate_rejects_bad_attitude(attitude, phrase):
    error = validate_mapping(VALID_TRANSFORM, attitude)
    assert error is not None and phrase in error


# ----------------------------------------------------------------------
# MocapSource: pose dict 契約と set_mapping
# ----------------------------------------------------------------------

class _RigidBody:
    def __init__(self, rb_id, pos, rot):
        self.id_num = rb_id
        self.pos = pos
        self.rot = rot
        self.tracking_valid = True
        self.error = 0.001
        self.rb_marker_list = []


def _make_source(**kwargs) -> MocapSource:
    natnet = {"server_address": "127.0.0.1", "client_address": "127.0.0.1",
              "use_multicast": True, "rigid_body_id": 1}
    return MocapSource(natnet, VALID_TRANSFORM, kwargs.get("attitude"),
                       client_factory=lambda: None)


def test_pose_dict_contains_new_fields():
    source = _make_source()
    q = quat_about_y(30.0 * DEG)
    pose = source._pose_from_rigid_body(
        _RigidBody(1, (0.1, 0.2, 0.3), q), 1, 42)
    assert pose["quat"] == pytest.approx(q)
    assert "yaw_true_rad" in pose and "flip_flags" in pose
    assert pose["flip_flags"] == 0
    # 既定 attitude では yaw_true == heading(sign=1, offset=0)
    assert pose["yaw_true_rad"] == pytest.approx(pose["heading_rad"])


def test_set_mapping_swaps_atomically_and_resets_state():
    source = _make_source()
    rb = _RigidBody(1, (1.0, 2.0, 3.0), quat_about_y(0.0))
    pose1 = source._pose_from_rigid_body(rb, 1, 1)
    assert (pose1["x"], pose1["y"], pose1["z"]) == pytest.approx(
        (3.0, -1.0, 2.0))
    flipped_y = {k: dict(v) for k, v in VALID_TRANSFORM.items()}
    flipped_y["y"]["sign"] = 1
    source.set_mapping(flipped_y, {"yaw_offset_deg": 90.0})
    pose2 = source._pose_from_rigid_body(rb, 1, 2)
    assert (pose2["x"], pose2["y"], pose2["z"]) == pytest.approx(
        (3.0, 1.0, 2.0))
    # y 符号反転で heading は厳密に符号反転する(位置とヨーは同一の誤り)
    assert pose2["heading_rad"] == pytest.approx(-pose1["heading_rad"])
    snapshot = source.mapping_snapshot()
    assert snapshot["coordinate_transform"]["y"]["sign"] == 1
    assert snapshot["attitude_transform"]["yaw_offset_deg"] == 90.0


# ----------------------------------------------------------------------
# ログ列契約との整合
# ----------------------------------------------------------------------

def test_new_meta_keys_are_logged_columns():
    for key in ("mocap_yaw_true_deg", "mocap_flip",
                "mocap_qx", "mocap_qy", "mocap_qz", "mocap_qw"):
        assert key in META_PASSTHROUGH_KEYS
        assert key in COLUMNS


# ----------------------------------------------------------------------
# session 層: 実行時変更 API(検証・地上ガード・永続化)
# ----------------------------------------------------------------------

@pytest.fixture
def control_json_path(tmp_path, monkeypatch):
    """テスト中の save_control_config が実 config を上書きしないよう退避。"""
    path = tmp_path / "control.json"
    monkeypatch.setattr(cfg, "CONTROL_CONFIG_PATH", path)
    return path


def _mapping_payload(y_sign=1, yaw_offset=90.0) -> dict:
    transform = {k: dict(v) for k, v in VALID_TRANSFORM.items()}
    transform["y"]["sign"] = y_sign
    return {"coordinate_transform": transform,
            "attitude_transform": {"yaw_offset_deg": yaw_offset}}


def test_update_mocap_mapping_applies_and_persists(
        session_factory, control_json_path):
    session, _, _ = session_factory()
    result = session.update_mocap_mapping(_mapping_payload())
    assert result["ok"], result
    # 適用: MocapSource のスナップショットに反映
    snapshot = session.mocap.mapping_snapshot()
    assert snapshot["coordinate_transform"]["y"]["sign"] == 1
    assert snapshot["attitude_transform"]["yaw_offset_deg"] == 90.0
    # メモリ反映(in-place: multi の生参照とも一致)
    assert session.control_config["coordinate_transform"]["y"]["sign"] == 1
    assert session.multi._control_config["coordinate_transform"]["y"]["sign"] == 1
    # 永続化
    saved = json.loads(control_json_path.read_text(encoding="utf-8"))
    assert saved["coordinate_transform"]["y"]["sign"] == 1
    assert saved["attitude_transform"]["yaw_offset_deg"] == 90.0


def test_update_mocap_mapping_rejects_invalid(session_factory,
                                              control_json_path):
    session, _, _ = session_factory()
    payload = _mapping_payload()
    payload["coordinate_transform"]["y"]["axis"] = "z"   # 縮退
    result = session.update_mocap_mapping(payload)
    assert not result["ok"]
    assert not control_json_path.exists()   # 保存されない
    assert session.mocap.mapping_snapshot()[
        "coordinate_transform"]["y"]["axis"] == "x"      # 適用もされない


def test_update_mocap_mapping_rejected_while_armed(session_factory,
                                                   control_json_path):
    from core import session as session_mod
    session, _, _ = session_factory()
    with session._lock:
        session._phase = session_mod.PHASE_ARMED
    result = session.update_mocap_mapping(_mapping_payload())
    assert not result["ok"] and "飛行中" in result["message"]
    assert not control_json_path.exists()


def test_mocap_mapping_status_shape(session_factory):
    session, _, _ = session_factory()
    status = session.mocap_mapping()
    assert status["ok"] and status["can_apply"]
    assert status["mapping"]["coordinate_transform"]["x"]["axis"] == "z"
    assert status["mapping"]["attitude_transform"]["forward_axis"] == "+x"
    assert status["primary_rigid_body_id"] == 1


# ----------------------------------------------------------------------
# レビュー指摘の回帰テスト(2026-07-27 敵対的レビュー)
# ----------------------------------------------------------------------

def test_continuity_guard_frozen_while_tracking_invalid(transformer):
    """tracking_valid=0(オクルージョン)中は継続性状態を更新しない。

    遮蔽中に実機が回転しても、再捕捉時はギャップ扱いで観測を信じ直す
    (遮蔽中の外挿姿勢で180°誤補正を蓄積しない)。
    """
    mapper = AttitudeMapper(None)
    state = mapper.new_state()
    q0 = quat_about_y(10.0 * DEG)
    mapper.derive(q0, transformer, state, 0.00)
    t_before = state["last_t"]
    # 遮蔽中: ソルバ外挿の擬似的な大ジャンプが来ても状態は不変
    q_flip = quat_mul(quat_about_y(math.pi), q0)
    _, _, flags = mapper.derive(q_flip, transformer, state, 0.01,
                                tracking_valid=False)
    assert not (flags & AttitudeMapper.FLIP_YAW_JUMP)
    assert state["cont_flip"] is False
    assert state["last_t"] == t_before          # 更新されていない
    # 0.6s 遮蔽ののち反対側で再捕捉: ギャップリセットで観測をそのまま信じる
    q1 = quat_about_y(-170.0 * DEG)
    _, yaw, flags = mapper.derive(q1, transformer, state, 0.61)
    assert not (flags & AttitudeMapper.FLIP_YAW_JUMP)
    _, yaw_fresh, _ = AttitudeMapper(None).derive(
        q1, transformer, AttitudeMapper.new_state(), 0.0)
    assert math.degrees(yaw) == pytest.approx(math.degrees(yaw_fresh))


def test_up_guard_frozen_while_tracking_invalid(transformer, mapper):
    """tracking_valid=0 中は上方軸ガードのヒステリシスも更新しない。"""
    state = mapper.new_state()
    q = quat_about_y(0.0)
    mapper.derive(q, transformer, state, 0.00)
    q_flip = quat_mul(q, FLIP_ABOUT_BODY_X)
    _, _, flags = mapper.derive(q_flip, transformer, state, 0.01,
                                tracking_valid=False)
    assert state["up_flipped"] is False
    assert not (flags & AttitudeMapper.FLIP_UP_INVERTED)


def test_continuity_guard_short_vertical_episode_resets(transformer, mapper):
    """前方軸が鉛直付近を通過したら(0.5s 未満でも)継続性を破棄する。

    鉛直中の方位は不定のため、復帰フレームはギャップ扱いになり
    蓄積した180°補正は残らない。
    """
    state = mapper.new_state()
    mapper.derive(quat_about_y(10.0 * DEG), transformer, state, 0.00)
    # 前方軸を鉛直へ(Motive Y-up: ボディ+X が上を向く回転 = Z まわり +90°)
    q_vert = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    mapper.derive(q_vert, transformer, state, 0.05)
    assert state["last_t"] is None              # 継続性を明示破棄
    # 0.1s 後(ギャップ 0.5s 未満)に水平で復帰 + 180° 跳んだ観測
    q_back = quat_about_y(-170.0 * DEG)
    _, yaw, flags = mapper.derive(q_back, transformer, state, 0.15)
    assert not (flags & AttitudeMapper.FLIP_YAW_JUMP)
    _, yaw_fresh, _ = AttitudeMapper(None).derive(
        q_back, transformer, AttitudeMapper.new_state(), 0.0)
    assert math.degrees(yaw) == pytest.approx(math.degrees(yaw_fresh))


def test_mapping_generation_stamped_and_incremented():
    """pose に世代が刻印され、set_mapping で +1 される。"""
    source = _make_source()
    rb = _RigidBody(1, (0.0, 0.0, 0.0), quat_about_y(0.0))
    assert source.mapping_generation == 0
    pose0 = source._pose_from_rigid_body(rb, 1, 1)
    assert pose0["mapping_gen"] == 0
    source.set_mapping(VALID_TRANSFORM, {})
    assert source.mapping_generation == 1
    pose1 = source._pose_from_rigid_body(rb, 1, 2)
    assert pose1["mapping_gen"] == 1


def test_position_controller_drops_stale_mapping_generation(
        server_config, control_config):
    """世代フロア未満の pose は on_mocap_pose で破棄される。

    マッピング差し替え直前に旧マッピングで計算されたフレームが、
    リセット直後のフィルタを旧座標系でシードする競合の防止。
    """
    from core.position import PositionController
    controller = PositionController(server_config, control_config,
                                    emit=lambda *a, **k: None)
    def pose(gen, x):
        return {"t_mono": 0.01 * gen, "x": x, "y": 0.0, "z": 0.3,
                "marker_count": 5, "tracking_valid": True, "quality": 1.0,
                "error": 0.001, "frame_number": gen, "yaw_rad": 0.0,
                "heading_rad": 0.0, "mapping_gen": gen}
    controller.set_mocap_mapping_floor(1)
    controller.on_mocap_pose(pose(0, 9.9))     # 旧世代 → 破棄
    assert controller.mocap_age_s(now=1.0) is None
    controller.on_mocap_pose(pose(1, 0.1))     # 新世代 → 受理
    assert controller.mocap_age_s(now=0.02) is not None
    snap = controller.mocap_snapshot(now=0.02)
    assert snap["x"] == pytest.approx(0.1, abs=0.01)


def test_update_mocap_mapping_resets_targets(session_factory,
                                             control_json_path):
    """マッピング適用で単機目標が既定へ戻る(旧座標系の絶対値を残さない)。"""
    session, _, _ = session_factory()
    session.position.set_target(1.0, -1.0, 0.5)
    result = session.update_mocap_mapping(_mapping_payload())
    assert result["ok"], result
    default = session.control_config["target_default"]
    assert session.position.get_target() == (
        default["x"], default["y"], default["z"])


# ----------------------------------------------------------------------
# 機体ワイヤフレーム変換(CMD_POS_ERR。2026-07-28 フレーム代数検証に基づく)
# ----------------------------------------------------------------------

RIGHT_HANDED_TRANSFORM = {
    "x": {"axis": "z", "sign": 1},
    "y": {"axis": "x", "sign": 1},    # レガシーの y 符号のみ反転(det=+1)
    "z": {"axis": "y", "sign": 1},
}

SWAPPED_TRANSFORM = {
    "x": {"axis": "x", "sign": 1},    # 軸入れ替え(ワイヤ変換の対応外)
    "y": {"axis": "z", "sign": 1},
    "z": {"axis": "y", "sign": 1},
}


def test_machine_wire_y_sign_classification():
    """レガシー=+1 / Y反転のみ=−1 / 軸入れ替え=None。"""
    assert CoordinateTransformer(None).machine_wire_y_sign == 1.0
    assert CoordinateTransformer(VALID_TRANSFORM).machine_wire_y_sign == 1.0
    assert CoordinateTransformer(
        RIGHT_HANDED_TRANSFORM).machine_wire_y_sign == -1.0
    assert CoordinateTransformer(SWAPPED_TRANSFORM).machine_wire_y_sign is None


class TestWireFrameConversion:
    """CMD_POS_ERR 送信時の機体フレーム変換(単機 session 経路)。"""

    def _meta(self, **overrides) -> dict:
        meta = {
            "mode": "position", "data_valid": True, "control_active": True,
            "mocap_dropout": False, "error_x": 0.35, "error_y": -0.2,
            "yaw_ref_rad": 0.5, "yaw_ctrl_on": True,
            "mocap_heading_rad": 1.62,
        }
        meta.update(overrides)
        return meta

    def _sent(self, transport):
        import stampfly_protocol as proto
        return [proto.CmdPosErr.from_payload(f.payload)
                for f in transport.sent_frames
                if f.type == proto.MsgType.CMD_POS_ERR]

    def _connect_quiet(self, session, transport):
        assert session.connect("COM-fake")
        session.posture.stop()
        session.position.stop()
        transport.sent_frames.clear()

    def test_legacy_mapping_unchanged(self, session_factory):
        """レガシーフレーム(det=−1)は無変換 = 従来とビット一致。"""
        import stampfly_protocol as proto
        session, transport, _ = session_factory()
        self._connect_quiet(session, transport)
        session._emit_setpoint(0.0, 0.0, 0.5, self._meta())
        pe = self._sent(transport)[0]
        assert pe.err_y == pytest.approx(-0.2)
        assert pe.mocap_yaw == pytest.approx(1.62)
        assert pe.flags & proto.CmdPosErr.FLAG_XY_ERR_VALID

    def test_right_handed_mapping_negates_err_y_and_heading(
            self, session_factory, control_json_path):
        """右手系マッピング適用中は err_y と方位角を反転して送る。

        検証済みフレーム代数: wire = diag(1,−1)·e_ctrl = e_legacy により
        機上XY制御は全ヨー角で負帰還に戻る(y=+0.2 目標で −y へ飛ぶ
        正帰還の修正)。
        """
        import stampfly_protocol as proto
        session, transport, _ = session_factory()
        result = session.update_mocap_mapping(
            {"coordinate_transform": RIGHT_HANDED_TRANSFORM,
             "attitude_transform": {}})
        assert result["ok"], result
        assert result["machine_frame"] == "mirrored_y"
        self._connect_quiet(session, transport)
        session._emit_setpoint(0.0, 0.0, 0.5, self._meta())
        pe = self._sent(transport)[0]
        assert pe.err_x == pytest.approx(0.35)      # x は不変
        assert pe.err_y == pytest.approx(+0.2)      # 反転
        assert pe.mocap_yaw == pytest.approx(-1.62)  # 方位角も反転
        assert pe.yaw_ref == pytest.approx(0.5)     # スライダ由来ヨーは不変
        assert pe.flags & proto.CmdPosErr.FLAG_XY_ERR_VALID

    def test_unsupported_mapping_disables_xy(self, session_factory,
                                             control_json_path):
        """軸入れ替えマッピングでは XY_ERR_VALID を立てない(飛行無効化)。"""
        import stampfly_protocol as proto
        session, transport, _ = session_factory()
        result = session.update_mocap_mapping(
            {"coordinate_transform": SWAPPED_TRANSFORM,
             "attitude_transform": {}})
        assert result["ok"], result
        assert result["machine_frame"] == "unsupported"
        self._connect_quiet(session, transport)
        session._emit_setpoint(0.0, 0.0, 0.5, self._meta())
        pe = self._sent(transport)[0]
        assert not (pe.flags & proto.CmdPosErr.FLAG_XY_ERR_VALID)


def test_tangent_yaw_negated_for_right_handed(server_config, control_config):
    """円軌道の接線ヨー(制御座標系方位角)は右手系では符号反転して指令。"""
    from core.position import PositionController
    outputs = {}

    def make(sign):
        emitted = []
        ctl = PositionController(server_config, control_config,
                                 emit=lambda r, p, a, meta: emitted.append(meta))
        ctl.set_yaw_azimuth_wire_sign(sign)
        ctl.set_yaw_control(True)
        # 円軌道を有効化(位相=0 開始、接線追従 ON)
        ctl._traj = {"center": (0.0, 0.0), "radius": 0.5, "period_s": 10.0,
                     "clockwise": False, "alt": 0.3, "face_tangent": True,
                     "phase0": 0.0, "t0": 0.0, "frozen_at": None}
        # mocap 有効状態を偽装(位相凍結を避ける)
        ctl._last_pose_t = 0.0
        ctl._last_data_valid = True
        # シェイパーは初回呼び出しで基準を確立し、以降スルーレートで
        # 目標へ向かう — 2回呼んで非ゼロの整形済みヨーを得る
        ctl.step(now=0.02)
        ctl._last_pose_t = 1.0
        ctl.step(now=1.0)
        return emitted[-1]["yaw_ref_rad"]

    outputs["legacy"] = make(1.0)
    outputs["right"] = make(-1.0)
    assert outputs["legacy"] != 0.0
    assert outputs["right"] == pytest.approx(-outputs["legacy"])
