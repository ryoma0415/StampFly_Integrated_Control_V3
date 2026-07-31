"""MoCap ヨーの連続性フィルタ(ジャイロ予測ゲート)。

背景(2026-07-31 18:20 飛行の解析で確定):
- MoCap ソルバに「約90°別解グリッチ」が存在する(観測 26.4% / 54 ラン /
  最長 3.56s。フレーム間ジャンプ中央値 94.9°/frame に対し正常時 p99 は
  5.4°/frame、実機ジャイロの物理上限 ≈0.6°/frame)。
- 旧ヨー継続性ガード(cont_flip)は 180° 専用のトグル補正で、95° 級の
  ジャンプを 180° 反転と誤認してログ列を汚染した。さらに正しい基準を
  送ってもグリッチ中の連続棄却でファーム EKF2 の 25 連続棄却ラッチが
  飛行中に恒久発動する(リプレイ実証)。

本フィルタは 180° フリップも 90° グリッチも同一機構で扱う:
- 受理条件: |wrap(y_meas − y_pred)| < ゲート(30°)。y_pred は
  「最後に受理した値 + ジャイロレート r による伝播」(フレーム dt 実測)。
- 棄却中も y_pred を r で伝播し続け、計測が予測 ±ゲート内へ戻ったら
  再受理する(3.5s 級グリッチにも実機回頭に正しく追従する)。
- 棄却が REALIGN_S(5s)を超えて継続したら計測へ再整列する
  (realign_count を加算 — 真のトラッキング喪失からの復帰対応。
  呼び出し側がカウント増加を警告に変換する)。
- 受信ギャップ(> GAP_RESEED_S)を跨いだら連続性を主張せず計測で
  再シードする(旧ガードのギャップリセット規約を踏襲)。

レート r の規約: フィルタ対象のヨー系列と同じ角度規約・同じ符号で
渡すこと。本システムでは yaw_true(パネル較正適用後 = 機体ワイヤ規約)を
フィルタするため、テレメトリの r(機体 Z 軸レート)は符号変換なしで
そのまま渡る(mocap.py 参照)。r 欠落時(None)はホールド予測に落ちる
(正常回頭でもフレーム間変化 ≪ ゲートのため受理は阻害されない)。

NatNet 受信スレッド上で毎フレーム呼ばれる: ロック・ブロッキング禁止。
状態はインスタンス属性のみ(呼び出しスレッドが単一である前提)。
"""

from __future__ import annotations

from math import pi
from typing import Optional

# 受理ゲート [rad](構造定数)。ファーム EKF2 の 30° ゲートと同値にして
# 「PC が通した基準はファームゲートも通る」を構造的に揃える
GATE_RAD = pi / 6.0

# 棄却がこれを超えて継続したら計測へ再整列する [s](構造定数)。
# 実測グリッチ最長 3.56s < 5s < 真のトラッキング喪失(位置フェイルセーフが
# 先に 2s で CMD_STOP)という順序関係で選定
REALIGN_S = 5.0

# 受信ギャップがこれを超えたら連続性を主張しない [s](旧 cont_flip ガードの
# _YAW_JUMP_MAX_GAP_S と同値)
GAP_RESEED_S = 0.5


def wrap_pi(angle: float) -> float:
    """角度を (-pi, pi] へラップする(mocap.wrap_pi と同一。循環 import 回避)。"""
    while angle > pi:
        angle -= 2.0 * pi
    while angle <= -pi:
        angle += 2.0 * pi
    return angle


class YawContinuityFilter:
    """生ヨー系列(フレーム毎)のジャイロ予測ゲート。

    update(y_meas, t, rate) が (出力ヨー, 棄却中フラグ) を返す。出力は
    受理時 = 計測そのもの、棄却中 = ジャイロ伝播予測(コースト)。
    表示列とワイヤ経路の両方がこの出力を使う(単一情報源)。
    """

    def __init__(self) -> None:
        self._y_pred: Optional[float] = None    # 予測ヨー(直近受理値+伝播)
        self._last_t: Optional[float] = None
        self._reject_since: Optional[float] = None
        self.rejecting = False                  # 直近フレームが棄却か
        self.realign_count = 0                  # 再整列の累計(警告カウント)

    def invalidate(self) -> None:
        """連続性を明示破棄する(前方軸が鉛直に近い等、方位不定のフレーム)。

        次の update() は計測で再シードする(realign_count は増やさない)。
        """
        self._y_pred = None
        self._last_t = None
        self._reject_since = None
        self.rejecting = False

    def _seed(self, y_meas: float, t: float) -> None:
        self._y_pred = y_meas
        self._last_t = t
        self._reject_since = None
        self.rejecting = False

    def update(self, y_meas: float, t: float,
               rate_rad_s: Optional[float] = None) -> tuple[float, bool]:
        """1フレームぶんの受理判定。Returns: (出力ヨー [rad], 棄却中フラグ)。

        y_meas: 生ヨー計測 [rad](±π)。t: フレーム時刻 [s](単調)。
        rate_rad_s: 最新テレメトリのヨーレート(y_meas と同規約)。
          None ならホールド予測(伝播なし)。
        """
        last_t = self._last_t
        if (self._y_pred is None or last_t is None
                or t < last_t or (t - last_t) > GAP_RESEED_S):
            # 初回・ギャップ越し・時刻逆行: 連続性を主張できない → 再シード
            self._seed(y_meas, t)
            return y_meas, False

        dt = t - last_t
        self._last_t = t
        if rate_rad_s is not None:
            self._y_pred = wrap_pi(self._y_pred + rate_rad_s * dt)

        if abs(wrap_pi(y_meas - self._y_pred)) < GATE_RAD:
            # 受理: 予測を計測へ引き直す(棄却エピソードも閉じる)
            self._y_pred = y_meas
            self._reject_since = None
            self.rejecting = False
            return y_meas, False

        # 棄却: 予測でコーストする
        if self._reject_since is None:
            self._reject_since = t
        elif (t - self._reject_since) > REALIGN_S:
            # 棄却が長すぎる: 真のトラッキング喪失からの復帰とみなして
            # 計測へ再整列する(警告カウント付き)
            self.realign_count += 1
            self._seed(y_meas, t)
            return y_meas, False
        self.rejecting = True
        return self._y_pred, True
