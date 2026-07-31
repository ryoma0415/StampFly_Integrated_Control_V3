/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "sensor.hpp"
#include "imu.hpp"
#include "tof.hpp"
#include "flight_control.hpp"
#include "flow/flow_hub.hpp"
#include "yaw_estimation/angle_utils.hpp"
#include "yaw_estimation/sensor_hub_ff.hpp"

Madgwick Drone_ahrs;
SensorState sensor_state;

// INA3221 は yaw_estimation/current_sensor(CH2のみ・140µs変換・AVG16)が
// 唯一の所有者。電圧・電流とも 20Hz 読みに一本化した(V2契約 §2.2)。
Filter acc_filter;
Filter az_filter;
Filter raw_ax_filter;
Filter raw_ay_filter;
Filter raw_az_filter;
Filter raw_az_d_filter;
Filter raw_gx_filter;
Filter raw_gy_filter;
Filter raw_gz_filter;
Filter alt_filter;

// Sensor-private calibration state
static volatile float Roll_rate_offset = 0.0f, Pitch_rate_offset = 0.0f, Yaw_rate_offset = 0.0f;
static volatile float Accel_z_offset = 0.0f;
static volatile float Roll_rate_raw, Pitch_rate_raw, Yaw_rate_raw;
int16_t deltaX, deltaY;

volatile uint16_t Offset_counter = 0;

uint8_t scan_i2c() {
    USBSerial.println("I2C scanner. Scanning ...");
    delay(50);
    byte count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        Wire1.beginTransmission(i);        // Begin I2C transmission Address (i)
        if (Wire1.endTransmission() == 0)  // Receive 0 = success (ACK response)
        {
            USBSerial.print("Found address: ");
            USBSerial.print(i, DEC);
            USBSerial.print(" (0x");
            USBSerial.print(i, HEX);
            USBSerial.println(")");
            count++;
        }
    }
    USBSerial.print("Found ");
    USBSerial.print(count, DEC);  // numbers of devices
    USBSerial.println(" device(s).");
    return count;
}

void sensor_reset_offset(void) {
    Roll_rate_offset  = 0.0f;
    Pitch_rate_offset = 0.0f;
    Yaw_rate_offset   = 0.0f;
    Accel_z_offset    = 0.0f;
    Offset_counter    = 0;
}

void sensor_calc_offset_avarage(void) {
    Roll_rate_offset  = (Offset_counter * Roll_rate_offset + Roll_rate_raw) / (Offset_counter + 1);
    Pitch_rate_offset = (Offset_counter * Pitch_rate_offset + Pitch_rate_raw) / (Offset_counter + 1);
    Yaw_rate_offset   = (Offset_counter * Yaw_rate_offset + Yaw_rate_raw) / (Offset_counter + 1);
    Accel_z_offset    = (Offset_counter * Accel_z_offset + sensor_state.Accel_z_raw) / (Offset_counter + 1);

    Offset_counter++;
}

void ahrs_reset(void) {
    Drone_ahrs.reset();
    // Z軸ジャイロ単純積算はAHRSと同じ「離陸時0基準」に揃える(V2契約 §2.2)
    sensor_state.Yaw_gyro_integral = 0.0f;
}

// ヨーゼロ設定(CMD_YAWZERO_SET valid=1)と同時に、Madgwickヨーとジャイロ積算の
// 0基準も現在方位へ揃える(3系統のヨー[CF/EKF・Madgwick・ジャイロ積算]の基準統一)。
// ahrs_reset() と違いクォータニオンはヨー成分のみ回転(q ← q_z(-ψ_M) ⊗ q)する
// ため、ロール/ピッチは無傷・収束過渡も発生しない。
//
// 軸入替規約との整合(本ファイル sensor_read 参照):
//   Madgwick へは gx=Pitch_rate, gy=Roll_rate, gz=-Yaw_rate を入力しており、
//   Madgwick内部フレームは (X_M=機体右, Y_M=機体前, Z_M=機体上) の右手系。
//   表示ヨーは Yaw_angle = wrapPi(-getYawRadians()) すなわち ψ_M の符号反転。
//   zeroYaw() は Madgwick 内部ヨー ψ_M を 0 にするので、Yaw_angle = -0 = 0 と
//   なり「表示ヨーが0になる」仕様を満たす(符号反転は 0 を不変に保つ)。
void ahrs_zero_yaw(void) {
    Drone_ahrs.zeroYaw();
    sensor_state.Yaw_gyro_integral = 0.0f;
}

void sensor_init() {
    // beep_init();

    Wire1.begin(SDA_PIN, SCL_PIN, 400000UL);
    if (scan_i2c() == 0) {
        USBSerial.printf("No I2C device!\r\n");
        USBSerial.printf("Can not boot AtomFly2.\r\n");
        while (1);
    }

    tof_init();
    imu_init();
    Drone_ahrs.begin(400.0);

    // ヨー推定モジュール初期化: BMM150 / INA3221(CH2のみ・140µs・AVG16)と
    // NVS 復元(mag3d → accel6 → attmount → geomag → yawzero → ffcal)。
    // 旧 ina3221.begin + 毎tick getVoltage は current_sensor の 20Hz 読みへ統一。
    sensorHubFfInit(Wire1);

    // オプティカルフロー初期化(MAG_AUTOTUNE_DESIGN.md §2.4/§2.5):
    // PMW3901(BMI270 と SPI2_HOST 共有。imu_init 済みが前提)+ NVS `flowcal`
    // ロード(ブートロード順の最後 = magbias の後)+ 起動時 burst セルフテスト。
    // burst 不成立ならフロー恒久無効(飛行挙動は完全に従来どおり)。
    flowHubInit();

    uint16_t cnt = 0;
    while (cnt < 10) {
        if (ToF_bottom_data_ready_flag) {
            ToF_bottom_data_ready_flag = 0;
            cnt++;
            USBSerial.printf("%d %d\n\r", cnt, tof_bottom_get_range());
        }
    }
    delay(10);

    // Acceleration filter
    acc_filter.set_parameter(0.005, 0.0025);

    raw_ax_filter.set_parameter(0.003, 0.0025);
    raw_ay_filter.set_parameter(0.003, 0.0025);
    raw_az_filter.set_parameter(0.003, 0.0025);

    raw_gx_filter.set_parameter(0.003, 0.0025);
    raw_gy_filter.set_parameter(0.003, 0.0025);
    raw_gz_filter.set_parameter(0.003, 0.0025);

    raw_az_d_filter.set_parameter(0.1, 0.0025);  // alt158
    az_filter.set_parameter(0.1, 0.0025);        // alt158
    alt_filter.set_parameter(0.005, 0.0025);
}

float sensor_read(void) {
    float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
    float ax, ay, az, gx, gy, gz, acc_norm, rate_norm;
    static float dp, dq, dr;
    static uint16_t dcnt = 0u;
    int16_t deff;
    static int16_t old_range[4]    = {0};
    static float alt_time          = 0.0f;
    static float sensor_time       = 0.0f;
    static float old_alt_time      = 0.0f;
    static uint8_t first_flag      = 0;
    static AutoFlightState preAutoState = AUTO_INIT;
    static uint8_t outlier_counter = 0;
    static float tof_height_m      = 0.0f;  // チルト補正後のToF高度 [m](ゲート棄却中は直前値保持)
    const uint8_t interval         = 400 / 30 + 1;
    float old_sensor_time          = 0.0;
    uint32_t st;
    float sens_interval;
    float h;
    static float opt_interval = 0.0;
    bool tof_read_this_tick = false;  // 今tickでToF読みが発生(磁気/電流の位相スタガ用)

    st              = micros();
    old_sensor_time = sensor_time;
    sensor_time     = (float)st * 1.0e-6;
    sens_interval   = sensor_time - old_sensor_time;
    opt_interval    = opt_interval + sens_interval;

    // 以下では航空工学の座標軸の取り方に従って
    // X軸：前後（前が正）左肩上がりが回転の正
    // Y軸：右左（右が正）頭上げが回転の正
    // Z軸：下上（下が正）右回りが回転の正
    // となる様に軸の変換を施しています
    // BMI270の座標軸の撮り方は
    // X軸：右左（右が正）頭上げが回転の正
    // Y軸：前後（前が正）左肩上がりが回転の正
    // Z軸：上下（上が正）左回りが回転の正

    // Get IMU raw data
    imu_update();  // IMUの値を読む前に必ず実行
    acc_x  = imu_get_acc_x();
    acc_y  = imu_get_acc_y();
    acc_z  = imu_get_acc_z();
    gyro_x = imu_get_gyro_x();
    gyro_y = imu_get_gyro_y();
    gyro_z = imu_get_gyro_z();

    // USBSerial.printf("%9.6f %9.6f %9.6f\n\r", flight_control_state.timing.Elapsed_time, sens_interval, acc_z);

    // Axis Transform
    sensor_state.Accel_x_raw    = acc_y;
    sensor_state.Accel_y_raw    = acc_x;
    sensor_state.Accel_z_raw    = -acc_z;
    Roll_rate_raw  = gyro_y;
    Pitch_rate_raw = gyro_x;
    Yaw_rate_raw   = -gyro_z;

    // モーター停止状態への遷移時にStatic変数を初期化（外れ値除去のバグ対策）
    if (((flight_control_state.mode.auto_state == AUTO_WAIT || flight_control_state.mode.auto_state == AUTO_COMPLETE || flight_control_state.mode.auto_state == AUTO_CALIBRATION) &&
         !(preAutoState == AUTO_WAIT || preAutoState == AUTO_COMPLETE || preAutoState == AUTO_CALIBRATION)))
    {
        first_flag   = 0;
        old_range[0] = 0;
        old_range[1] = 0;
        old_range[2] = 0;
        old_range[3] = 0;

        raw_ax_filter.reset();
        raw_ay_filter.reset();
        raw_az_filter.reset();
        raw_az_d_filter.reset();

        raw_gx_filter.reset();
        raw_gy_filter.reset();
        raw_gz_filter.reset();

        az_filter.reset();
        alt_filter.reset();

        acc_filter.reset();
    }

    if (flight_control_state.mode.auto_state > AUTO_CALIBRATION) {
        sensor_state.Accel_x   = raw_ax_filter.update(sensor_state.Accel_x_raw, flight_control_state.timing.Interval_time);
        sensor_state.Accel_y   = raw_ay_filter.update(sensor_state.Accel_y_raw, flight_control_state.timing.Interval_time);
        sensor_state.Accel_z   = raw_az_filter.update(sensor_state.Accel_z_raw, flight_control_state.timing.Interval_time);
        sensor_state.Accel_z_d = raw_az_d_filter.update(sensor_state.Accel_z_raw - Accel_z_offset, flight_control_state.timing.Interval_time);

        sensor_state.Roll_rate  = raw_gx_filter.update(Roll_rate_raw - Roll_rate_offset, flight_control_state.timing.Interval_time);
        sensor_state.Pitch_rate = raw_gy_filter.update(Pitch_rate_raw - Pitch_rate_offset, flight_control_state.timing.Interval_time);
        sensor_state.Yaw_rate   = raw_gz_filter.update(Yaw_rate_raw - Yaw_rate_offset, flight_control_state.timing.Interval_time);
        // Z軸ジャイロ単純積算(400Hz): Yaw_rate 確定直後に積算し、ahrs_reset()で
        // ゼロクリアする(ドリフト評価用の第3のヨー系統。V2契約 §2.2)
        sensor_state.Yaw_gyro_integral += sensor_state.Yaw_rate * flight_control_state.timing.Interval_time;

        Drone_ahrs.updateIMU((sensor_state.Pitch_rate) * (float)RAD_TO_DEG, (sensor_state.Roll_rate) * (float)RAD_TO_DEG,
                             -(sensor_state.Yaw_rate) * (float)RAD_TO_DEG, sensor_state.Accel_y, sensor_state.Accel_x, -sensor_state.Accel_z);
        sensor_state.Roll_angle  = Drone_ahrs.getPitch() * (float)DEG_TO_RAD;
        sensor_state.Pitch_angle = Drone_ahrs.getRoll() * (float)DEG_TO_RAD;
        // getYaw() は Arduino 版 Madgwick のコンパス方位規約(内部 atan2 値
        // +180° の 0..360°)のため使わない。生の atan2 値(±π)を符号反転して
        // ラップし、リセット直後 0(=離陸方位)・±π 範囲という他のヨー系統
        // (ジャイロ積算・CF・EKF)と同じ規約に揃える。
        sensor_state.Yaw_angle   = wrapPi(-Drone_ahrs.getYawRadians());

        // for debug
        // USBSerial.printf("%6.3f %7.4f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n\r",
        //   flight_control_state.timing.Elapsed_time, flight_control_state.timing.Interval_time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);

        // Get altitude (30Hz)
        sensor_state.Az = az_filter.update(-sensor_state.Accel_z_d, sens_interval);

        if (dcnt > interval) {
            if (ToF_bottom_data_ready_flag) {
                dcnt                       = 0u;
                old_alt_time               = alt_time;
                alt_time                   = micros() * 1.0e-6;
                h                          = alt_time - old_alt_time;
                ToF_bottom_data_ready_flag = 0;
                tof_read_this_tick         = true;  // 重量級I2C読みtick → 磁気/電流を繰延べ

                // 距離の値の更新
                // old_range[0] = dist;
                sensor_state.RawRange = tof_bottom_get_range();
                if (flight_control_state.mode.auto_state == AUTO_WAIT || flight_control_state.mode.auto_state == AUTO_COMPLETE || flight_control_state.mode.auto_state == AUTO_CALIBRATION) sensor_state.RawRangeFront = tof_front_get_range();
                // USBSerial.printf("%9.6f %d\n\r", flight_control_state.timing.Elapsed_time, sensor_state.RawRange);
                if (sensor_state.RawRange > 20) {
                    sensor_state.Range = sensor_state.RawRange;
                }
                if (sensor_state.RawRangeFront > 0.01) {
                    sensor_state.RangeFront = sensor_state.RawRangeFront;
                }

                // 外れ値処理
                deff = sensor_state.Range - old_range[1];
                if (deff > 500 && outlier_counter < 2) {
                    sensor_state.Range = old_range[1] + (old_range[1] - old_range[3]) / 2;
                    outlier_counter++;
                } else if (deff < -500 && outlier_counter < 2) {
                    sensor_state.Range = old_range[1] + (old_range[1] - old_range[3]) / 2;
                    outlier_counter++;
                }
                // old_range[3] = old_range[2];
                // old_range[2] = old_range[1];
                // old_range[1] = sensor_state.Range;
                else {
                    outlier_counter = 0;
                    old_range[3]    = old_range[2];
                    old_range[2]    = old_range[1];
                    old_range[1]    = sensor_state.Range;
                }

                // ToF チルト補正: スラント距離を鉛直高度へ変換(height = slant·cosφ·cosθ)。
                // チルトがゲート超のときは新規測距を高度系に反映せず直前値を保持(外れ値処理と同方針)
                if (FLIGHT_CONFIG.tof_tilt_comp_enabled) {
                    float tilt = sqrtf(sensor_state.Roll_angle * sensor_state.Roll_angle + sensor_state.Pitch_angle * sensor_state.Pitch_angle);
                    if (tilt <= FLIGHT_CONFIG.tof_tilt_gate_rad) {
                        tof_height_m = (float)sensor_state.Range / 1000.0f * cosf(sensor_state.Roll_angle) * cosf(sensor_state.Pitch_angle);
                    }
                } else {
                    tof_height_m = (float)sensor_state.Range / 1000.0f;
                }

                // USBSerial.printf("%9.6f, %9.6f, %9.6f, %9.6f, %9.6f\r\n",flight_control_state.timing.Elapsed_time,sensor_state.Altitude/1000.0,  sensor_state.Altitude2,
                // sensor_state.Alt_velocity,-(sensor_state.Accel_z_raw - Accel_z_offset)*9.81/(-Accel_z_offset));
            }
        } else
            dcnt++;

        sensor_state.Altitude = alt_filter.update(tof_height_m, flight_control_state.timing.Interval_time);
        if (first_flag == 1)
            sensor_state.EstimatedAltitude.update(sensor_state.Altitude, sensor_state.Az, flight_control_state.timing.Interval_time);
        else
            first_flag = 1;
        sensor_state.Altitude2 = sensor_state.EstimatedAltitude.Altitude;
        // MAX_ALTを超えたら高度下げる（自動着陸）
        if ((sensor_state.Altitude2 > ALT_LIMIT && flight_control_state.mode.Alt_flag >= 1) || sensor_state.RawRange == 0)
            sensor_state.Range0flag++;
        else
            sensor_state.Range0flag = 0;
        if (sensor_state.Range0flag > RANGE0_FLAG_MAX) sensor_state.Range0flag = RANGE0_FLAG_MAX;
        sensor_state.Alt_velocity = sensor_state.EstimatedAltitude.Velocity;
        sensor_state.Az_bias      = sensor_state.EstimatedAltitude.Bias;
        // USBSerial.printf("Sens=%f Az=%f Altitude=%f Velocity=%f Bias=%f\n\r",sensor_state.Altitude, sensor_state.Az, sensor_state.Altitude2, sensor_state.Alt_velocity,
        // sensor_state.Az_bias);
    }

    // Accel fail safe
    acc_norm = sqrt(sensor_state.Accel_x * sensor_state.Accel_x + sensor_state.Accel_y * sensor_state.Accel_y + sensor_state.Accel_z_d * sensor_state.Accel_z_d);
    sensor_state.Acc_norm = acc_filter.update(acc_norm, flight_control_state.timing.Control_period);
    if (sensor_state.Acc_norm > OVER_G_THRESHOLD) {
        sensor_state.OverG_flag = 1;
        if (sensor_state.Over_g == 0.0) sensor_state.Over_g = acc_norm;
    }

    // ---- ヨー推定・電流FF補正の統合tick(V2契約 §2.2) ----
    // 20Hz スロット(電流→磁気の順)・EKF predict(毎tick)/update(fresh磁気のみ)・
    // アイドルアンカーは sensorHubFfUpdate 内で回る。ToF読みtickでは磁気/電流を
    // スキップして次tickへ繰延べる(位相スタガ)。
    {
        const auto& fout = flight_control_state.output;
        const AutoFlightState ast = flight_control_state.mode.auto_state;
        SensorHubFfInputs ff_in;
        // ω_z = −gyro_z − 起動offset(yaw側と同一の未フィルタ規約で渡す)
        ff_in.yaw_rate_rad_s = Yaw_rate_raw - Yaw_rate_offset;
        // p = gyro_y − 起動offset(EKFチルト運動学予測用。ω_z と同じ未フィルタ規約)
        ff_in.roll_rate_rad_s = Roll_rate_raw - Roll_rate_offset;
        ff_in.roll_rad = sensor_state.Roll_angle;
        ff_in.pitch_rad = sensor_state.Pitch_angle;
        ff_in.dt_s = flight_control_state.timing.Interval_time;
        ff_in.tof_read_this_tick = tof_read_this_tick;
        // 飛行中はブロッキングし得る BMM150 再初期化リトライを保留する
        // (begin() の delay(5)/delay(10) が 400Hz ループの dt を乱すため)
        const bool in_flight =
            (ast == AUTO_TAKEOFF || ast == AUTO_HOVER || ast == AUTO_LANDING);
        ff_in.in_flight = in_flight;
        // EKF2 飛行状態再アンカー(MAG_AUTOTUNE_DESIGN.md §2.1-3)の条件判定用
        ff_in.alt_est_m = sensor_state.Altitude2;
        ff_in.alt_ref_m = fout.Alt_ref;
        if (ast == AUTO_MOTOR_TEST) {
            // MOTOR_TEST 中: テスト duty×mask を FF へ配線(V2契約 §2.3)。
            // ランプダウン中の実出力>0 も「回転中」として扱う(アンカー窓の汚染防止)。
            const float test_duty = motor_test_applied_duty();
            ff_in.motors_running = motor_test_output_active();
            ff_in.duty[0] = test_duty;  // FL,FR,RL,RR とも共通duty(mask外は compute が0扱い)
            ff_in.duty[1] = test_duty;
            ff_in.duty[2] = test_duty;
            ff_in.duty[3] = test_duty;
            ff_in.motor_mask = motor_test_active_mask();
        } else {
            // 「回転中」= 飛行状態、またはPWM実出力あり(アンカー窓の汚染防止)。
            ff_in.motors_running =
                in_flight ||
                fout.FrontLeft_motor_duty > 0.0f || fout.FrontRight_motor_duty > 0.0f ||
                fout.RearLeft_motor_duty > 0.0f || fout.RearRight_motor_duty > 0.0f;
            // FF差動項へのduty配線は FL,FR,RL,RR の順(ベース変数は FR,FL,RR,RL 順
            // なのでここで並べ替える。V2契約 §2.3)
            ff_in.duty[0] = fout.FrontLeft_motor_duty;
            ff_in.duty[1] = fout.FrontRight_motor_duty;
            ff_in.duty[2] = fout.RearLeft_motor_duty;
            ff_in.duty[3] = fout.RearRight_motor_duty;
            ff_in.motor_mask = 0x0F;
        }
        sensorHubFfUpdate(ff_in);
    }

    // ---- オプティカルフロー 50Hz スロット(MAG_AUTOTUNE_DESIGN.md §2.4) ----
    // 既存 20Hz 低速スロットと同居しない独立 20ms 周期(flow_hub 内で自己
    // スロットル)。SPI(BMI270 共有)なので ToF/磁気の I2C とはバスが別だが、
    // 重量級読みの発生した tick(ToF 読み・20Hz 電流/磁気スロット)では次 tick
    // へ繰延べて tick 予算(2500µs)を保護する(位相スタガと同方針)。
    // 回転補償の p,q・ToF 生スラント距離・ヨーは既存 sensor_state / hub から。
    // burst 不成立時は flowHubUpdate が即 return(飛行挙動不変)。
    if (!tof_read_this_tick && !g_yaw_est.current_slot_fired) {
        flowHubUpdate(sensor_state.Roll_rate, sensor_state.Pitch_rate,
                      static_cast<float>(sensor_state.Range) * 1.0e-3f,
                      sensor_state.RawRange > 20,
                      g_yaw_est.ff.yaw_active_rad);
    }

    // Battery voltage check
    // 電圧はINA3221(CH2)の20Hz読み(bus_voltage)に一本化(毎tick getVoltage 廃止)。
    // 低電圧判定「<3.34V が 0.25s 継続」は 20Hz×5サンプル連続に置換(意味を保存)。
    // 平滑は INA3221 のHW平均(AVG16)が旧 voltage_filter を代替する。
    // 計測不能(INA3221不調)な20Hzスロットも低電圧と同様に連続カウントし、
    // 「電圧が測れない機体は飛ばさない」という旧挙動の安全側を保存する。
    if (g_yaw_est.current_slot_fired) {
        const bool sample_valid = g_yaw_est.current_sample.valid;
        if (sample_valid) {
            sensor_state.Voltage   = g_yaw_est.current_sample.bus_voltage_v;
            sensor_state.Current_a = g_yaw_est.current_sample.current_a;
        }
        if (sensor_state.Under_voltage_flag != UNDER_VOLTAGE_COUNT) {
            if (!sample_valid || sensor_state.Voltage < POWER_LIMIT)
                sensor_state.Under_voltage_flag++;
            else
                sensor_state.Under_voltage_flag = 0;
            if (sensor_state.Under_voltage_flag > UNDER_VOLTAGE_COUNT) sensor_state.Under_voltage_flag = UNDER_VOLTAGE_COUNT;
        }
    }

    // ヨー推定の公開値を SensorState へ転記(telemetry/飛行制御は sensor_state 経由で読む)
    sensor_state.Yaw_est_rad     = g_yaw_est.ff.yaw_active_rad;
    sensor_state.Yaw_ekf_rad     = g_yaw_est.yaw_kf.yaw();
    sensor_state.Db_hat_x_ut     = g_yaw_est.ff.delta_b.x;
    sensor_state.Db_hat_y_ut     = g_yaw_est.ff.delta_b.y;
    sensor_state.Bm_x_ut         = g_yaw_est.yaw_kf.bmx();
    sensor_state.Bm_y_ut         = g_yaw_est.yaw_kf.bmy();
    sensor_state.Ekf_nis         = g_yaw_est.yaw_kf.nis();
    sensor_state.Ekf_ffg         = g_yaw_est.yaw_kf.gateBits();
    sensor_state.Ff_mode         = g_yaw_est.ff.ff_mode;
    sensor_state.Est_mode        = g_yaw_est.ff.est_mode;
    sensor_state.Ff_anchor_valid = g_yaw_est.ff.anchor_valid ? 1 : 0;
    sensor_state.Ff_cal_loaded   = g_yaw_est.ff_calibration.valid() ? 1 : 0;
    sensor_state.Mag_fresh       = sensorHubFfMagFresh(millis()) ? 1 : 0;

    // 磁気オートチューン拡張(TLM_STATE 135-172。MAG_AUTOTUNE_DESIGN.md §1.1/§2.2):
    // b_cal・レベル化観測 z・EKF2 の公開値を転記(EKF2 はシャドー中も常時)
    sensor_state.Mag_cal_x_ut       = g_yaw_est.frame.mag_cal_body.x;
    sensor_state.Mag_cal_y_ut       = g_yaw_est.frame.mag_cal_body.y;
    sensor_state.Mag_cal_z_ut       = g_yaw_est.frame.mag_cal_body.z;
    sensor_state.Mag_lev_x_ut       = g_yaw_est.frame.mag_lev_x_ut;
    sensor_state.Mag_lev_y_ut       = g_yaw_est.frame.mag_lev_y_ut;
    sensor_state.Ekf2_yaw_rad       = g_yaw_est.yaw_kf2.yaw();
    sensor_state.Ekf2_bm_x_ut       = g_yaw_est.yaw_kf2.bmx();
    sensor_state.Ekf2_bm_y_ut       = g_yaw_est.yaw_kf2.bmy();
    sensor_state.Ekf2_yaw_innov_rad = g_yaw_est.yaw_kf2.yawInnov();
    sensor_state.Ekf2_status        = sensorHubFfEkf2Status(millis());
    sensor_state.Ekf2_gate          = g_yaw_est.yaw_kf2.gateBits();

    // オプティカルフロー公開値の転記(TLM_STATE 173-183。契約 §1.1/§2.4)
    {
        const FlowState& fs = flowHubState();
        sensor_state.Flow_vx_mps = fs.vx;
        sensor_state.Flow_vy_mps = fs.vy;
        sensor_state.Flow_squal  = fs.squal;
        sensor_state.Flow_status = flowHubStatusByte();
        float flow_dt_ms = fs.read_dt_s * 1000.0f;
        if (flow_dt_ms < 0.0f) flow_dt_ms = 0.0f;
        if (flow_dt_ms > 255.0f) flow_dt_ms = 255.0f;
        sensor_state.Flow_dt_ms = static_cast<uint8_t>(flow_dt_ms);
    }

    preAutoState = flight_control_state.mode.auto_state;  // 今の状態を記憶

    uint32_t et = micros();
    // USBSerial.printf("Sensor read %f %f %f\n\r", (mt-st)*1.0e-6, (et-mt)*1e-6, (et-st)*1.0e-6);
    return (et - st) * 1.0e-6;
}
