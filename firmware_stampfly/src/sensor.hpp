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

#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <Arduino.h>
#include "flight_state.hpp"
#include "pid.hpp"
#include <MadgwickAHRS.h>
#include <common.h>
#include <stdint.h>
#include "alt_kalman.hpp"
#include <driver/spi_master.h>
#include "driver/gpio.h"
#include "sdkconfig.h"

#define SDA_PIN      (3)
#define SCL_PIN      (4)
#define PIN_NUM_MISO (43)
#define PIN_NUM_MOSI (14)
#define PIN_NUM_CLK  (44)
#define PIN_CS       (46)

typedef struct {
    spi_host_device_t host;  ///< The SPI host used, set before calling `spi_eeprom_init()`
    gpio_num_t cs_io;        ///< CS gpio number, set before calling `spi_eeprom_init()`
    gpio_num_t miso_io;      ///< MISO gpio number, set before calling `spi_eeprom_init()`
    bool intr_used;  ///< Whether to use polling or interrupt when waiting for write to be done. Set before calling
                     ///< `spi_eeprom_init()`.
} eeprom_config_t;

typedef struct eeprom_context_t* eeprom_handle_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quat_t;

typedef struct {
    uint16_t distance;
    uint16_t cnt;
} distance_t;

extern SensorState sensor_state;
extern volatile uint8_t ToF_bottom_data_ready_flag;

void sensor_init(void);
float sensor_read(void);
void sensor_reset_offset(void);
void sensor_calc_offset_avarage(void);
void ahrs_reset(void);
// Madgwickヨーのみゼロ化(ロール/ピッチ無傷)+ジャイロ積算0(ヨーゼロ設定用)
void ahrs_zero_yaw(void);
uint8_t scan_i2c(void);

#endif
