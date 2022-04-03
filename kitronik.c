/*
    Bumblebeep: Autonomous robot firmware for Kitronik
    Copyright (C) 2021 J. E. Blundell

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
                            GPIO
                            ----
                  LED       25

Button          1 UART_TX   0 ..       VBUS 40
                2 UART_RX   1 ..       VSYS 39
                3 GND                   GND 38
Echo Rear       4           2 ..     3V3_EN 37
Trigger Rear    5           3 ..        3V3 36
[SDA]           6 SDA       4 ..     ADCREF 35
[SCL]           7 SCL       5 28       ADC2 34   Left LF
                8 GND      .. ..       AGND 33
Motor 2 FWD     9           6 27       ADC1 32   Centre LF
Motor 2 REV    10           7 26       ADC0 31   Right LF
[DRDY]         11           8 ..        RUN 30
[INT]          12           9 22            29
               13 GND      .. ..        GND 28
Servo1         14          10 21            27   Servo0
Servo3         15          11 20            26   Motor 1 FWD
               16          12 19     SPI_TX 25   Motor 1 REV
               17          13 18    SPI_SCK 24   LEDs
               18 GND      .. ..        GND 23
Trigger Front  19          14 17     SPI_CS 22   Servo2
Echo Front     20          15 16     SPI_RX 21   Buzzer

                      SWCLK GND SWDIO

GY-271 QMC5883L Triple axis compass magnetometer [SCL,SDA,DRDY]
GY-521 MPU6050 6 axis accelerometer              [SCL,SDA,INT]
*/


/*** Preamble *****************************************************************/
#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/pio.h>
#include <hardware/timer.h>
#include "generated/kitronik.pio.h"
#include "kitronik.h"

PIO led_pio;
int led_sm;
uint32_t led_buf[LED_N];

float distbuf[DISTBUF_N];
uint64_t distbuf_last;
uint8_t distbuf_n;

/*** Motor control ************************************************************/
void motor_init (uint fwd, uint rev) {
    uint slice_f = pwm_gpio_to_slice_num (fwd);
    uint slice_r = pwm_gpio_to_slice_num (rev);
    uint chan_f = pwm_gpio_to_channel (fwd);
    uint chan_r = pwm_gpio_to_channel (fwd);
    pwm_config cfg;

    cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int (&cfg, PWM_DIV);
    pwm_config_set_clkdiv_mode (&cfg, PWM_DIV_FREE_RUNNING);
    pwm_init (slice_f, &cfg, false);
    pwm_init (slice_r, &cfg, false);
    gpio_set_function (fwd, GPIO_FUNC_PWM);
    gpio_set_function (rev, GPIO_FUNC_PWM);
    pwm_set_chan_level (slice_f, chan_f, 0);
    pwm_set_chan_level (slice_r, chan_r, 0);
    pwm_set_enabled (slice_f, true);
    pwm_set_enabled (slice_r, true);
}
void motor_set (uint fwd, uint rev, int32_t f_speed) {
    uint16_t f_lev, r_lev;
    f_lev = (f_speed <= 0) ? 0 : f_speed & 0xffff;
    r_lev = (f_lev > 0) ? 0 : f_speed & 0xffff;
    pwm_set_gpio_level (fwd, f_lev);
    pwm_set_gpio_level (rev, r_lev);
}

/*** LED control **************************************************************/
void led_init () {
    led_pio = pio0;
    led_sm = 0;
    uint offset = pio_add_program (led_pio, &kitronik_program);
    kitronik_program_init(led_pio, led_sm, offset, PIN_LED, LED_FREQ);
}
void led_write (uint8_t n, uint8_t r, uint8_t g, uint8_t b) {
    led_buf[n] = (g<<16) | (r<<8) | b;
}
void led_commit () {
    uint8_t i;
    for (i=0; i<LED_N; i++) {
        pio_sm_put (led_pio, led_sm, led_buf[i] << 8);
    }
    while (!pio_sm_is_tx_fifo_empty(led_pio, led_sm)) {
    }
}

/*** Ultrasound control *******************************************************/
void us_init (uint trigger, uint echo) {
    gpio_init (trigger);
    gpio_set_dir (trigger, GPIO_OUT);
    gpio_init (echo);
    gpio_set_dir (echo, GPIO_IN);
}
float us_cm (uint trigger, uint echo, uint32_t timeout) {
    uint32_t t_start, t_diff;

    gpio_put (trigger, 0);
    sleep_us (2);
    gpio_put (trigger, 1);
    sleep_us (5);
    gpio_put (trigger, 0);
    t_start = time_us_64();
    do {
        t_diff = time_us_64() - t_start;
        if (gpio_get(echo) != 0) {
            t_start = time_us_64();
            break;
        }
    } while (t_diff < timeout);
    if (t_diff > timeout) {
        return -1;
    }

    do {
        t_diff = time_us_64() - t_start;
        if (gpio_get(echo) == 0) {
            return ((float)(t_diff) * 0.0343) / 2;
        }
    } while (t_diff < timeout);
    return -1;
}

/*** Buzzer control ***********************************************************/
void buzzer_init () {
    uint slice_num = pwm_gpio_to_slice_num(PIN_BUZZER);
    uint channel = pwm_gpio_to_channel (PIN_BUZZER);
    pwm_config cfg = pwm_get_default_config ();
    gpio_set_function (PIN_BUZZER, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv (&cfg, clock_get_hz(clk_sys) / (500 * 65536));
    pwm_init (slice_num, &cfg, false);
    pwm_set_wrap (slice_num, 0xffff);
    pwm_set_chan_level (slice_num, channel, 0x8000);
}
void buzzer_freq (uint64_t freq, uint ms) {
    uint slice_num = pwm_gpio_to_slice_num(PIN_BUZZER);
    uint channel = pwm_gpio_to_channel (PIN_BUZZER);
    pwm_config cfg = pwm_get_default_config ();
    gpio_set_function (PIN_BUZZER, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv (&cfg, clock_get_hz(clk_sys) / (freq * 65536));
    pwm_init (slice_num, &cfg, true);
    pwm_set_wrap (slice_num, 0xffff);
    pwm_set_chan_level (slice_num, channel, 0x8000);
    sleep_ms (ms);
    pwm_set_enabled (slice_num, false);
}

/*** Distance averaging *******************************************************/
void distbuf_init () {
    uint8_t i;

    for (i=0; i<DISTBUF_N; i++) {
        distbuf[i] = 0;
    }
    distbuf_last = time_us_64 ();
    distbuf_n = 0;
}
void distbuf_push (float dist) {
    distbuf[distbuf_n++] = dist;
    if (distbuf_n >= DISTBUF_N) {
        distbuf_n = 0;
    }
    distbuf_last = time_us_64 ();
}
float distbuf_min () {
    uint8_t i;
    float out = distbuf[0];
    for (i=1, out=distbuf[0]; i<DISTBUF_N; i++) {
        if (distbuf[i] < out) {
            out = distbuf[i];
        }
    }
    return out;
}
float distbuf_mean () {
    uint8_t i;
    float out;
    for (i=0,out=0; i<DISTBUF_N; i++) {
        out += distbuf[i] / ((float)DISTBUF_N);
    }
    return out;
}
float distbuf_max () {
    uint8_t i;
    float out = distbuf[0];
    for (i=1, out=distbuf[0]; i<DISTBUF_N; i++) {
        if (distbuf[i] > out) {
            out = distbuf[i];
        }
    }
    return out;
}

