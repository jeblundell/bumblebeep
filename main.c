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

#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/pio.h>
#include <hardware/timer.h>
#include "generated/kitronik.pio.h"
#include "kitronik.h"

/*** Main *********************************************************************/
int main () {
    float dist;
    uint16_t speed;
    uint8_t state;
    uint64_t t_state, t_status;

    stdio_init_all ();

    us_init (PIN_F_TRIGGER, PIN_F_ECHO);
    motor_init (PIN_L_FWD, PIN_L_REV);
    motor_init (PIN_R_FWD, PIN_R_REV);
    led_init ();
    buzzer_init ();
    distbuf_init ();

    state = STATE_INIT;
    t_state = time_us_64();
    t_status = 0;

    for (;;) {
        dist = us_cm (PIN_F_TRIGGER, PIN_F_ECHO, 30000);
        if (time_us_64() - distbuf_last > DISTBUF_US && dist >= 0) {
            distbuf_push (dist);
        }
        dist = distbuf_mean ();
        if (dist < 0) {
            led_write (0, 127, 0, 0);
            led_write (1, 127, 0, 0);
            speed = SPEED_SLOW;
        }  else if (dist <= DIST_STOP || (time_us_64() - distbuf_last) > DISTBUF_STALE) {
            led_write (0, 255, 0, 0);
            led_write (1, 255, 0, 0);
            speed = SPEED_SLOW;
        } else if (dist > DIST_FAST) {
            led_write (0, 0, 127, 0);
            led_write (1, 0, 127, 0);
            speed = SPEED_FAST;
        } else {
            led_write (0, 0, 0, 127);
            led_write (1, 0, 0, 127);
            speed = (float)SPEED_SLOW + ((dist - (float)DIST_STOP)/(float)(DIST_FAST - DIST_STOP)) * (float)(SPEED_FAST - SPEED_SLOW);
        }

        if (state == STATE_INIT) {
            led_write (2, 127, 127, 0);
            led_write (3, 127, 127, 0);
            if ((time_us_64() - t_state > STATECHANGE_US) && (time_us_64() - distbuf_last) < DISTBUF_STALE && time_us_64() > 1e6) {
                if (dist > DIST_FAST) {
                    state = STATE_FORWARD;
                    t_state = time_us_64();
                } else if (dist > DIST_STOP) {
                    state = STATE_PROBE_LEFT;
                    t_state = time_us_64();
                } else if (dist <= DIST_STOP) {
                    state = STATE_BACKUP;
                    t_state = time_us_64();
                }
            }
        } else if (state == STATE_FORWARD) {
            led_write (2, 0, 127, 0);
            led_write (3, 0, 127, 0);
            if (dist < DIST_STOP || (time_us_64() - distbuf_last) > DISTBUF_STALE) {
                state = STATE_PROBE_LEFT;
                t_state = time_us_64();
                motor_set (PIN_L_FWD, PIN_L_REV, 0);
                motor_set (PIN_R_FWD, PIN_R_REV, 0);
                buzzer_freq (BUZZER_STOP, BUZZER_MS);
            } else if ((time_us_64() - t_state > STATECHANGE_US && distbuf_max() - distbuf_min() < DISTDIFF_MIN) || (time_us_64() - t_state > FORWARD_MAX)) {
                state = STATE_PROBE_LEFT;
                t_state = time_us_64();
                motor_set (PIN_L_FWD, PIN_L_REV, 0);
                motor_set (PIN_R_FWD, PIN_R_REV, 0);
            } else {
                motor_set (PIN_L_FWD, PIN_L_REV, speed);
                motor_set (PIN_R_FWD, PIN_R_REV, speed);
            } 
        } else if (state == STATE_PROBE_LEFT) {
            led_write (1, 0, 0, 0);
            led_write (2, 0, 0, 0);
            led_write (3, 255, 255, 0);
            if (distbuf_min() > DIST_STOP && distbuf_mean() > DIST_FAST && time_us_64() - t_state > STATECHANGE_US) {
                state = STATE_INIT;
                t_state = time_us_64();
                motor_set (PIN_L_FWD, PIN_L_REV, 0);
                motor_set (PIN_R_FWD, PIN_R_REV, 0);
            } else if (time_us_64() - t_state > PROBECHANGE_US) {
                state = STATE_PROBE_RIGHT;
                t_state = time_us_64();
                motor_set (PIN_L_FWD, PIN_L_REV, 0);
                motor_set (PIN_R_FWD, PIN_R_REV, 0);
            } else {
                motor_set (PIN_L_FWD, PIN_L_REV, speed);
                motor_set (PIN_R_FWD, PIN_R_REV, -speed);
            }
        } else if (state == STATE_PROBE_RIGHT) {
            led_write (0, 0, 0, 0);
            led_write (2, 255, 255, 0);
            led_write (3, 0, 0, 0);
            if (distbuf_min() > DIST_STOP && distbuf_mean() > DIST_FAST && time_us_64() - t_state > STATECHANGE_US) {
                state = STATE_INIT;
                t_state = time_us_64();
                motor_set (PIN_L_FWD, PIN_L_REV, 0);
                motor_set (PIN_R_FWD, PIN_R_REV, 0);
            } else if (time_us_64() - t_state > PROBECHANGE_US*2) {
                state = STATE_BACKUP;
                t_state = time_us_64();
                motor_set (PIN_L_FWD, PIN_L_REV, 0);
                motor_set (PIN_R_FWD, PIN_R_REV, 0);
                buzzer_freq (BUZZER_STOP, BUZZER_MS);
            } else {
                motor_set (PIN_L_FWD, PIN_L_REV, -speed);
                motor_set (PIN_R_FWD, PIN_R_REV, speed);
            }
        } else if (state == STATE_BACKUP) { 
            led_write (2, 255, 255, 255);
            led_write (3, 255, 255, 255);
            if ((distbuf_mean() > DIST_STOP && time_us_64() - t_state > PROBECHANGE_US*2) ||
                    (distbuf_min() > DIST_FAST && time_us_64() - t_state > PROBECHANGE_US)) {
                state = STATE_INIT;
                t_state = time_us_64();
                motor_set (PIN_L_FWD, PIN_L_REV, 0);
                motor_set (PIN_R_FWD, PIN_R_REV, 0);
            } else if (time_us_64() - t_state > PROBECHANGE_US*3) {
                state = STATE_PROBE_LEFT;
                t_state = time_us_64();
                motor_set (PIN_L_FWD, PIN_L_REV, 0);
                motor_set (PIN_R_FWD, PIN_R_REV, 0);
            } else {
                motor_set (PIN_L_FWD, PIN_L_REV, -speed);
                motor_set (PIN_R_FWD, PIN_R_REV, -speed);
            }
        } else {
            led_write (2, 255, 0, 0);
            led_write (3, 255, 0, 0);
        }

        led_commit ();

        if (time_us_64() - t_status > 100e3) {
            printf ("State %d (%4lldms) speed=%d distbuf=%3.1f/%3.1f/%3.1f delta=%3.1f distbuf_stale(ms)=%4lld\n", state, (time_us_64() - t_state)/1000, speed, distbuf_min(), distbuf_mean(), distbuf_max(), distbuf_max() - distbuf_min(), (time_us_64() - distbuf_last)/1000);
            t_status = time_us_64();
        }
        sleep_ms (1);
    }
    return 0;
}
