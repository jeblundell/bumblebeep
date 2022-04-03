
#ifndef _KITRONIK_H_
#define _KITRONIK_H_

#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/pio.h>
#include <hardware/timer.h>
#include "generated/kitronik.pio.h"

#define PIN_L_FWD 20
#define PIN_L_REV 19
#define PIN_R_FWD 6
#define PIN_R_REV 7
#define SPEED 65535

#define PIN_LED 18
#define PWM_DIV 1250
#define LED_FREQ 8e6
#define LED_N 4

#define PIN_F_TRIGGER 14
#define PIN_F_ECHO    15

#define PIN_R_TRIGGER 3
#define PIN_R_ECHO    2

#define PIN_BUZZER 16

#define STATE_INIT 0
#define STATE_FORWARD 1
#define STATE_PROBE_LEFT 2
#define STATE_PROBE_RIGHT 3
#define STATE_BACKUP 4
#define STATECHANGE_US 100e3
#define PROBECHANGE_US 250e3

#define DIST_FAST 80
#define DIST_STOP 20
#define DISTDIFF_MIN 20
#define FORWARD_MAX 3e6
#define DISTBUF_N 100
#define DISTBUF_US 200
#define DISTBUF_STALE 500e3

#define SPEED_FAST 25000
#define SPEED_SLOW 10000

#define BUZZER_GO   500
#define BUZZER_STOP 400
#define BUZZER_MS 50

extern PIO led_pio;
extern int led_sm;
extern uint32_t led_buf[LED_N];

extern float distbuf[DISTBUF_N];
extern uint64_t distbuf_last;
extern uint8_t distbuf_n;

void motor_init (uint fwd, uint rev);
void motor_set (uint fwd, uint rev, int32_t f_speed);
void led_init ();
void led_write (uint8_t n, uint8_t r, uint8_t g, uint8_t b);
void led_commit ();
void us_init (uint trigger, uint echo);
float us_cm (uint trigger, uint echo, uint32_t timeout);
void buzzer_init ();
void buzzer_freq (uint64_t freq, uint ms);
void distbuf_init ();
void distbuf_push (float dist);
float distbuf_min ();
float distbuf_mean ();
float distbuf_max ();

int main ();

#endif
