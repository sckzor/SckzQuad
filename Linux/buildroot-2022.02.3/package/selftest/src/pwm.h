#include <linux/types.h>

#ifndef _PWM_H
#define _PWM_H

static const __u8 MODE1 = 0x00;
static const __u8 MODE2 = 0x01;
static const __u8 PRESCALE = 0xFE;
static const __u8 LED0_ON_L = 0x06;
static const __u8 LED0_ON_H = 0x07;
static const __u8 LED0_OFF_L = 0x08;
static const __u8 LED0_OFF_H = 0x09;
static const __u8 ALL_LED_ON_L = 0xFA;
static const __u8 ALL_LED_ON_H = 0xFB;
static const __u8 ALL_LED_OFF_L = 0xFC;
static const __u8 ALL_LED_OFF_H = 0xFD;

static const __u8 RESTART = 0x80;
static const __u8 SLEEP = 0x10;
static const __u8 ALLCALL = 0x01;
static const __u8 INVRT = 0x10;
static const __u8 OUTDRV = 0x04;

int setup_pwm(int);

void set_pwm_frequency(int, double);
void set_pwm(int, int, int, int);
void set_all_pwm(int, int, int);

#endif
