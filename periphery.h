/*
 * periphery.h
 *
 *  Created on: 6 но€б. 2017 г.
 *      Author: Andrew
 */

#ifndef PERIPHERY_H_
#define PERIPHERY_H_

void init_uarts();
void setup_clock();
void setup_timers();

static inline void set_uart_mode() {
	P1SEL |= BIT3;
}

static inline void pin_down() {
	P1OUT &= ~BIT3;
	P1SEL &= ~BIT3;
}

static inline void green_led_on() {
	P2OUT |= BIT6;
}

static inline void green_led_off() {
	P2OUT &= ~BIT6;
}

static inline void yellow_led_on() {
	P2OUT |= BIT5;
}

static inline void yellow_led_off() {
	P2OUT &= ~BIT5;
}

static inline void relay_on() {
	P1OUT |= BIT6;
}

static inline void relay_off() {
	P1OUT &= ~BIT6;
}

static inline int get_green_state() {
	return P2OUT | BIT6;
}

static inline void uca2_read() {
	P2OUT &= ~BIT4;
}

static inline void uca2_write() {
	P2OUT |= BIT4;
}
#endif /* PERIPHERY_H_ */
