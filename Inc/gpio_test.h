/*
 * gpio_test.h
 *
 *  Created on: Jun 10, 2022
 *      Author: NONAMENEEDED
 */

#ifndef GPIO_TEST_H_
#define GPIO_TEST_H_

#define 	HIGH 			ENABLE
#define 	LOW 			DISABLE
#define 	BTN_PRESSED 	LOW

void delay(void);
void delay_200ms(void);
void led_toggle(void);
void led_button(void);
void led_button_ext(void);
void button_interrupt(void);

#endif /* GPIO_TEST_H_ */
