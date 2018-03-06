/*
 * dv_led.h
 *
 *  Created on: 2018/3/6
 *      Author: kcchang
 */

#ifndef DV_LED_H_
#define DV_LED_H_

typedef enum
{
    led_green,
    led_red,

    led_final
} dv_led_e;

void dv_led_init(void);
void dv_led_on(dv_led_e name);
void dv_led_off(dv_led_e name);


#endif /* DV_LED_H_ */
