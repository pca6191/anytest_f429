/*
 * dv_led.c
 *
 *  Created on: 2018/3/6
 *      Author: kcchang
 */
#include "stm32f4xx_hal.h"
#include "dv_led.h"

/*
 * @file    discovery429 led 有兩顆：Green (PG13) / RED (PG14)
 */

/*
 * @brief   initialize the gpio of leds.
 */
void dv_led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //init PG clock
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*Configure GPIO pin : PG13/14 */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);
}

/*
 * @bried turn the led.
 */
void dv_led_on(dv_led_e name)
{
    switch (name)
    {
        case led_green:
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
            break;

        case led_red:
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
            break;

        default:
            break;
    }
}

/*
 * @brief  turn off the led.
 */
void dv_led_off(dv_led_e name)
{
    switch (name)
    {
        case led_green:
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
            break;

        case led_red:
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);
            break;

        default:
            break;
    }
}

