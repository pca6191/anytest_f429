/*
 * dv_stm32f429_i2cit.h
 *
 *  Created on: 2018/3/19
 *      Author: kcchang
 */

#ifndef DV_STM32F429_I2CIT_H_
#define DV_STM32F429_I2CIT_H_

#include "global_configs.h"


#if USE_I2CIT_PROCESS
#define MASTER_BOARD       0

#define I2C_ADDRESS        0x30 //ex. 0x30, LA 從左到右 address bit 應為
                                 // S 0011 000(w0/r1)

/* Definition for I2Cx clock resources */
#define I2Cx                             I2C3
#define I2Cx_CLK_ENABLE()                __HAL_RCC_I2C3_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOC_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()

#define I2Cx_FORCE_RESET()               __HAL_RCC_I2C3_FORCE_RESET()
#define I2Cx_RELEASE_RESET()             __HAL_RCC_I2C3_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_8
#define I2Cx_SCL_GPIO_PORT              GPIOA
#define I2Cx_SCL_AF                     GPIO_AF4_I2C3
#define I2Cx_SDA_PIN                    GPIO_PIN_9
#define I2Cx_SDA_GPIO_PORT              GPIOC
#define I2Cx_SDA_AF                     GPIO_AF4_I2C3

/* Definition for I2Cx's NVIC */
#define I2Cx_EV_IRQn                    I2C3_EV_IRQn
#define I2Cx_EV_IRQHandler              I2C3_EV_IRQHandler
#define I2Cx_ER_IRQn                    I2C3_ER_IRQn
#define I2Cx_ER_IRQHandler              I2C3_ER_IRQHandler

/* Size of Transmission buffer */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

void dv_stm32f429_i2cit_setup(void);
void dv_stm32f429_i2cit_process(void);

#endif

#endif /* DV_STM32F429_I2CIT_H_ */
