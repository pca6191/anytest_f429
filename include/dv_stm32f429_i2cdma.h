/*
 * dv_stm32f429_i2cdma.h
 *
 *  Created on: 2018年3月15日
 *      Author: pca
 */

#ifndef DV_STM32F429_I2CDMA_H_
#define DV_STM32F429_I2CDMA_H_

#include "global_configs.h"

#if USE_I2CDMA_PROCESS
#define MASTER_BOARD       1

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

/* Definition for I2Cx's DMA */
#define DMAx_CLK_ENABLE()               __DMA1_CLK_ENABLE()
#define I2Cx_TX_DMA_CHANNEL             DMA_CHANNEL_3
#define I2Cx_TX_DMA_STREAM              DMA1_Stream4
#define I2Cx_RX_DMA_CHANNEL             DMA_CHANNEL_3
#define I2Cx_RX_DMA_STREAM              DMA1_Stream2

/* Definition for I2Cx's NVIC */
#define I2Cx_DMA_TX_IRQn                DMA1_Stream4_IRQn
#define I2Cx_DMA_RX_IRQn                DMA1_Stream2_IRQn
#define I2Cx_DMA_TX_IRQHandler          DMA1_Stream4_IRQHandler
#define I2Cx_DMA_RX_IRQHandler          DMA1_Stream2_IRQHandler


/* Size of Transmission buffer */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

void dv_stm32f429_i2cdma_setup(void);
void dv_stm32f429_i2cdma_process(void);

#endif

#endif /* DV_STM32F429_I2CDMA_H_ */
