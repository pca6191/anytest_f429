/*
 * global_configs.h
 *
 *  Created on: 2018年2月2日
 *      Author: pca
 */

#ifndef MAIN_H_
#define MAIN_H_

#define USE_BLINKLED_PROCESS    0
#define USE_LCD_PROCESS         0
#define USE_I2C_PROCESS         0
#define USE_I2CDMA_PROCESS      0 //尚未成功
#define USE_I2CIT_PROCESS       1
#define USE_UART_PROCESS        0
#define USE_SPI_PROCESS         0


typedef enum
{
    false,
    true
} bool;

#endif /* MAIN_H_ */
