/*
 * dv_stm32f429_spi.h
 *
 *  Created on: 2018年2月28日
 *      Author: pca
 */

#ifndef DV_STM32F429_SPI_H_
#define DV_STM32F429_SPI_H_

#define TEST_MASTER_OUTPUT  0   //測試 master 單送波形對否
#define TEST_DMA_LOOP       1   //測試 SPI3 --> SPI4 DMA RX

void dv_stm32f429_SPI_setup();
void dv_stm32f429_spi_process();

#endif /* DV_STM32F429_SPI_H_ */
