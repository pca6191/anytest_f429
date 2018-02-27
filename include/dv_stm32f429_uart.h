/*
 * dv_stm32f429_uart.h
 *
 *  Created on: 2018/2/27
 *      Author: kcchang
 */

#ifndef DV_STM32F429_UART_H_
#define DV_STM32F429_UART_H_

#define TEST_DEBUGPORT_TX  0 //1: 純測試對 PC 發送訊息
#define TEST_UARTLOOP      0 //1: 測試 UART5 TX 送給 RX，是否會通
#define TEST_UARTDMA       1 //1: 測試 UART5 TX 送給 RX DMA，是否會通


void dv_stm32f429_uart_setup(void);
void dv_stm32f429_uart_process(void);

#endif /* DV_STM32F429_UART_H_ */
