/*
 * dv_stm32f429_uart.c
 *
 *  Created on: 2018/2/27
 *      Author: kcchang
 */
#include "stm32f4xx_hal.h"
#include "dv_stm32f429_uart.h"

/*
 * @file   參考 https://github.com/pca6191/STM32F4-Discovery-Tutorial/tree/master/10.Uart%20Echo%20Data%20Using%20DMA：
 * @note   stm32f429_discovery UART 佔用情況：
 *         UART4:
 *           PA0 (TX) -> 220K pull low, 接 button, 可正常用於 TX.
 *
 *         UART5:
 *           PC12(TX) -> NC
 *            PD2(RX) -> NC (DMA1, stream0, CH4)
 *
 *         UART7:
 *            PF6(RX) -> NC
 *
 *         其餘都已經被週邊佔用，不能正用於 UART 收送。
 */

UART_HandleTypeDef huart4;  //單向對 PC 傳訊息
UART_HandleTypeDef huart5;  //TX/RX 可用

DMA_HandleTypeDef hdma_usart5_rx; //使用 DMA1, Stream0, Ch4

uint8_t send_data=32,receive_data=0;

static void Error_Handler(void);
static void println(char *s);

//=============================================================================
//  中斷 Call back 區域
//=============================================================================
/**
* @brief This function handles DMA1 stream1 global interrupt.
*/
void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart5_rx);
}

/**
* @brief This function handles USART5 global interrupt.
*/
void USART5_IRQHandler(void)
{
    //正常不會呼叫此處，此處用於 DMA 接收完成，萬一有 uart error，
    //再連帶呼叫
    HAL_UART_IRQHandler(&huart5);
}

/*
 * @brief  handle Rx complete event.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==huart5.Instance)
    {
        HAL_UART_Transmit(&huart4,&receive_data,1, 10);
    }
}

//=============================================================================
//  MSP(MCU Specific Package) 實作區域，用以覆寫 STM HAL library 預設程式碼
//=============================================================================
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    if (huart->Instance == UART4)
    {
        /* Peripheral clock enable */
        __HAL_RCC_UART4_CLK_ENABLE();

        /**USART4 GPIO Configuration
         PA0     ------> USART4_TX
         */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (huart->Instance == UART5)
    {
        /* Peripheral clock enable */
        __HAL_RCC_UART5_CLK_ENABLE();

        /**USART5 GPIO Configuration
         PC12     ------> USART5_TX
         PD2     ------> USART5_RX
         */
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_2;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /*Peripheral DMA init*/
        __HAL_RCC_DMA1_CLK_ENABLE() ;

        hdma_usart5_rx.Instance = DMA1_Stream0;
        hdma_usart5_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart5_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart5_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart5_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart5_rx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(huart, hdmarx, hdma_usart5_rx);

        /*Peripheral interrupt init: error 中斷會用到*/
        HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(UART5_IRQn);

        /*
         * @note    經實驗，DMA 中斷在最後一步打開， 用 st-link 觀察
         *          setup 過程才不會一直卡頓。
         */
        HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if (huart->Instance == UART4)
    {
        /* Peripheral clock disable */
        __HAL_RCC_UART4_CLK_DISABLE();

        /**USART4 GPIO Configuration
         PA0     ------> USART4_TX
         */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
    }
    else if (huart->Instance == UART5)
    {
        /* Peripheral clock disable */
        __HAL_RCC_UART5_CLK_DISABLE();

        /**USART5 GPIO Configuration
         PC12     ------> USART5_TX
         PD2     ------> USART5_RX
         */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

        /* Peripheral DMA DeInit*/
        HAL_DMA_DeInit(huart->hdmarx);

        /* Peripheral interrupt DeInit*/
        HAL_NVIC_DisableIRQ(UART5_IRQn);
    }
}

/*
 * @brief    USART4 init function
 */
static void UART4_Init(void)
{
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX; //單向對 PC 傳訊息
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 * @brief    USART5 init function
 */
static void UART5_Init(void)
{
    huart5.Instance = UART5;
    huart5.Init.BaudRate = 115200;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart5) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 * @brief   init uart related GPIO clock.
 */
static void GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {
    }
    /* USER CODE END Error_Handler */
}

/*
 * @brief    簡單列印字串。
 */
static void println(char *s)
{
    char *p = s;
    char n = '\n';

    for(;*p != '\0';p++)
    {
        HAL_UART_Transmit(&huart4, (uint8_t*)p, 1, 10);
    }

    HAL_UART_Transmit(&huart4, (uint8_t*)&n, 1, 10);
}

/*
 * @brief  初始化 uart 之應用
 */
void dv_stm32f429_uart_setup(void)
{
#if TEST_DEBUGPORT_TX  //純粹測試 UART4 對 PC 送訊息是否有通
    GPIO_Init();
    UART4_Init();

    if(0)
    {
        UART2_Init();
        UART3_Init();
    }
#endif

#if TEST_UARTLOOP

    GPIO_Init();
    UART4_Init();
    UART5_Init();

    //HAL_UART_Receive_DMA(&huart5,&receive_data,1);

    println("uart loop setup ok.\n");
#endif

#if TEST_UARTDMA
    GPIO_Init();
    UART4_Init();
    UART5_Init();

    HAL_UART_Receive_DMA(&huart5, &receive_data, 1);

    println("uart DMA setup ok.\n");
#endif
}

/*
 * @brief 本模組應用面入口函式
 */
void dv_stm32f429_uart_process(void)
{
#if TEST_DEBUGPORT_TX
    char str[]="hello\n";

    println(str);
    HAL_Delay(1000);
#endif

#if TEST_UARTLOOP
    HAL_UART_Transmit(&huart5, &send_data, 1, 10);
    send_data++;

    if (HAL_UART_Receive(&huart5, &receive_data, 1, 10) == HAL_OK)
    {
        HAL_UART_Transmit(&huart4, &receive_data, 1, 10);
    }
#endif

#if TEST_UARTDMA
    HAL_UART_Transmit(&huart5, &send_data, 1, 10);
    send_data++;
    HAL_Delay(300);

    //接著 UART5-RX completr callback 會印
#endif
}
