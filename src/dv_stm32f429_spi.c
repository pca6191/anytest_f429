/*
 * dv_stm32f429_spi.c
 *
 *  Created on: 2018年2月28日
 *      Author: pca
 */
#include "stm32f4xx_hal.h"
#include "dv_stm32f429_spi.h"


/*
 * @file ref: https://github.com/pca6191/STM32F4-Discovery-Tutorial/tree/master/11.SPI%20Send%20_Receive%20Data%20Using%20DMA
 */
/*********************************************************************************************
In this example I will use SPI3 run in mode Master(NSS pin control by software) to send data.
SPI4 run in mode Slave (NSS pin control by harware) receive data using DMA.
PC11 (SPI3 MISO) connect to PE5(SPI4 MISO).
PC12 (SPI3 MOSI) connect to PE6(SPI4 MOSI).
PC10 (SPI3 SCK) connect to PE2 (SPI4 SCK).
PC8 (SPI3 NSS pin control by sofware) connect to PE4(SPI4 NSS pin control by hardware).
**********************************************************************************************/

#define spi_enable      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)
#define spi_disable   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)

SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_rx;

static void Error_Handler(void);
static void GPIO_Init(void);
static void DMA_Init(void);
static void SPI3_Init(void);
static void SPI4_Init(void);

static uint8_t send_data=32,receive_data=0;

//=============================================================================
//  中斷 Call back 區域
//=============================================================================
/**
* @brief This function handles DMA1 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi4_rx);
}

/**
* @brief This function handles SPI3 global interrupt.
*/
void SPI3_IRQHandler(void)
{
  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
}

/**
* @brief This function handles SPI4 global interrupt.
*/
void SPI4_IRQHandler(void)
{
  /* USER CODE END SPI4_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi4);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance==hspi3.Instance)
    {
        //master 中斷送完，關閉 spi device. (CS high)..(active low)
        spi_disable;
    }
    else if(hspi->Instance==hspi4.Instance)
    {
        //顯示 receive_data
    }
}

//=============================================================================
//  MSP(MCU Specific Package) 實作區域，用以覆寫 STM HAL library 預設程式碼
//=============================================================================
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    if (hspi->Instance == SPI3)
    {
        /* Peripheral clock enable */
        __HAL_RCC_SPI3_CLK_ENABLE()
        ;

        /**SPI3 GPIO Configuration
         PC11     ------> SPI3_MISO
         PC12     ------> SPI3_MOSI
         PC10     ------> SPI3_SCK
         */
        GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 |GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* Peripheral interrupt init */
        HAL_NVIC_SetPriority(SPI3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(SPI3_IRQn);
    }
    else if (hspi->Instance == SPI4)
    {
        /* Peripheral clock enable */
        __HAL_RCC_SPI4_CLK_ENABLE()
        ;

        /**SPI4 GPIO Configuration
         PE4     ------> SPI4_NSS
         PE2     ------> SPI3_SCK
         PE5     ------> SPI3_MISO
         PE6     ------> SPI3_MOSI
         */
        GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* Peripheral DMA init*/
        __HAL_RCC_DMA1_CLK_ENABLE()
         ;
        hdma_spi4_rx.Instance = DMA2_Stream0;
        hdma_spi4_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_spi4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_spi4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi4_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi4_rx.Init.Mode = DMA_CIRCULAR;
        hdma_spi4_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_spi4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

        if (HAL_DMA_Init(&hdma_spi4_rx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(hspi, hdmarx, hdma_spi4_rx);

        /* Peripheral interrupt init */
        HAL_NVIC_SetPriority(SPI4_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(SPI4_IRQn);
    }
}


void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
     PC11     ------> SPI3_MISO
     PC12     ------> SPI3_MOSI
     PC10     ------> SPI3_SCK
     */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_10);

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(SPI3_IRQn);
  }
  else if(hspi->Instance==SPI4)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SPI4_CLK_DISABLE();

    /**SPI4 GPIO Configuration
     PE4     ------> SPI4_NSS
     PE2     ------> SPI3_SCK
     PE5     ------> SPI3_MISO
     PE6     ------> SPI3_MOSI
     */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_4|GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hspi->hdmarx);

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(SPI4_IRQn);
  }
}

/* SPI3 init function */
static void SPI3_Init(void)
{
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        Error_Handler();
    }
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* SPI4 init function */
static void SPI4_Init(void)
{
    hspi4.Instance = SPI4;
    hspi4.Init.Mode = SPI_MODE_SLAVE;
    hspi4.Init.Direction = SPI_DIRECTION_2LINES;
    hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi4.Init.NSS = SPI_NSS_HARD_INPUT;
    hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi4.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * Enable DMA controller clock
  */
static void DMA_Init(void)
{
//    /* DMA controller clock enable */
//    __HAL_RCC_DMA1_CLK_ENABLE()
//    ;
//
//    /* DMA interrupt init */
//    /* DMA2_Stream0_IRQn interrupt configuration */
//    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE()
    ;
    __HAL_RCC_GPIOE_CLK_ENABLE()
    ;

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

    /*Configure GPIO pin : PC8 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {
    }
    /* USER CODE END Error_Handler */
}

/*
 * @brief  初始化 uart 之應用
 */
void dv_stm32f429_SPI_setup()
{
    GPIO_Init();
    DMA_Init();
    SPI3_Init();
    SPI4_Init();

    HAL_SPI_Receive_DMA(&hspi4, &receive_data, 1);
}

/*
 * @brief 本模組應用面入口函式
 */
void dv_stm32f429_spi_process()
{
    spi_enable;
    HAL_SPI_Transmit_IT(&hspi3, &send_data, 1);
    HAL_Delay(300);
    send_data++;
}
