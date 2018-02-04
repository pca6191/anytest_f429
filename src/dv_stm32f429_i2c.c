/*
 * dv_stm32f429_i2c.c
 *
 *  Created on: 2018年2月4日
 *      Author: pca
 */
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery.h"
#include "dv_stm32f429_i2c.h"

I2C_HandleTypeDef I2cHandle;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = " ****I2C_TwoBoards communication based on Polling****  ****I2C_TwoBoards communication based on Polling****  ****I2C_TwoBoards communication based on Polling**** ";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

static void Error_Handler(void);
//=============================================================================
//  中斷 Call back 區域
//=============================================================================

//=============================================================================
//  MSP(MCU Specific Package) 實作區域，用以覆寫 STM HAL library 預設程式碼
//=============================================================================

/**
  * @brief I2C MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - DMA configuration for transmission request by peripheral
  *           - NVIC configuration for DMA interrupt request enable
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

    if (hi2c == NULL)
    {
    }

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  I2Cx_SCL_GPIO_CLK_ENABLE();
  I2Cx_SDA_GPIO_CLK_ENABLE();
  /* Enable I2C1 clock */
  I2Cx_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* I2C TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = I2Cx_SCL_AF;

  HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);

  /* I2C RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = I2Cx_SDA_PIN;
  GPIO_InitStruct.Alternate = I2Cx_SDA_AF;

  HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief I2C MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
    }

    /*##-1- Reset peripherals ##################################################*/
    I2Cx_FORCE_RESET();
    I2Cx_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* Configure I2C Tx as alternate function  */
    HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
    /* Configure I2C Rx as alternate function  */
    HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);
}
//======================================================================================
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED4 on */
//  BSP_LED_On(LED4);
//  while(1)
//  {
//  }
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if ((*pBuffer1) != *pBuffer2)
        {
            return BufferLength;
        }
        pBuffer1++;
        pBuffer2++;
    }

    return 0;
}

/*
 * @brief  初始化 i2c 之裝置
 */
void dv_stm32f429_i2c_init(void)
{
    /*##-1- Configure the I2C peripheral #######################################*/
    I2cHandle.Instance = I2Cx;

    I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
    I2cHandle.Init.ClockSpeed = 400000;
    I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
    I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    I2cHandle.Init.OwnAddress1 = I2C_ADDRESS;
    I2cHandle.Init.OwnAddress2 = 0xFE;

    if (HAL_I2C_Init(&I2cHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_KEY
  * @retval The Button GPIO pin value.
  */
static uint32_t user_btn_state(Button_TypeDef button)
{
    switch(button)
    {
        case BUTTON_KEY:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
            break;

        default:
            break;
    }

    return 0;
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED3
  *     @arg LED4
  */
static void led_on(Led_TypeDef led)
{
    switch(led)
    {
        case LED3:
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
            break;

        case LED4:
        default:
            break;
    }

}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED3
  *     @arg LED4
  */
static void led_off(Led_TypeDef led)
{
    switch(led)
    {
        case LED3:
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
            break;

        case LED4:
        default:
            break;
    }

}

/*
 * @brief  初始化 i2c 之應用
 */
void dv_stm32f429_i2c_setup(void)
{
    /* init i2c device */
    dv_stm32f429_i2c_init();

    /* Configure USER Button PA0 */
    /* init the input button PA0; LED3 PG13 */
    GPIO_InitTypeDef type_A =
    {
    GPIO_PIN_0, GPIO_MODE_INPUT, GPIO_PULLDOWN, GPIO_SPEED_LOW, 0
    };
    GPIO_InitTypeDef type_G =
    {
    GPIO_PIN_13, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FAST, 0
    };

    __GPIOA_CLK_ENABLE()
    ;
    __GPIOG_CLK_ENABLE()
    ;
    HAL_GPIO_Init(GPIOA, &type_A);
    HAL_GPIO_Init(GPIOG, &type_G);
}

/*
 * @brief 本模組應用面入口函式
 */
void dv_stm32f429_i2c_process(void)
{
#ifdef MASTER_BOARD
    /* Wait for USER Button press before starting the Communication */
    while (user_btn_state(BUTTON_KEY) != 1)
    {
    }

    /* Wait for USER Button release before starting the Communication */
    while (user_btn_state(BUTTON_KEY) != 0)
    {
    }

    /* The board sends the message and expects to receive it back */

    /*##-2- Start the transmission process #####################################*/
    /* While the I2C in reception process, user can transmit data through
     "aTxBuffer" buffer */
    /* Timeout is set to 10S */
    while (HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t) I2C_ADDRESS,
            (uint8_t*) aTxBuffer, TXBUFFERSIZE, 10000) != HAL_OK)
    {
        /* Error_Handler() function is called when Timeout error occurs.
         When Acknowledge failure occurs (Slave don't acknowledge it's address)
         Master restarts communication */
        if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {
            Error_Handler();
        }
    }

    /* Turn LED3 on: Transfer in Transmission process is correct */
    led_on(LED3);

    /* Wait for USER Button press before starting the Communication */
    while (user_btn_state(BUTTON_KEY) != 1)
    {
    }

    /* Wait for USER Button release before starting the Communication */
    while (user_btn_state(BUTTON_KEY) != 0)
    {
    }

    /*##-3- Put I2C peripheral in reception process ############################*/
    /* Timeout is set to 10S */
    while (HAL_I2C_Master_Receive(&I2cHandle, (uint16_t) I2C_ADDRESS,
            (uint8_t *) aRxBuffer, RXBUFFERSIZE, 10000) != HAL_OK)
    {
        /* Error_Handler() function is called when Timeout error occurs.
         When Acknowledge failure occurs (Slave don't acknowledge it's address)
         Master restarts communication */
        if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {
            Error_Handler();
        }
    }

    /* Turn LED3 off: Transfer in reception process is correct */
    led_off(LED3);

#else

    /* The board receives the message and sends it back */

    /*##-2- Put I2C peripheral in reception process ############################*/
    /* Timeout is set to 10S  */
    if(HAL_I2C_Slave_Receive(&I2cHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE, 10000) != HAL_OK)
    {
        /* Transfer error in reception process */
        Error_Handler();
    }

    /* Turn LED3 on: Transfer in reception process is correct */
    led_on(LED3);

    /*##-3- Start the transmission process #####################################*/
    /* While the I2C in reception process, user can transmit data through
     "aTxBuffer" buffer */
    /* Timeout is set to 10S */
    if(HAL_I2C_Slave_Transmit(&I2cHandle, (uint8_t*)aTxBuffer, TXBUFFERSIZE, 10000)!= HAL_OK)
    {
        /* Transfer error in transmission process */
        Error_Handler();
    }

    /* Turn LED3 off: Transfer in transmission process is correct */
    led_offLED3);

#endif /* MASTER_BOARD */

    /*##-4- Compare the sent and received buffers ##############################*/
    if (Buffercmp((uint8_t*) aTxBuffer, (uint8_t*) aRxBuffer, RXBUFFERSIZE))
    {
        /* Processing Error */
        Error_Handler();
    }
}

