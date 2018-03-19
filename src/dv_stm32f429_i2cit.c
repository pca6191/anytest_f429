/*
 * dv_stm32f429_i2cit.c
 *
 *  Created on: 2018/3/19
 *      Author: kcchang
 */

/*
 * @file This example describes how to perform I2C data buffer transmission/reception between
 two boards in Polling mode.

 _________________________                        _________________________
 |           ______________|                      |______________           |
 |          | I2C3         |                      |          I2C3|          |
 |          |              |                      |              |          |
 |          |      SCL(PA8)|______________________|(PA8)SCL      |          |
 |          |              |                      |              |          |
 |          |              |                      |              |          |
 |          |              |                      |              |          |
 |          |      SDA(PC9)|______________________|(PC9)SDA      |          |
 |          |              |                      |              |          |
 |          |______________|                      |______________|          |
 |      __                 |                      |        __               |
 |     |__|                |                      |       |__|              |
 |     USER             GND|______________________|GND    USER              |
 |                         |                      |                         |
 |_STM32F429_______________|                      |_STM32F429_______________|

 - 承上：兩塊 discovery 5V - 5V , GND - GND 對接
 - 正常操作：
 1. master 接 usb 通電 (slave 跟著吃電).
 2. 10 秒內，master 按下按鈕，兩板子綠燈亮，表示 master --> slave 成功送收。
 3. 再按下按鈕，兩板子綠燈滅，表示 slave --> master 回送接收成功。
 4. 以上超過 10 秒按鈕、或訊號有誤，則亮紅燈卡死。
 */

#include "global_configs.h"
#include "stm32f4xx_hal.h"
#include "dv_led.h"
#include "dv_stm32f429_i2cit.h"

#if USE_I2CIT_PROCESS

I2C_HandleTypeDef I2cHandle;

/* Buffer used for transmission */
//uint8_t aTxBuffer[] = " ****I2C_TwoBoards communication based on Polling****  ****I2C_TwoBoards communication based on Polling****  ****I2C_TwoBoards communication based on Polling**** ";
uint8_t aTxBuffer[] = "A123b";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

static void Error_Handler(void);
//=============================================================================
//  中斷 Call back 區域
//=============================================================================
/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
/**
 * @brief  This function handles I2C event interrupt request.
 * @param  None
 * @retval None
 * @Note   This function is redefined in "main.h" and related to I2C data transmission
 */
void I2Cx_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&I2cHandle);
}

/**
 * @brief  This function handles I2C error interrupt request.
 * @param  None
 * @retval None
 * @Note   This function is redefined in "main.h" and related to I2C error
 */
void I2Cx_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&I2cHandle);
}

#if MASTER_BOARD
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    if (I2cHandle->Instance == I2C3)
    {
        dv_led_on(led_green);
    }
}
#else
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    if(I2cHandle->Instance == I2C3)
    {
        dv_led_off(led_green);
    }
}
#endif /* MASTER_BOARD */

/**
 * @brief  Rx Transfer completed callback.
 * @param  I2cHandle: I2C handle
 * @note   This example shows a simple way to report end of IT Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
#if MASTER_BOARD
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    if (I2cHandle->Instance == I2C3)
    {
        dv_led_off(led_green);
    }
}
#else
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    if(I2cHandle->Instance == I2C3)
    {
        dv_led_on(led_green);
    }
}
#endif /* MASTER_BOARD */

/**
 * @brief  I2C error callbacks.
 * @param  I2cHandle: I2C handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
    if (I2cHandle->Instance == I2C3)
    {
        uint8_t cnt = 10;
        while (cnt--)
        {
            HAL_Delay(300);
            dv_led_on(led_red);
            HAL_Delay(300);
            dv_led_off(led_red);
        }
    }
}

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
    GPIO_InitTypeDef GPIO_InitStruct;

    if (hi2c == NULL)
    {
    }

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    I2Cx_SCL_GPIO_CLK_ENABLE();
    I2Cx_SDA_GPIO_CLK_ENABLE();
    /* Enable I2Cx clock */
    I2Cx_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* I2C TX GPIO pin configuration  */
    GPIO_InitStruct.Pin = I2Cx_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = I2Cx_SCL_AF;

    HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);

    /* I2C RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = I2Cx_SDA_PIN;
    GPIO_InitStruct.Alternate = I2Cx_SDA_AF;

    HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);

    /*##-3- Configure the NVIC for I2C #########################################*/
    /* NVIC for I2C1 */
    HAL_NVIC_SetPriority(I2Cx_ER_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(I2Cx_ER_IRQn);
    HAL_NVIC_SetPriority(I2Cx_EV_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(I2Cx_EV_IRQn);
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

    /*##-3- Disable the NVIC for I2C ###########################################*/
    HAL_NVIC_DisableIRQ(I2Cx_ER_IRQn);
    HAL_NVIC_DisableIRQ(I2Cx_EV_IRQn);
}
//======================================================================================
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

/**
 * @brief  Returns the selected Button state.
 * @param  Button: Specifies the Button to be checked.
 *   This parameter should be: BUTTON_KEY
 * @retval The Button GPIO pin value.
 */
static bool is_usrbtn_pressed()
{
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
    /* Turn LED4 on */
    dv_led_on(led_red);
    while (1)
    {
    }
}

/*
 * @brief  初始化 i2c 之裝置
 */
void dv_stm32f429_i2c_init(void)
{
    /*##-1- Configure the I2C peripheral #######################################*/
    I2cHandle.Instance = I2Cx;

    I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle.Init.ClockSpeed = 100000;
    I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    I2cHandle.Init.OwnAddress1 = I2C_ADDRESS;
    I2cHandle.Init.OwnAddress2 = 0;

    //執行此步驟，確保 i2c 沒有被咬住，往下才不會觸發 busy 導致第一次 transmit 失敗
    I2Cx_FORCE_RESET();
    I2Cx_RELEASE_RESET();

    if (HAL_I2C_Init(&I2cHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
}

/*
 * @brief  初始化 i2c 之應用
 */
void dv_stm32f429_i2cit_setup(void)
{
    /* init i2c device */
    dv_stm32f429_i2c_init();

    /* Configure USER Button PA0 */
    /* init the input button PA0; LED3 PG13 */
    GPIO_InitTypeDef type_A =
    {
    GPIO_PIN_0, GPIO_MODE_INPUT, GPIO_PULLDOWN, GPIO_SPEED_LOW, 0
    };

    /* enable user button gpio */
    __GPIOA_CLK_ENABLE()
    ;

    HAL_GPIO_Init(GPIOA, &type_A);
}

/*
 * @brief 本模組應用面入口函式
 */
void dv_stm32f429_i2cit_process(void)
{
#if MASTER_BOARD
    //-- 沒有按下按鈕，卡這裡等 ---------------------------------------
    while (is_usrbtn_pressed() == false) //while(not pressed)
    {
    }

    //-- 等按鈕按下 -------------------------------------------------
    while (is_usrbtn_pressed() == true) //while(pressed)
    {
    }

    //-- 登記要中斷發送資料 --------------
    while (HAL_I2C_Master_Transmit_IT(&I2cHandle, (uint16_t) I2C_ADDRESS, (uint8_t*) aTxBuffer, TXBUFFERSIZE) != HAL_OK)
    {
        //如果 slave 沒回應的 AF error，則繼續送; 其他 error，就 error handle 亮紅燈
        if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {
            Error_Handler();
        }
    } //-- 送成功，callback 亮綠燈 --

    //-- 確認送完，好繼續下一步接收 --
    while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
    {
    }

    //-- 再按一次，收資料 --
    //-- 沒有按下按鈕，卡這裡等 ---------------------------------------
    while (is_usrbtn_pressed() == false)
    {
    }

    //-- 等按鈕按下 -------------------------------------------------
    while (is_usrbtn_pressed() == true)
    {
    }

    //-- 登記要接收資料 ------------------
    while (HAL_I2C_Master_Receive_IT(&I2cHandle, (uint16_t) I2C_ADDRESS, (uint8_t *) aRxBuffer, RXBUFFERSIZE) != HAL_OK)
    {
        //-- //如果 slave 沒回應的 AF error，則繼續送; 其他 error，就 error handle 亮紅燈--
        if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {
            Error_Handler();
        }
    } //-- 收成功，callback 關綠燈 ------------------------------------------------------

#else

    //=========================================================================
    // Slave 接收訊息並且回傳
    //=========================================================================

    //-- 登記要中斷收資料 ---------------------------
    if(HAL_I2C_Slave_Receive_IT(&I2cHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
    {
        /* Transfer error in reception process */
        Error_Handler();
    } //-- 收到訊息亮綠燈 --

    //-- 確保收完，接著送出資料 --
    while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
    {
    }

    //-- 登記要中斷送資料 ---------------------------
    if(HAL_I2C_Slave_Transmit_IT(&I2cHandle, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK)
    {
        /* Transfer error in transmission process */
        Error_Handler();
    } // 送成功，關綠燈 --

#endif //end MASTER_BOARD

    //--確定做完上述動作 --
    while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
    {
    }

    //-- 最後比較收、送資料，不一致就 error 燈 (紅燈) 卡死 ------------------------
    if (Buffercmp((uint8_t*) aTxBuffer, (uint8_t*) aRxBuffer, RXBUFFERSIZE))
    {
        /* Processing Error */
        Error_Handler();
    }
}
#endif
