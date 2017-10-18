/******************************************************************************
*                                                                             *
*                            Module specification                             *
*                                                                             *
*                                Copyright by                                 *
*                                                                             *
*                              HOYEN TECH Co., Ltd                            *
*                                                                             *
*******************************************************************************
Name             :  IQS316_driver.c
Description      :  STM32M0 specific functions for IQS316 I2C Firmware library
*******************************************************************************/

#include "iqs316_driver.h"
#include "stdio.h"
/*****************************************************************************
//
//! Initialise
//!
//! Initializes the STM32F0 I2C
//!
//! \param None
//!
//! \return None
// 
//*****************************************************************************/
void IQS316_Init(void)
{
    Comms_init();
    //
    // Add delay here to allow IQS316 to startup
    // According to datasheet first comms is available roughly 16ms
    // after MCLR is released
    //
    delay_msSysTick(15);

    // Place other functions responsible for hardware initialization here.

    IQS316_Settings();
}

/*****************************************************************************
//
//! I2C Initialise
//!
//! Initializes the I2C module on the PIC18F4550 
//! Note that the SSPADD acts as a counter to determine the I2C frequency. A   
//! smaller value will increase the frequency.
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************/
void Comms_init()
{
  I2C_InitTypeDef  I2C_InitStructure;
  
  HW_TP_LowLevel_Init();
  
  /* I2C configuration */
  /* sEE_I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_Timing = TP_I2C_TIMING;
  
  /* Apply sEE_I2C configuration after enabling it */
  I2C_Init(TP_I2C, &I2C_InitStructure);
   
  /* sEE_I2C Peripheral Enable */
  I2C_Cmd(TP_I2C, ENABLE);
}

void DebugLED(uint8_t ui8LEDState, uint8_t ui8LEDNumber)
{
   if(ui8LEDState == CLEAR_LED)
   {
      // LED is inactive on an I/O HIGH state (sink)
      //
      LED_On(LED1);
   }
   else
   {
       // LED is active on an I/O LOW state (sink)
      LED_Off(LED1);
   }
}

void DebugIO(uint8_t ui8IOState, uint8_t ui8IONumber)
{
   if(ui8IOState == SET_IO)
   {
     printf("debug IO set");
   }
   else
   {
      printf("debug IO reset");
   }
}

/**
  * @brief  Initializes the I2C source clock and IOs used to drive the Touch.
  * @param  None
  * @retval None
  */
void HW_TP_LowLevel_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Configure the I2C clock source. The clock is derived from the HSI */
  RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);

  /* sEE_I2C_SCL_GPIO_CLK and sEE_I2C_SDA_GPIO_CLK Periph clock enable */
  RCC_AHBPeriphClockCmd(TP_I2C_SCL_GPIO_CLK | TP_I2C_SDA_GPIO_CLK, ENABLE);

  /* sEE_I2C Periph clock enable */
  RCC_APB1PeriphClockCmd(TP_I2C_CLK, ENABLE);

  /* Connect PXx to I2C_SCL*/
  GPIO_PinAFConfig(TP_I2C_SCL_GPIO_PORT, TP_I2C_SCL_SOURCE, EE_I2C_SCL_AF);

  /* Connect PXx to I2C_SDA*/
  GPIO_PinAFConfig(TP_I2C_SDA_GPIO_PORT, TP_I2C_SDA_SOURCE, EE_I2C_SDA_AF);

  /* GPIO configuration */
  /* Configure sEE_I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = TP_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(TP_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /* Configure sEE_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = TP_I2C_SDA_PIN;
  GPIO_Init(TP_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
}
