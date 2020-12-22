/* Includes ------------------------------------------------------------------*/
//#include <RotaryEncoder.h>
//#include "DSPControl.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"

/** @addtogroup STM32F1xx_HAL_Applications
  * @{
  */



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
void lcd_numTOstr(uint16_t value, uint8_t nDigit, uint8_t *buf)
{
    //u_int8_t digi;

    switch (nDigit)
    {
    case 4:
        buf[nDigit - 4] = ((value / 1000) + '0');
    case 3:
        buf[nDigit - 3] = (((value / 100) % 10) + '0');
    case 2:
        buf[nDigit - 2] = (((value / 10) % 10) + '0');
    case 1:
        buf[nDigit - 1] = ((value % 10) + '0');
    }
}

void lcd_put_hex(uint8_t b, uint8_t *buf)
{
    /* upper nibble */
    if ((b >> 4) < 0x0a)
        buf[0] = ((b >> 4) + '0');
    else
        buf[0] = ((b >> 4) - 0x0a + 'a');

    /* lower nibble */
    if ((b & 0x0f) < 0x0a)
        buf[1] = ((b & 0x0f) + '0');
    else
        buf[1] = ((b & 0x0f) - 0x0a + 'a');
}

    void DispSB(uint32_t DataPin, uint32_t ClockPin, uint8_t byte)
    {
        uint8_t k;
        delay_us(120);
        digitalWrite(ClockPin, LOW);
        delay_us(2);
        for (k = 0; k < 8; k++)
        {
            digitalWrite(ClockPin, LOW);
            delay_us(2);
            digitalWrite(DataPin, (byte & (1 << k)));
            delay_us(2);
            digitalWrite(ClockPin, HIGH);
            delay_us(2);
        };
        delay_us(2);
        digitalWrite(ClockPin, HIGH);
        digitalWrite(DataPin, LOW);
    }

    void DispSend(uint32_t DispDataPin, uint32_t DispClockPin, uint32_t DispCSPin, uint8_t *data, uint8_t size) //size = command without cheksum + zero byte
    {
        uint8_t chksumm = 0;
        uint8_t i;
        digitalWrite(DispCSPin, HIGH);

        for (i = 0; i < size; i++)
        {
            chksumm = chksumm + data[i];
            DispSB(DispDataPin, DispClockPin, data[i]);
        }
        DispSB(DispDataPin, DispClockPin, chksumm);
        DispSB(DispDataPin, DispClockPin, 0);

        delay_us(2);
        digitalWrite(DispCSPin, LOW);
    }


/* Private variables ---------------------------------------------------------*/

/* Virtual address defined by the user: 0xFFFF value is prohibited */


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F103xB HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 64 MHz */
  SystemClock_Config();

    LL_GPIO_AF_Remap_SWJ_NOJTAG();

    pinMode(POWERLED, OUTPUT);
    digitalWrite(POWERLED, HIGH);
    pinMode(PowerON, OUTPUT);
    digitalWrite(PowerON, LOW); //LOW
    pinMode(F_RLY, OUTPUT);
    digitalWrite(F_RLY, LOW);
    pinMode(SMUTE, OUTPUT);
    digitalWrite(SMUTE, LOW);

    pinMode(DISP_RESET, OUTPUT);
    digitalWrite(DISP_RESET, LOW);//LOW

    pinMode(DISP_SPI_CS, OUTPUT);
    digitalWrite(DISP_SPI_CS, LOW);

    pinMode(DISP_SPI_MOSI, OUTPUT);
    digitalWrite(DISP_SPI_MOSI, LOW);
    pinMode(DISP_SPI_SCK, OUTPUT);


    pinMode(POWERKEY, INPUT_PULLDOWN);

    while (digitalRead(POWERKEY) != HIGH)
        {

        delay(10);

        }

    digitalWrite(PowerON, HIGH);
    powerstatus = ON;
    digitalWrite(POWERLED, LOW);

    delay(2000); //1000

    digitalWrite(DISP_SPI_SCK, HIGH);
    digitalWrite(DISP_RESET, HIGH);

    delay(100);

    encoder.begin(); //set encoders pins as input & enable built-in pullup resistors
    encoder.setPosition(volumeposition);
    oldvolumeposition = volumeposition;


    DispSend(DISP_SPI_MOSI, DISP_SPI_SCK, DISP_SPI_CS, disp_init1, sizeof(disp_init1));
    delay(12);
    DispSend(DISP_SPI_MOSI, DISP_SPI_SCK, DISP_SPI_CS, disp_init2, sizeof(disp_init2));
    delay(12);
    disp_init3[3] = volume_m[volumeposition - 1].message2[0];
    disp_init3[4] = volume_m[volumeposition - 1].message2[1];
    disp_init3[5] = volume_m[volumeposition - 1].message2[2];
    disp_init3[6] = volume_m[volumeposition - 1].message2[3];
    DispSend(DISP_SPI_MOSI, DISP_SPI_SCK, DISP_SPI_CS, disp_init3, sizeof(disp_init3));
    delay(12);
    DispSend(DISP_SPI_MOSI, DISP_SPI_SCK, DISP_SPI_CS, disp_init4, sizeof(disp_init4));


    attachInterrupt(digitalPinToInterrupt(ENC1), encoderISR, CHANGE); //call encoderISR()    every high->low or low->high changes
    //attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call pushButtonISR() every high->low              changes

    delay(1000);

    DSP.begin(volume_m[volumeposition - 1].voldsp, chan); //prepare DSP

    delay(100);

    //Speaker A relay ON
    //Mute off
    digitalWrite(F_RLY, HIGH);
    digitalWrite(SMUTE, HIGH); //MUTE off. Real XMUTE is inverse. For mute send low level or 0v to DSP board.


    //DSP.set_mvolume(volume_m[volumeposition - 1].voldsp);

    /* GPIOA Clock */
    //LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    //LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    //LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    /**TIM2 GPIO Configuration  
    PA0-WKUP   ------> TIM2_CH1
    PA1   ------> TIM2_CH2 
    */
    //LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    //GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
    //GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    //GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    //LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Peripheral clock enable */
    //__HAL_RCC_TIM2_CLK_ENABLE();
    //LL_APB1_GRP1_PERIPH_PWR
    //__HAL_RCC_APB1_RELEASE_RESET
    //LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    /*
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12);
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV32_N8);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV32_N8);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_FALLING);
    TIM_InitStruct.Prescaler = 72;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 65535;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV2;
    LL_TIM_Init(TIM2, &TIM_InitStruct);
    LL_TIM_EnableARRPreload(TIM2);
    LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM2);
    */



  while (1)
  {
            delay(100);
        //selector = LL_TIM_GetCounter(TIM2); //
        //direction = LL_TIM_GetDirection(TIM2);
        volumeposition = encoder.getPosition();

        /*
        if (volumeposition > 238)
        {
            volumeposition = 238;
            encoder.setPosition(volumeposition);
            //LL_TIM_SetCounter(TIM2, volumeposition);
        }
        else if (volumeposition < 1)
        {
            volumeposition = 1;
            encoder.setPosition(volumeposition);
            //LL_TIM_SetCounter(TIM2, volumeposition);
        }
        */

        if (oldvolumeposition != volumeposition)
        {
            //uint8_t disp_data[5] = 
            //lcd_numTOstr(volumeposition, 3, disp_data);
            disp_init3[3] = volume_m[volumeposition-1].message2[0];
            disp_init3[4] = volume_m[volumeposition-1].message2[1];
            disp_init3[5] = volume_m[volumeposition-1].message2[2];
            disp_init3[6] = volume_m[volumeposition-1].message2[3];
            DispSend(DISP_SPI_MOSI, DISP_SPI_SCK, DISP_SPI_CS, disp_init3, sizeof(disp_init3));

            DSP.set_mvolume(volume_m[volumeposition - 1].voldsp);
            delay(5);

            oldvolumeposition = volumeposition;
        }
        else 
        {
            oldvolumeposition = volumeposition;
        }
            //LL_TIM_COUNTERDIRECTION_UP  LL_TIM_COUNTERDIRECTION_DOWN


        if (digitalRead(POWERKEY) == HIGH)
        {
            digitalWrite(DISP_RESET, LOW);
            //delay(10);
            digitalWrite(F_RLY, LOW);
            digitalWrite(SMUTE, HIGH);
            digitalWrite(PowerON, LOW);
            digitalWrite(POWERLED, HIGH);
            delay(1000);
            NVIC_SystemReset();
        }
  };
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Configure PLL ------------------------------------------------------*/
  /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  /* Enable HSI and activate PLL with HSi_DIV2 as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
  oscinitstruct.HSEState        = RCC_HSE_OFF;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_ON;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
