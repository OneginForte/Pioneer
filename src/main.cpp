#include <Arduino.h>
#include "main.h"

#include <RotaryEncoder.h>
#include "DSPControl.h"
#include "stm32yyxx_hal_conf.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"

static void MX_GPIO_Init(void);
__STATIC_INLINE void DispSB(GPIO_TypeDef *DispDataPort, uint32_t DispDataPin, GPIO_TypeDef *DispClockPort, uint32_t DispClockPin, uint8_t byte);

    //#define USE_FULL_LL_DRIVER
    //#include <Wire.h>
    //#include <SPI.h>
    //#include <stdint.h>
    //#include <delay.h>

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

void setup()
{
    LL_GPIO_AF_Remap_SWJ_NOJTAG();

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    MX_GPIO_Init();

    /* 
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
*/

    while (LL_GPIO_IsInputPinSet(POWKEY_GPIO_Port, POWKEY_Pin) != HIGH)
    {

        delay(10);

    }

    //digitalWrite(PowerON, HIGH);
    LL_GPIO_SetOutputPin(POWERON_GPIO_Port, POWERON_Pin);

    powerstatus = ON;
    //digitalWrite(POWERLED, LOW);
    LL_GPIO_SetOutputPin(POWERLED_GPIO_Port,POWERLED_Pin);

    delay(2000); //1000

    //digitalWrite(DISP_SPI_SCK, HIGH);
    LL_GPIO_SetOutputPin(DISP_SPI_SCK_GPIO_Port, DISP_SPI_SCK_Pin);
    //digitalWrite(DISP_RESET, HIGH);
    LL_GPIO_SetOutputPin(DISP_SPI_RESET_GPIO_Port, DISP_SPI_RESET_Pin);

    delay(100);

    encoder.begin(); //set encoders pins as input & enable built-in pullup resistors
    encoder.setPosition(volumeposition);
    oldvolumeposition = volumeposition;

    DispSend(DISP_SPI_MOSI_GPIO_Port, DISP_SPI_MOSI_Pin, DISP_SPI_SCK_GPIO_Port, DISP_SPI_SCK_Pin, DISP_SPI_CS_GPIO_Port, DISP_SPI_CS_Pin, disp_init1, sizeof(disp_init1));
    delay(12);
    DispSend(DISP_SPI_MOSI_GPIO_Port, DISP_SPI_MOSI_Pin, DISP_SPI_SCK_GPIO_Port, DISP_SPI_SCK_Pin, DISP_SPI_CS_GPIO_Port, DISP_SPI_CS_Pin, disp_init2, sizeof(disp_init2));
    delay(12);
    disp_init3[3] = volume_m[volumeposition - 1].message2[0];
    disp_init3[4] = volume_m[volumeposition - 1].message2[1];
    disp_init3[5] = volume_m[volumeposition - 1].message2[2];
    disp_init3[6] = volume_m[volumeposition - 1].message2[3];
    DispSend(DISP_SPI_MOSI_GPIO_Port, DISP_SPI_MOSI_Pin, DISP_SPI_SCK_GPIO_Port, DISP_SPI_SCK_Pin, DISP_SPI_CS_GPIO_Port, DISP_SPI_CS_Pin, disp_init3, sizeof(disp_init3));
    delay(12);
    DispSend(DISP_SPI_MOSI_GPIO_Port, DISP_SPI_MOSI_Pin, DISP_SPI_SCK_GPIO_Port, DISP_SPI_SCK_Pin, DISP_SPI_CS_GPIO_Port, DISP_SPI_CS_Pin, disp_init4, sizeof(disp_init4));

    attachInterrupt(digitalPinToInterrupt(ENC1), encoderISR, CHANGE); //call encoderISR()    every high->low or low->high changes
    //attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call pushButtonISR() every high->low              changes

    delay(1000);

    DSP.begin(volume_m[volumeposition - 1].voldsp, chan); //prepare DSP

    delay(100);

    //Speaker A relay ON
    //Mute off
    //digitalWrite(F_RLY, HIGH);
    LL_GPIO_SetOutputPin(F_RLY_GPIO_Port, F_RLY_Pin);
    //digitalWrite(SMUTE, HIGH); //MUTE off. Real XMUTE is inverse. For mute send low level or 0v to DSP board.
    LL_GPIO_SetOutputPin(SMUTE_GPIO_Port, SMUTE_Pin);

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
}

    void loop()
    {
        // put your main code here, to run repeatedly:
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
            DispSend(DISP_SPI_MOSI_GPIO_Port, DISP_SPI_MOSI_Pin, DISP_SPI_SCK_GPIO_Port, DISP_SPI_SCK_Pin, DISP_SPI_CS_GPIO_Port, DISP_SPI_CS_Pin, disp_init3, sizeof(disp_init3));

            DSP.set_mvolume(volume_m[volumeposition - 1].voldsp);
            delay(5);

            oldvolumeposition = volumeposition;
        }
        else 
        {
            oldvolumeposition = volumeposition;
        }
            //LL_TIM_COUNTERDIRECTION_UP  LL_TIM_COUNTERDIRECTION_DOWN

        if (LL_GPIO_IsInputPinSet(POWKEY_GPIO_Port, POWKEY_Pin) == HIGH)
        {
            //digitalWrite(DISP_RESET, LOW);
            LL_GPIO_ResetOutputPin(DISP_SPI_RESET_GPIO_Port, DISP_SPI_RESET_Pin);
            //delay(10);
            //digitalWrite(F_RLY, LOW);
            LL_GPIO_ResetOutputPin(F_RLY_GPIO_Port, F_RLY_Pin);
            //digitalWrite(SMUTE, HIGH);
            LL_GPIO_SetOutputPin(SMUTE_GPIO_Port, SMUTE_Pin);
            //digitalWrite(PowerON, LOW);
            LL_GPIO_ResetOutputPin(POWERON_GPIO_Port, POWERON_Pin);
            //digitalWrite(POWERLED, HIGH);
            LL_GPIO_SetOutputPin(POWERLED_GPIO_Port, POWERLED_Pin);
            delay(1000);
            NVIC_SystemReset();
        }
    }

    static void MX_GPIO_Init(void)
    {
        LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

        /* GPIO Ports Clock Enable */
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

        /**/
        LL_GPIO_SetOutputPin(POWERLED_GPIO_Port, POWERLED_Pin);

        /**/
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_14 | LL_GPIO_PIN_15);

        /**/
        LL_GPIO_ResetOutputPin(GPIOB, POWERON_Pin | SMUTE_Pin | F_RLY_Pin | SP_B_RLY_Pin | DISP_SPI_SCK_Pin | DISP_SPI_MOSI_Pin | DISP_SPI_RESET_Pin);

        /**/
        LL_GPIO_ResetOutputPin(DISP_SPI_CS_GPIO_Port, DISP_SPI_CS_Pin);

        /**/
        GPIO_InitStruct.Pin = POWERLED_Pin;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
        LL_GPIO_Init(POWERLED_GPIO_Port, &GPIO_InitStruct);

        /**/
        GPIO_InitStruct.Pin = LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /**/
        GPIO_InitStruct.Pin = VOL_A_Pin | VOL_B_Pin | ENC1_Pin | ENC2_Pin;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
        LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /**/
        GPIO_InitStruct.Pin = POWERON_Pin | SMUTE_Pin | F_RLY_Pin | SP_B_RLY_Pin;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
        LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /**/
        GPIO_InitStruct.Pin = SPIDSP_SCK_Pin | SPIDSP_MOSI_Pin;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
        LL_GPIO_Init(SPIDSP_Port, &GPIO_InitStruct);

        /**/
        GPIO_InitStruct.Pin = POWKEY_Pin;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
        LL_GPIO_Init(POWKEY_GPIO_Port, &GPIO_InitStruct);

        /**/
        GPIO_InitStruct.Pin = DISP_SPI_CS_Pin;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        LL_GPIO_Init(DISP_SPI_CS_GPIO_Port, &GPIO_InitStruct);

        /**/
        GPIO_InitStruct.Pin = DISP_SPI_SCK_Pin | DISP_SPI_MOSI_Pin | DISP_SPI_RESET_Pin;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }

    __STATIC_INLINE void DispSB(GPIO_TypeDef * DispDataPort, uint32_t DispDataPin, GPIO_TypeDef * DispClockPort, uint32_t DispClockPin, uint8_t byte)
    {
        uint8_t k;
        delay_us(120);
        //digitalWrite(ClockPin, LOW);
        LL_GPIO_ResetOutputPin(DispClockPort, DispClockPin);
        delay_us(2);
        for (k = 0; k < 8; k++)
        {
            //digitalWrite(ClockPin, LOW);
            LL_GPIO_ResetOutputPin(DispClockPort, DispClockPin);
            delay_us(2);
            //digitalWrite(DataPin, (byte & (1 << k)));
            if ((byte & (1 << k))==HIGH)
            {
                LL_GPIO_SetOutputPin(DispDataPort, DispDataPin);
            }
            else
            {
                LL_GPIO_ResetOutputPin(DispDataPort, DispDataPin);
            }

            delay_us(2);
            //digitalWrite(ClockPin, HIGH);
            LL_GPIO_SetOutputPin(DispClockPort, DispClockPin);
            delay_us(2);
        };
        delay_us(2);
        //digitalWrite(ClockPin, HIGH);
        LL_GPIO_SetOutputPin(DispClockPort, DispClockPin);
        //digitalWrite(DataPin, LOW);
        LL_GPIO_ResetOutputPin(DispDataPort, DispDataPin);
    }

    void DispSend(GPIO_TypeDef *DispDataPort, uint32_t DispDataPin, GPIO_TypeDef *DispClockPort, uint32_t DispClockPin, GPIO_TypeDef * DispCSPort, uint32_t DispCSPin, uint8_t *data, uint8_t size) //size = command without cheksum + zero byte
    {
        uint8_t chksumm = 0;
        uint8_t i;
        //digitalWrite(DispCSPin, HIGH);
        LL_GPIO_SetOutputPin(DispCSPort, DispCSPin);

        for (i = 0; i < size; i++)
        {
            chksumm = chksumm + data[i];
            DispSB(DispDataPort, DispDataPin, DispClockPort, DispClockPin, data[i]);
        }
        DispSB(DispDataPort, DispDataPin, DispClockPort, DispClockPin, chksumm);
        DispSB(DispDataPort, DispDataPin, DispClockPort, DispClockPin, 0);

        delay_us(2);
        //digitalWrite(DispCSPin, LOW);
        LL_GPIO_ResetOutputPin(DispCSPort, DispCSPin);
    }
