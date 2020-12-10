#include <Arduino.h>
#include "main.h"

#include <RotaryEncoder.h>
#include "DSPControl.h"
#include "stm32yyxx_hal_conf.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"

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

    pinMode(POWERLED, OUTPUT);
    digitalWrite(POWERLED, HIGH);
    pinMode(PowerON, OUTPUT);
    digitalWrite(PowerON, HIGH); //LOW
    pinMode(F_RLY, OUTPUT);
    digitalWrite(F_RLY, LOW);
    pinMode(XSMUTE, OUTPUT);
    digitalWrite(XSMUTE, LOW);

    pinMode(DISP_RESET, OUTPUT);
    digitalWrite(DISP_RESET, LOW);//LOW

    pinMode(DISP_SPI_CS, OUTPUT);
    digitalWrite(DISP_SPI_CS, LOW);

    pinMode(DISP_SPI_MOSI, OUTPUT);
    digitalWrite(DISP_SPI_MOSI, LOW);
    pinMode(DISP_SPI_SCK, OUTPUT);


    pinMode(POWERKEY, INPUT_PULLDOWN);
/*
    while (digitalRead(POWERKEY) != HIGH)
        {

        delay(10);

        }
*/
    digitalWrite(PowerON, HIGH);
    powerstatus = true;
    delay(100); //1000

    digitalWrite(DISP_SPI_SCK, HIGH);
    digitalWrite(DISP_RESET, HIGH);

    digitalWrite(POWERLED, LOW);

    DSP.begin(); //prepare DSP

    encoder.begin(); //set encoders pins as input & enable built-in pullup resistors
    encoder.setPosition(volumeposition);
    oldvolumeposition = volumeposition;
    delay(100);

    DSP.set_mvolume(volume_m[volumeposition - 1].volumestr);

    delay(1);
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
    delay(12);

    attachInterrupt(digitalPinToInterrupt(ENC1), encoderISR, CHANGE); //call encoderISR()    every high->low or low->high changes
    //attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call pushButtonISR() every high->low              changes

    //Speacer relay ON
    //Mute off
    //digitalWrite(F_RLY, HIGH);
    //digitalWrite(XSMUTE, HIGH);

    /* GPIOA Clock */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    /**TIM2 GPIO Configuration  
    PA0-WKUP   ------> TIM2_CH1
    PA1   ------> TIM2_CH2 
    */
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Peripheral clock enable */
    //__HAL_RCC_TIM2_CLK_ENABLE();
    //LL_APB1_GRP1_PERIPH_PWR
    //__HAL_RCC_APB1_RELEASE_RESET
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

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

}

    void loop()
    {
        // put your main code here, to run repeatedly:
        delay(100);
        selector = LL_TIM_GetCounter(TIM2); //
        direction = LL_TIM_GetDirection(TIM2);
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

            DSP.set_mvolume(volume_m[volumeposition - 1].volumestr);
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
            digitalWrite(XSMUTE, LOW);
            digitalWrite(PowerON, LOW);
            digitalWrite(POWERLED, HIGH);
            delay(1000);
            NVIC_SystemReset();
        }
    }

    __STATIC_INLINE void DispSB(uint32_t DataPin, uint32_t ClockPin, uint8_t byte)
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

