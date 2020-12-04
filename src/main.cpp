#include <Arduino.h>
#include "main.h"

#include <RotaryEncoder.h>
#include "DSPControl.h"
#include "stm32yyxx_hal_conf.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

//#define USE_FULL_LL_DRIVER
//#include <Wire.h>
//#include <SPI.h>
//#include <stdint.h>
//#include <delay.h>

void setup()
{
    // put your setup code here, to run once:
    //RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    //AFIO->MAPR |= AFIO_MAPR_SPI1_REMAP;          //remap SPI1 pins to PB3,4,5
    //AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE; //disable only JTAG for free more pins
    LL_GPIO_AF_Remap_SWJ_NOJTAG();

    pinMode(POWERLED, OUTPUT);
    digitalWrite(POWERLED, HIGH);
    pinMode(PowerON, OUTPUT);
    digitalWrite(PowerON, LOW);
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

    while (digitalRead(POWERKEY) != HIGH)
        {

        delay(10);

        }

    digitalWrite(PowerON, HIGH);
    powerstatus = true;
    delay(1000);

    digitalWrite(POWERLED, LOW);

    DSP.begin(); //prepare DSP

    encoder_vol.begin(); //set encoders pins as input & enable built-in pullup resistors
    encoder_vol.setPosition(volumeposition);

    digitalWrite(DISP_SPI_SCK, HIGH);
    digitalWrite(DISP_RESET, HIGH);
    
    delay(100);

    delay(1);
    DispSend(DISP_SPI_MOSI, DISP_SPI_SCK, DISP_SPI_CS, disp_init1, 4);
    delay(20);
    DispSend(DISP_SPI_MOSI, DISP_SPI_SCK, DISP_SPI_CS, disp_init2, 18);
    delay(20);
    DispSend(DISP_SPI_MOSI, DISP_SPI_SCK, DISP_SPI_CS, disp_init3, 8);
    delay(20);
    DispSend(DISP_SPI_MOSI, DISP_SPI_SCK, DISP_SPI_CS, disp_init4, 14);
    delay(12);


    attachInterrupt(digitalPinToInterrupt(VOL_A), encoderISR, CHANGE);         //call encoderISR()    every high->low or low->high changes
    //attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call pushButtonISR() every high->low              changes
    
    digitalWrite(F_RLY, HIGH);
    digitalWrite(XSMUTE, HIGH);

    
    pinMode(VOL_A, INPUT_PULLUP);
    pinMode(VOL_B, INPUT_PULLUP);
    /* GPIOA Clock */

    LL_TIM_InitTypeDef TIM_InitStruct = {0};


    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();


    LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12);
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV32_N8);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 65535;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM2, &TIM_InitStruct);
    LL_TIM_EnableARRPreload(TIM2);
    LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM2);
}

    void loop()
    {
        // put your main code here, to run repeatedly:
        delay(100);
        volumeposition = encoder_vol.getPosition();

        if (volumeposition > 237)
        {
            volumeposition = 237;
            encoder_vol.setPosition(volumeposition);
        }
        else if (volumeposition < 0)
        {
            volumeposition = 0;
            encoder_vol.setPosition(volumeposition);
        }

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

    void DispSend(uint32_t DispDataPin, uint32_t DispClockPin, uint32_t DispCSPin, uint8_t *data, uint8_t size)
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