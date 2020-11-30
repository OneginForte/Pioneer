#include <Arduino.h>
#include "main.h"

#include <RotaryEncoder.h>
#include "DSPControl.h"

//#include <Wire.h>
//#include <SPI.h>
//#include <stdint.h>
//#include <delay.h>

#define VOL_A PB8  //ky-040 clk pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define VOL_B PB9  //ky-040 dt  pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define BUTTON PA10 //ky-040 sw  pin, add 100nF/0.1uF capacitors between pin & ground!!!

enum channel_t : uint8_t
{
    IN1,
    IN2,
    IN3,
    IN4,
    IN5,
    IN6,
    IN7,
    IN8,
    IN9,
    IN10,
    IN11,
    IN12,
    IN13,
    IN14,
    IN15,
    IN16,
    IN17,
    IN18
};

uint16_t buttonCounter = 0;

//DSPControl(uint8_t dsp_sck, uint8_t dsp_mosi,uint8_t volume, Channel channel);
volatile uint8_t powerstatus = false;

RotaryEncoder encoder_vol(VOL_A, VOL_B);

//ini DSP pins and default value
#define SPIDSP_SCK PB14
#define SPIDSP_MOSI PB15
uint16_t volumeposition = 48; //default volume 0dB

channel_t chan;
//chan = IN10; //default channel

//DSPControl DSP(SPIDSP_SCK,SPIDSP_MOSI, volumeposition, (uint8_)chan);


void encoderISR()
{
    encoder_vol.readAB();
}

__STATIC_INLINE void delay_us(uint32_t us)
{
    uint32_t us_count_tic = us * (SystemCoreClock / 1000000U);
    DWT->CYCCNT = 0U;
    while (DWT->CYCCNT < us_count_tic)
        ;
};

void DispSend(uint32_t DispDataPin, uint32_t DispClockPin, uint32_t DispCSkPin, uint8_t *data, uint8_t size);

uint8_t disp_init1[] = 
    {
    0x05,
    0x11,
    0x14,
    0x01,
    0x2B,
    0x00
    };

uint8_t disp_init2[] =
    {
    0x13, 
    0x11, 
    0x11, 
    ' ',' ',' ','P','O','W','E','R',' ','O','N',' ',' ',' ',  //14 letters 
    0x00, 
    0x3F, 
    0x00
    };

uint8_t disp_init3[] =
    {
    0x09,
    0x11,
    0x12,
    ' ',' ',' ',' ', //3 letters
    0x00,
    0xAC,
    0x00
    };

uint8_t disp_init4[] =
    {
    0x0F,
    0x11,
    0x13,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x33,
    0x00
    };






#define DISP_SPI_CS PA15
#define DISP_SPI_MOSI PB5
//#define DISP_SPI_MISO PB4
#define DISP_SPI_SCK PB3
#define DISP_RESET PB6

#define PowerON PB10 //aka RYAC
#define F_RLY PB12
#define SP_B_RLY PB13
#define XSMUTE PB11  //XAMUTE active low in controller, active high in scheme
#define POWERKEY BUTTON
#define POWERLED PC13

//SPIClass::SPIClass(uint8_t mosi, uint8_t miso, uint8_t sclk, uint8_t ssel)
//SPIClass SPIdisp(DISP_SPI_MOSI, DISP_SPI_MISO, DISP_SPI_SCK);
//SPISettings spiDispSettings(10000, LSBFIRST, SPI_MODE3);

void setup ()
{
    // put your setup code here, to run once:
    //RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    //AFIO->MAPR |= AFIO_MAPR_SPI1_REMAP;          //remap SPI1 pins to PB3,4,5
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE; //disable only JTAG for free more pins

    pinMode(POWERLED, OUTPUT);
    digitalWrite(POWERLED, HIGH);
    pinMode(PowerON, OUTPUT);
    digitalWrite(PowerON, LOW);
    pinMode(F_RLY, OUTPUT);
    digitalWrite(F_RLY, LOW);
    pinMode(XSMUTE, OUTPUT);
    digitalWrite(XSMUTE, LOW);

    pinMode(DISP_SPI_CS, OUTPUT);
    digitalWrite(DISP_SPI_CS, LOW);

    pinMode(DISP_RESET, OUTPUT);
    digitalWrite(DISP_RESET, LOW);

    pinMode(POWERKEY, INPUT_PULLDOWN);

        while (digitalRead(POWERKEY) != HIGH)
    {

    

    }

    digitalWrite(PowerON, HIGH);
    powerstatus = true;


    digitalWrite(POWERLED, LOW);

    //DSP.begin(); //prepare DSP


    pinMode(DISP_SPI_MOSI, OUTPUT);
    digitalWrite(DISP_SPI_MOSI, LOW);
    pinMode(DISP_SPI_SCK, OUTPUT);
    digitalWrite(DISP_SPI_SCK, HIGH);
    digitalWrite(DISP_RESET, HIGH);
    delay(1000);
    encoder_vol.begin(); //set encoders pins as input & enable built-in pullup resistors

    delay(100);

    delay(1);
    DispSend(DISP_SPI_MOSI, DISP_SPI_SCK, DISP_SPI_CS, disp_init1, 6);
    delay(20);
    DispSend(DISP_SPI_MOSI, DISP_SPI_SCK, DISP_SPI_CS, disp_init2, 19);
    delay(20);
    DispSend(DISP_SPI_MOSI, DISP_SPI_SCK, DISP_SPI_CS, disp_init3, 9);
    delay(20);
    DispSend(DISP_SPI_MOSI, DISP_SPI_SCK, DISP_SPI_CS, disp_init4, 15);
    delay(12);


    attachInterrupt(digitalPinToInterrupt(VOL_A), encoderISR, CHANGE);         //call encoderISR()    every high->low or low->high changes
    //attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call pushButtonISR() every high->low              changes
    
    /*
    for (size_t i = 0; i < 8; i++) //init DSP
    {
        //SPIsnd.transfer16(sound_init[i], SPI_LAST);
        shift16(SPIDSP_MOSI, SPIDSP_SCK, sound_init[i]);
        delay_us(10);
    }
    digitalWrite(F_RLY, HIGH);
    digitalWrite(XSMUTE, HIGH);
    */
}

    void loop()
    {
        // put your main code here, to run repeatedly:
        delay(100);
        volumeposition = encoder_vol.getPosition();

        if (digitalRead(POWERKEY) == HIGH)
        {
            
            digitalWrite(F_RLY, LOW);
            digitalWrite(XSMUTE, LOW);
            digitalWrite(PowerON, LOW);
            digitalWrite(POWERLED, HIGH);
            delay(1000);
            NVIC_SystemReset();
        }
    }

    void DispSend(uint32_t DispDataPin, uint32_t DispClockPin, uint32_t DispCSPin, uint8_t *data, uint8_t size)
    {
        digitalWrite(DispCSPin, HIGH);
        //delay_us(100);
        for (size_t i = 0; i < size; i++)
            {
            uint8_t k;
            delay_us(130);
            digitalWrite(DispClockPin, LOW);
            delay_us(3);
            for (k = 0; k < 8; k++)
            {
                digitalWrite(DispClockPin, LOW);
                delay_us(2);
                digitalWrite(DispDataPin, (data[i] & (1 << k)));
                delay_us(3);
                digitalWrite(DispClockPin, HIGH);
                delay_us(3);

            }
            delay_us(2);
            digitalWrite(DispClockPin, HIGH);
            
            digitalWrite(DispDataPin, LOW);
            //delay_us(100);
            }
        digitalWrite(DispCSPin, LOW);
    }