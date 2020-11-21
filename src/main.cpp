#include <Arduino.h>
//#include <stm32yyxx_ll_gpio.h>
#include <RotaryEncoder.h>
//#include <include/gpio.h>
//#include <Wire.h>
#include <SPI.h>
//#include <stdint.h>
//#include <delay.h>

#define VOL_A PB8  //ky-040 clk pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define VOL_B PB9  //ky-040 dt  pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define BUTTON PA12 //ky-040 sw  pin, add 100nF/0.1uF capacitors between pin & ground!!!

uint16_t buttonCounter = 0;
uint16_t volumeposition = 0;

RotaryEncoder encoder_vol(VOL_A, VOL_B, BUTTON);

void encoderISR()
{
    encoder_vol.readAB();
}

void encoderButtonISR()
{
    encoder_vol.readPushButton();
}

void shift16(uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder, uint16_t ulVal);

void DispSend(uint8_t *data, uint8_t size);

    __STATIC_INLINE void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // запускаем счётчик
}

__STATIC_INLINE void delay_us(uint32_t us)
{
    uint32_t us_count_tic = us * (SystemCoreClock / 1000000U);
    DWT->CYCCNT = 0U;
    while (DWT->CYCCNT < us_count_tic);
}

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
    'A','N','T','O','N',' ','M','E','L','N','I','K','O','V',  //14 letters
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

uint16_t sound_init[] = {
    0x2800, //input select IN10 to MAIN.
    0x0001, //input select SUB1 & SUB2
    0x4002, //mode L+R to MAIN
    0x10a3, //volume FR 0x0003 0dB 0x03f3 -31,5dB
    0x30a3, //volume FL 0x2003 0dB 0x23f3 -31,5dB
    0x0008, //tone bass
    0x0009, //tone treeble
    0x0000, //test, as is from datasheet.
    // However datasheet specified sent to addresses from 0 to 5
    // 0,1,2,3...3,4,5  adress 3 init every channel
    0x000B //volume change scheme
};

//#define SPIDSP_CS PA15
#define SPIDSP_SCK PB14
//#define SPIDSP_MISO PB14
#define SPIDSP_MOSI PB15

#define DISP_SPI_CS PA15
#define DISP_SPI_MOSI PB5
#define DISP_SPI_MISO PB4
#define DISP_SPI_SCK PB3
#define DISP_RESET PB6

#define PowerON PB10 //aka RYAC
#define F_RLY PB12
#define SP_B_RLY PB13
#define XSMUTE PB11  //XAMUTE active low in controller, active high in scheme
#define POWERKEY PA12

//SPIClass::SPIClass(uint8_t mosi, uint8_t miso, uint8_t sclk, uint8_t ssel)
SPIClass SPIdisp(DISP_SPI_MOSI, DISP_SPI_MISO, DISP_SPI_SCK);
SPISettings spiDispSettings(10000, LSBFIRST, SPI_MODE3);

void setup ()
{
    // put your setup code here, to run once:
    
    AFIO->MAPR |= AFIO_MAPR_SPI1_REMAP;          //remap SPI1 pins to PB3,4,5
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE; //disable only JTAG for free more pins

    pinMode(PC13, OUTPUT);
    pinMode(DISP_RESET, OUTPUT);
    //digitalWrite(DISP_RESET, LOW);

    pinMode(DISP_SPI_CS, OUTPUT);
    digitalWrite(DISP_SPI_CS, LOW);

    pinMode(SPIDSP_SCK, OUTPUT);
    pinMode(SPIDSP_MOSI, OUTPUT);
    digitalWrite(SPIDSP_SCK, LOW);
    digitalWrite(SPIDSP_MOSI, LOW);

    DWT_Init();

    SPIdisp.begin(); //prepare and start SPI
    SPIdisp.beginTransaction(spiDispSettings);

    //SPIsnd.setBitOrder(LSBFIRST);
    //SPIsnd.setDataMode(SPI_MODE0);

    encoder_vol.begin(); //set encoders pins as input & enable built-in pullup resistors

    digitalWrite(DISP_RESET, HIGH);
    delay (100);
    digitalWrite(DISP_SPI_CS, HIGH);

    DispSend(disp_init1, 6);
    delay(12);
    DispSend(disp_init2, 19);
    delay(12);
    DispSend(disp_init3, 9);
    delay(12);
    DispSend(disp_init4, 15);
    delay(12);
    digitalWrite(DISP_SPI_CS, LOW);

    attachInterrupt(digitalPinToInterrupt(VOL_A), encoderISR, CHANGE);         //call encoderISR()    every high->low or low->high changes
    attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call pushButtonISR() every high->low              changes

    for (size_t i = 0; i < 8; i++) //init DSP
    {
        //SPIsnd.transfer16(sound_init[i], SPI_LAST);
        shift16(SPIDSP_MOSI, SPIDSP_SCK, MSBFIRST, sound_init[i]);
        delay_us(10);
    }

}

    void loop()
    {
        // put your main code here, to run repeatedly:
        delay(1000);
        volumeposition = encoder_vol.getPosition();
        buttonCounter = encoder_vol.getPushButton();
    }

    void shift16(uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder, uint16_t ulVal)
    {
        uint8_t i;
        for (i = 0; i < 16; i++)
        {
            delay_us(1);
            digitalWrite(ulDataPin, LOW);
            delay_us(1);
            digitalWrite(ulClockPin, LOW);
            delay_us(1);
            digitalWrite(ulDataPin, (ulVal & (1 << (15 - i))));
            delay_us(1);
            digitalWrite(ulClockPin, HIGH);
            
        }
        delay_us(1);
        digitalWrite(ulDataPin, HIGH);
        delay_us(1);
        digitalWrite(ulClockPin, LOW);
        delay_us(1);
        digitalWrite(ulDataPin, LOW);
    }

    void DispSend(uint8_t *data, uint8_t size)
    {
        for (size_t i = 0; i < size; i++)
            {
            SPIdisp.transfer(data[i]);
            delay_us(100);
        }
    }