/***************************************************************************************************/
/*
   This is an Arduino library for 

   written by : OneginForte
   sourse code: https://github.com/OneginForte/

    The control data is 16 bit instructions with a latch bit at the end.
    Lowest 4 bits register address. 0-5... 
    The remaining 12 bits determine the contents of the registers
    
    However datasheet specified sent init chain to addresses from 0 to 5
    0,1,2,3...3,4,5 Adress 3 must init every of 8 channel (for BD3473KS2)

    0 adress input selector external channel to MAIN and REC internal channel. 0h00x0 - MUTE
      IN    REC
    xxxxxx|xxxxxx|0000
    IN1 - 0h04xx, IN2 - 0h08xx, IN3 - 0h0Cxx, IN4 - 0h10xx, IN5 - 0h14xx,
    IN6 - 0x1800, IN7 - 0x1C00, IN8 - 0x2000, IN9 - 0x2400, IN10 - 0x2800,
    IN11 - 0x2C00, IN12 - 0x3000, IN13 - 0x3400, IN14 - 0x3800, IN15 - 0x3C00,
    IN16 - 0x4000, IN17 - 0x4400, IN18 - 0x4800

    1 adress input selector to SUB1 and SUB2 channels. 0h0001 - MUTE
      SUB1   SUB2
    0000000|000000|0001
    
    From 0h0411 to 0h4921

    2 adress mode selector for channels MAIN, MULTI1, MULTI2 and ADC exit attenuation

    Not used channel must set to minimum volume level, is -95dB, DEC 191 HEX xxx|0|10111111|0011
    0h1xx3 volume range 0dB to +24dB, 0h0xx3 volume range 0 to -95dB
    FR - 0h0xx3, FL - 2xx3, SW - 0h4xx3, C- 0h6xx3, SR - 0h8xx3, SL - 0hAxx3, SBR - 0hCxx3, SBL - 0hExx3 

   Frameworks & Libraries:
   ATtiny  Core          - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core          - https://github.com/espressif/arduino-esp32
   ESP8266 Core          - https://github.com/esp8266/Arduino
   STM32   Core          - https://github.com/rogerclarkmelbourne/Arduino_STM32

   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#include "DSPControl.h"


/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/
DSPControl::DSPControl(uint8_t dsp_sck, uint8_t dsp_mosi,uint8_t volume, uint8_t channel)
{
uint8_t _dsp_sck = dsp_sck;
uint8_t _dsp_mosi = dsp_sck;
uint8_t _volume = volume;
uint8_t _channel = channel; 
}

__STATIC_INLINE void DWT_Init(void)
              {
              CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
              DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // запускаем счётчик
              };

__STATIC_INLINE void _delay_us(uint32_t us)
              {
              uint32_t us_count_tic = us * (SystemCoreClock / 1000000U);
              DWT->CYCCNT = 0U;
              while (DWT->CYCCNT < us_count_tic);
              };


/**************************************************************************/
/*
    begin

    Sets exit pins, init DWT.

    NOTE:

*/
/**************************************************************************/
void DSPControl::begin(void)
{
    uint16_t sound_init[] = 
    {
    0x2800, //input select IN10 to MAIN.
    0x0001, //input select SUB1 & SUB2
    0x4002, //mode L+R to MAIN
    0x10a3, //volume FR 0x0303 +24dB, 0x0003 0dB, 0x0BF3 -95dB
    0x30a3, //volume FL 0x2303 +24dB, 0x2003 0dB, 0x2BF3 -95dB
    0x4BF3, //SW channel -95dB
    0x6BF3, //C channel -95dB
    0x8BF3, //SR channel -95dB
    0xABF3, //SL channel -95dB
    0xCBF3, //SBR channel -95dB
    0xEBF3, //SBL channel -95dB
    0x0008, //tone bass
    0x0009, //tone treeble
    0x0000, //test, as is from datasheet.
    // However datasheet specified sent init chain to addresses from 0 to 5
    // 0,1,2,3...3,4,5  adress 3 must init every of 8 channel. 
    //Not used channel must set to minimum volume level, is -95dB, DEC 191 HEX xxx|0|10111111|0011
    //1xx3 volume range 0dB to +24dB,  0xx3 volume range 0 to -95dB
    //FR - 0xx3, FL - 2xx3, SW - 4xx3, C- 6xx3, SR - 8xx3, SL - Axx3, SBR - Cxx3, SBL - Exx3 
    0x000B //volume change scheme
    };
    
    pinMode(_dsp_sck, OUTPUT);
    digitalWrite(_dsp_sck, LOW);
    pinMode(_dsp_mosi, OUTPUT);
    digitalWrite(_dsp_mosi, LOW);

    DWT_Init();
    
    for (size_t i = 0; i < 8; i++) //init DSP
    {
        shift16(_dsp_mosi, _dsp_sck, sound_init[i]);
        _delay_us(10);
    }



}

/**************************************************************************/
/*
    setvolume()

    Set volume of channel

    NOTE:

*/
/**************************************************************************/
void  setvolume(uint8_t volume, uint8_t channel)
{
   delay(1);
}

/**************************************************************************/
/*
    setmute()

    Mute selected channel

    NOTE:

*/
/**************************************************************************/
void  setmute(uint8_t channel)
{

delay(1);

}


/*************************************************************************/
/*
    shift16()

    Software SPI process
*/
/**************************************************************************/
void shift16(uint32_t DataPin, uint32_t ClockPin, uint16_t Val)
    {
        uint8_t i;
        for (i = 0; i < 16; i++)
        {
            _delay_us(1);
            digitalWrite(DataPin, LOW);
            _delay_us(1);
            digitalWrite(ClockPin, LOW);
            _delay_us(1);
            digitalWrite(DataPin, (Val & (1 << (15 - i))));
            _delay_us(1);
            digitalWrite(ClockPin, HIGH);
            
        }
        _delay_us(1);
        digitalWrite(DataPin, HIGH);
        _delay_us(1);
        digitalWrite(ClockPin, LOW);
        _delay_us(1);
        digitalWrite(DataPin, LOW);
    }

