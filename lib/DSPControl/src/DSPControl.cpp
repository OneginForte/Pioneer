/***************************************************************************************************/
/*
   This is an Arduino library for 

   written by : enjoyneering79
   sourse code: https://github.com/enjoyneering/



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
        delay_us(10);
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