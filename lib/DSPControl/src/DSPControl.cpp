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

    Not used exit channel must set to minimum volume level, is -95dB, DEC 191 HEX xxx|0|10111111|0011  xxx - is nr of output channel
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

#ifdef __cplusplus
 extern "C" {
#endif

/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/
DSPControl::DSPControl(uint32_t dsp_sck, uint32_t dsp_mosi)
{
_dsp_sck = dsp_sck;
_dsp_mosi = dsp_mosi;
}

__STATIC_INLINE void DWT_Init(void)
              {
              CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
              DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // запускаем счётчик
              };


/**************************************************************************/
/*
    begin

    Sets exit pins, init DWT, init DSP.

    NOTE:

*/
/**************************************************************************/
              void DSPControl::begin(uint16_t volume, channel_tm channelin)
              {
                  uint16_t fsound_init[15] =
                      {
                          0x2800, //input select IN10 to MAIN.  //0x2800, //input select.
                          0x0001, //input select SUB1 & SUB2
                          0x4002, //mode L+R to MAIN
                          0x0003, //volume FR 0x0303 +24dB, 0x0003 0dB, 0x0BF3 -95dB, 0x0FF3 - mute
                          0x2003, //volume FL 0x2303 +24dB, 0x2003 0dB, 0x2BF3 -95dB, 0x2FF3 - mute
                          0x4FF3, //SW channel -95dB or mute
                          0x6FF3, //C channel -95dB or mute
                          0x8FF3, //SR channel -95dB or mute
                          0xAFF3, //SL channel -95dB or mute
                          0xCFF3, //SBR channel -95dB or mute
                          0xEFF3, //SBL channel -95dB or mute
                          0x2008, //tone bass
                          0x0009, //tone treeble
                          0x000A, //test, as is from datasheet.
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
                  fsound_init[0] = channelin;
                  fsound_init[3] = fsound_init[3] ^ volume;
                  fsound_init[4] = fsound_init[4] ^ volume;

                uint8_t i;
                  for (i = 0; i < 15; i++) //init DSP
                  {
                      shift16(_dsp_mosi, _dsp_sck, fsound_init[i]);
                      delay_us(100);
                  }



}

/**************************************************************************/
/*
    setvolume()

    Set volume of MAIN channel

    NOTE:

*/
/**************************************************************************/
void DSPControl::set_mvolume(uint16_t volume)
{
    uint16_t sound_init[5];
    sound_init[0] = 0x2800,          //input select IN10 to MAIN.  //0x2800, //input select.
    sound_init[1] = 0x0001,          //input select SUB1 & SUB2
    sound_init[2] = 0x4002,          //mode L+R to MAIN
    sound_init[3] = 0x0003 ^ volume; //FR
    sound_init[4] = 0x2003 ^ volume; //FL

    for (size_t i = 0; i < 5; i++) //send Volume
    {
        shift16(_dsp_mosi, _dsp_sck, sound_init[i]);
        delay_us(10);
    }
   delay(1);
}

/**************************************************************************/
/*
    setmute()

    Mute selected channel

    NOTE:

*/
/**************************************************************************/
void DSPControl::setmute(channel_tm channelin)
{
    uint16_t sound_init[2];
    sound_init[0] = 0x0FF3; //FR
    sound_init[1] = 0x2FF3; //FL

    for (size_t i = 0; i < 2; i++) //send Volume
    {
        shift16(_dsp_mosi, _dsp_sck, sound_init[i]);
        delay_us(10);
    }
}


/*************************************************************************/
/*
    shift16()

    Software 16 bit  SPI process with latch bit, MSB first
*/
/**************************************************************************/
void DSPControl::shift16(uint32_t DataPin, uint32_t ClockPin, uint16_t Val)
    {
        uint8_t i;
        for (i = 0; i < 16; i++)
        {
            delay_us(2);
            digitalWrite(DataPin, LOW);
            delay_us(2);
            digitalWrite(ClockPin, LOW);
            delay_us(2);
            digitalWrite(DataPin, (Val & (1 << (15 - i))));
            delay_us(2);
            digitalWrite(ClockPin, HIGH);
        }
        delay_us(2);
        digitalWrite(DataPin, HIGH);     //latch on
        delay_us(2);
        digitalWrite(ClockPin, LOW);
        delay_us(2);
        digitalWrite(DataPin, LOW);      //latch off
    }

#ifdef __cplusplus
}
#endif
