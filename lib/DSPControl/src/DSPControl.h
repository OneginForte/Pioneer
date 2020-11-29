/***************************************************************************************************/
/*
   This is an Arduino library for Quadrature Rotary Encoder 

   written by : enjoyneering79
   sourse code: https://github.com/enjoyneering/

   This library uses interrupts, specials pins are required to interface
   Board:                                    int.0  int.1  int.2  int.3  int.4  int.5            Level
   Uno, Mini, Pro, ATmega168, ATmega328..... 2      3      x       x      x     x                5v
   Mega2560................................. 2      3      21      20     19    18               5v
   Leonardo, Micro, ATmega32U4.............. 3      2      0       1      7     x                5v
   Digistump, Trinket, ATtiny85............. 2/physical pin 7                                    5v
   Due, SAM3X8E............................. all digital pins                                    3v
   Zero, ATSAMD21G18........................ all digital pins, except pin 4                      3v
   Blue Pill, STM32F103xxxx boards.......... all digital pins, maximum 16 pins at the same time  3v
   ESP8266.................................. all digital pins, except gpio6 - gpio11 & gpio16    3v/5v
   ESP32.................................... all digital pins                                    3v

   NOTE:
   - Quadrature encoder makes two waveforms that are 90° out of phase:
                           _______         _______         __
                  PinA ___|       |_______|       |_______|   PinA
          CCW <--              _______         _______
                  PinB _______|       |_______|       |______ PinB

                               _______         _______
                  PinA _______|       |_______|       |______ PinA
          CW  -->          _______         _______         __
                  PinB ___|       |_______|       |_______|   PinB


          The half of the pulses from top to bottom create full state array:  

          prev.A+B   cur.A+B   (prev.AB+cur.AB)  Array   Encoder State
          -------   ---------   --------------   -----   -------------
            00         00            0000          0     stop/idle
            00         01            0001          1     CW,  0x01
            00         10            0010         -1     CCW, 0x02
            00         11            0011          0     invalid state
            01         00            0100         -1     CCW, 0x04
            01         01            0101          0     stop/idle
            01         10            0110          0     invalid state
            01         11            0111          1     CW, 0x07
            10         00            1000          1     CW, 0x08
            10         01            1001          0     invalid state
            10         10            1010          0     stop/idle
            10         11            1011         -1     CCW, 0x0B
            11         00            1100          0     invalid state
            11         01            1101         -1     CCW, 0x0D 
            11         10            1110          1     CW,  0x0E
            11         11            1111          0     stop/idle

          - CW  states 0b0001, 0b0111, 0b1000, 0b1110
          - CCW states 0b0010, 0b0100, 0b1011, 0b1101

   Frameworks & Libraries:
   TimerOne AVR          - https://github.com/PaulStoffregen/TimerOne
   ATtiny  Core          - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core          - https://github.com/espressif/arduino-esp32
   ESP8266 Core          - https://github.com/esp8266/Arduino
   STM32   Core          - https://github.com/rogerclarkmelbourne/Arduino_STM32

   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/
#ifndef DSPControl_h
#define DSPControl_h

#if defined(ARDUINO) && ((ARDUINO) >= 100) //arduino core v1.0 or later
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#if defined(__AVR__)
#include <avr/pgmspace.h>                  //use for PROGMEM Arduino AVR
#elif defined(ESP8266)
#include <pgmspace.h>                      //use for PROGMEM Arduino ESP8266
#elif defined(_VARIANT_ARDUINO_STM32_)
#include <avr/pgmspace.h>                  //use for PROGMEM Arduino STM32
#endif


class DSPControl
{
  public:
    DSP(uint8_t volume, uint8_t channel);

           void     begin(uint8_t channel);
           void     setvolume(uint8_t volume, uint8_t channel);
           void     setmute(uint8_t channel);




__STATIC_INLINE void delay_us(uint32_t us)
{
    uint32_t us_count_tic = us * (SystemCoreClock / 1000000U);
    DWT->CYCCNT = 0U;
    while (DWT->CYCCNT < us_count_tic);
}

  private:
    void shift16(uint32_t DataPin, uint32_t ClockPin, uint16_t Val);
    
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
    __STATIC_INLINE void DWT_Init(void)
    {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // запускаем счётчик
    }

  protected:
    

};

#endif
