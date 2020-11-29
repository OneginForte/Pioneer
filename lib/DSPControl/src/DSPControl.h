/***************************************************************************************************/
/*
   This is an Arduino library for 

   written by : enjoyneering79
   sourse code: https://github.com/enjoyneering/



   NOTE:


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

#include <Arduino.h>


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
          DSPControl(uint8_t dsp_sck, uint8_t dsp_mosi,uint8_t volume, uint8_t channel);
           void     begin(void);
           void     setvolume(uint8_t volume, uint8_t channel);
           void     setmute(uint8_t channel);

            __STATIC_INLINE void delay_us(uint32_t us)
            {
            uint32_t us_count_tic = us * (SystemCoreClock / 1000000U);
            DWT->CYCCNT = 0U;
            while (DWT->CYCCNT < us_count_tic);
            }

  private:
            uint8_t _dsp_sck;
            uint8_t _dsp_mosi;
            uint8_t _volume;
            uint8_t _channel; 

            void shift16(uint32_t DataPin, uint32_t ClockPin, uint16_t Val);
   
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

  protected:
    

};

#endif
