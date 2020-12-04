/***************************************************************************************************/
/*
   This is an Arduino library for 

   written by : OneginForte
   sourse code: https://github.com/OneginForte/



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
    DSPControl(uint32_t dsp_sck, uint32_t dsp_mosi, uint16_t volume, uint16_t channelin);

    void begin(void);
    void setvolume(uint8_t volume, uint8_t channelin);
    void setmute(uint8_t channelin);

    __STATIC_INLINE void delay_us(uint32_t us)
    {
      uint32_t us_count_tic = us * (SystemCoreClock / 1000000U);
      DWT->CYCCNT = 0U;
      while (DWT->CYCCNT < us_count_tic);
     }

  private:
    typedef enum Channel
    {
      IN1 = 0x0400,
      IN2 = 0x0800,
      IN3 = 0x0C00,
      IN4 = 0x1000,
      IN5 = 0x1400,
      IN6 = 0x1800,
      IN7 = 0x1C00,
      IN8 = 0x2000,
      IN9 = 0x2400,
      IN10 = 0x2800,
      IN11 = 0x2C00,
      IN12 = 0x3000,
      IN13 = 0x3400,
      IN14 = 0x3800,
      IN15 = 0x3C00,
      IN16 = 0x4000,
      IN17 = 0x4400,
      IN18 = 0x4800,
    } channel_t;

    void shift16(uint32_t DataPin, uint32_t ClockPin, uint16_t Val);

    uint32_t _dsp_sck;
    uint32_t _dsp_mosi;
    uint16_t _volume;
    channel_t _channel;

    typedef struct
    {
      channel_t channels;
      uint16_t chncode;
	} volume_t;

  volume_t volume_m[18] = {
	{ IN1, 0x0400 },
  { IN2, 0x0800 },
  { IN3, 0x0C00 },
  { IN4, 0x1000 },
  { IN5, 0x1400 },
  { IN6, 0x1800 },
  { IN7, 0x1C00 },
  { IN8, 0x2000 },
  { IN9, 0x2400 },
  { IN10, 0x2800 },
  { IN11, 0x2C00 },
  { IN12, 0x3000 },
  { IN13, 0x3400 },
  { IN14, 0x3800 },
  { IN15, 0x3C00 },
  { IN16, 0x4000 },
  { IN17, 0x4400 },
  { IN18, 0x4800 }
  };
            
  
   
  protected:


};

#endif
