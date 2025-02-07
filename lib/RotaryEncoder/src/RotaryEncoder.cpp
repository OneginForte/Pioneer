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

#include "RotaryEncoder.h"


/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/
RotaryEncoder::RotaryEncoder(uint8_t encoderA, uint8_t encoderB)
{
  _encoderA      = encoderA;
  _encoderB      = encoderB;

}

/**************************************************************************/
/*
    begin

    Sets encoders pins as input & enables built-in pullup resistors

    NOTE:
    - high value of pull-up resistor limits the speed
    - typical value of external pull-up resistor is 10kOhm, acceptable
      range 5kOhm..100kOhm
    - for ESP8266 value of internal pull-up resistors is 30kOhm..100kOhm
    - for AVR     value of internal pull-up resistors is 30kOhm..60kOhm
*/
/**************************************************************************/
void RotaryEncoder::begin()
{
  pinMode(_encoderA,      INPUT_PULLUP); //enable internal pull-up resistors 
  pinMode(_encoderB,      INPUT_PULLUP);

}

/**************************************************************************/
/*
    readAB()

    Reads "A" & "B" pins value

    NOTE:
    - always call this function before getPosition()
    - 100nF/0.1μF capacitors between A & B channel pin & ground is a must!!!
    - for fast MCU like Cortex use Interrupt Service Routine with "CHANGE"
      parameter, ISR called when pin "A" changes from "1" to "0"
      or from "0" to "1"
    - for slow MCU like 8-bit AVR use Timer1 interrupt & TimerOne library
    - the ISR function must take no parameters & return nothing
    - delay() doesn't work during ISR & millis() doesn't increment
    - declare all global variables inside ISR as "volatile", it prevent
      compiler to make any optimization & unnecessary changes in the code
      with the variable
*/
/**************************************************************************/
void RotaryEncoder::readAB()
{
  noInterrupts();                                       //disable interrupts

  _currValueAB  = digitalRead(_encoderA) << 1;
  _currValueAB |= digitalRead(_encoderB);

  switch ((_prevValueAB | _currValueAB))
  {

    case 0b1110: case 0b1100: case 0b0011: case 0b1000: case 0b0001: //CW states, 1 count  per click

      _counter++;
      if (_counter > 239)
      {
        _counter = 239;
      }
      break;

    case 0b1001: case 0b1101: case 0b1010: case 0b0100: case 0b0110: //CCW states, 1 count  per click
      _counter--;
      if (_counter < 1)
      {
        _counter = 1;
      }
      break;
  }




  _prevValueAB = _currValueAB << 2;                     //update previouse state

  interrupts();                                         //enable interrupts
}


/**************************************************************************/
/*
    getPosition()

    Return encoder position
*/
/**************************************************************************/
int16_t RotaryEncoder::getPosition()
{
  return _counter;
}


/**************************************************************************/
/*
    setPosition()

    Manualy sets encoder position
*/
/**************************************************************************/
void RotaryEncoder::setPosition(int16_t position)
{
  _counter = position;
}

