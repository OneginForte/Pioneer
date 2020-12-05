#include <Arduino.h>
#include <RotaryEncoder.h>
#include "DSPControl.h"

#define VOL_A PA0   //ky-040 clk pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define VOL_B PA1   //ky-040 dt  pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define ENC1 PA8
#define ENC2 PA9
#define BUTTON PA10 //ky-040 sw  pin, add 100nF/0.1uF capacitors between pin & ground!!!

enum channel_tm : uint16_t
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
};

uint16_t buttonCounter = 0;

//DSPControl(uint8_t dsp_sck, uint8_t dsp_mosi,uint8_t volume, Channel channel);
volatile uint8_t powerstatus = false;

RotaryEncoder encoder_vol(ENC1, ENC2);

//ini DSP pins and default value
#define SPIDSP_SCK PB14
#define SPIDSP_MOSI PB15
uint16_t volumeposition = 48; //default volume 0dB
uint8_t direction;

channel_tm chan = IN10;
//chan = IN10; //default channel

DSPControl DSP(SPIDSP_SCK, SPIDSP_MOSI, volumeposition, chan);

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

uint8_t disp_init1[6] =
    {
        0x05,
        0x11,
        0x14,
        0x01,
        0x2B,
        0x00};

uint8_t disp_init2[20] =
    {
        0x13,
        0x11,
        0x11,
        ' ', 'S', 'v', 'e', 't', 'l', 'a', 'n', 'a', ' ', 'v', '0', '.', '1', //14 letters
        //' ', ' ', ' ', 'P', 'O', 'W', 'E', 'R', ' ', 'O', 'N', ' ', ' ', ' ', //14 letters
        0x00,
        0x3A, //0x3F,
        0x00};

uint8_t disp_init3[10] =
    {
        0x09,
        0x11,
        0x12,
        '2', ' ', '0', '0', //3 numbers, first digit 0-blank, 1-minus, 2-plus
        0x00,
        0xAC,
        0x00};

uint8_t disp_init4[16] =
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
        0x00};

#define DISP_SPI_CS PA15
#define DISP_SPI_MOSI PB5
//#define DISP_SPI_MISO PB4
#define DISP_SPI_SCK PB3
#define DISP_RESET PB6

#define PowerON PB10 //aka RYAC
#define F_RLY PB12
#define SP_B_RLY PB13
#define XSMUTE PB11 //XAMUTE active low in controller, active high in scheme
#define POWERKEY BUTTON
#define POWERLED PC13

struct vlStruct 
    { 
        char message[6];
        uint16_t volumestr;
    };

const vlStruct volume_m[]    //238 values
    {
"+24",0x0300,
"+23,5",0x02F0,
"+23",0x02E0,
"+22,5",0x02D0,
"+22",0x02C0,
"+21,5",0x02B0,
"+21",0x02A0,
"+20,5",0x0290,
"+20",0x0280,
"+19,5",0x0270,
"+19",0x0260,
"+18,5",0x0250,
"+18",0x0240,
"+17,5",0x0230,
"+17",0x0220,
"+16,5",0x0210,
"+16",0x0200,
"+15,5",0x01F0,
"+15",0x01E0,
"+14,5",0x01D0,
"+14",0x01C0,
"+13,5",0x01B0,
"+13",0x01A0,
"+12,5",0x0190,
"+12",0x0180,
"+11,5",0x0170,
"+11",0x0160,
"+10,5",0x0150,
"+10",0x0140,
"+9,5",0x0130,
"+9",0x0120,
"+8,5",0x0110,
"+8",0x0100,
"+7,5",0x00F0,
"+7",0x00E0,
"+6,5",0x00D0,
"+6",0x00C0,
"+5,5",0x00B0,
"+5",0x00A0,
"+4,5",0x0090,
"+4",0x0080,
"+3,5",0x0070,
"+3",0x0060,
"+2,5",0x0050,
"+2",0x0040,
"+1,5",0x0030,
"+1",0x0020,
"+0,5",0x0010,
"0",0x0000,
"-0,5",0x0010,
"-1",0x0020,
"-1,5",0x0030,
"-2",0x0040,
"-2,5",0x0050,
"-3",0x0060,
"-3,5",0x0070,
"-4",0x0080,
"-4,5",0x0090,
"-5",0x00A0,
"-5,5",0x00B0,
"-6",0x00C0,
"-6,5",0x00D0,
"-7",0x00E0,
"-7,5",0x00F0,
"-8",0x0100,
"-8,5",0x0110,
"-9",0x0120,
"-9,5",0x0130,
"-10",0x0140,
"-10,5",0x0150,
"-11",0x0160,
"-11,5",0x0170,
"-12",0x0180,
"-12,5",0x0190,
"-13",0x01A0,
"-13,5",0x01B0,
"-14",0x01C0,
"-14,5",0x01D0,
"-15",0x01E0,
"-15,5",0x01F0,
"-16",0x0200,
"-16,5",0x0210,
"-17",0x0220,
"-17,5",0x0230,
"-18",0x0240,
"-18,5",0x0250,
"-19",0x0260,
"-19,5",0x0270,
"-20",0x0280,
"-20,5",0x0290,
"-21",0x02A0,
"-21,5",0x02B0,
"-22",0x02C0,
"-22,5",0x02D0,
"-23",0x02E0,
"-23,5",0x02F0,
"-24",0x0300,
"-24,5",0x0310,
"-25",0x0320,
"-25,5",0x0330,
"-26",0x0340,
"-26,5",0x0350,
"-27",0x0360,
"-27,5",0x0370,
"-28",0x0380,
"-28,5",0x0390,
"-29",0x03A0,
"-29,5",0x03B0,
"-30",0x03C0,
"-30,5",0x03D0,
"-31",0x03E0,
"-31,5",0x03F0,
"-32",0x0400,
"-32,5",0x0410,
"-33",0x0420,
"-33,5",0x0430,
"-34",0x0440,
"-34,5",0x0450,
"-35",0x0460,
"-35,5",0x0470,
"-36",0x0480,
"-36,5",0x0490,
"-37",0x04A0,
"-37,5",0x04B0,
"-38",0x04C0,
"-38,5",0x04D0,
"-39",0x04E0,
"-39,5",0x04F0,
"-40",0x0500,
"-40,5",0x0510,
"-41",0x0520,
"-41,5",0x0530,
"-42",0x0540,
"-42,5",0x0550,
"-43",0x0560,
"-43,5",0x0570,
"-44",0x0580,
"-44,5",0x0590,
"-45",0x05A0,
"-45,5",0x05B0,
"-46",0x05C0,
"-46,5",0x05D0,
"-47",0x05E0,
"-47,5",0x05F0,
"-48",0x0600,
"-48,5",0x0610,
"-49",0x0620,
"-49,5",0x0630,
"-50",0x0640,
"-50,5",0x0650,
"-51",0x0660,
"-51,5",0x0670,
"-52",0x0680,
"-52,5",0x0690,
"-53",0x06A0,
"-53,5",0x06B0,
"-54",0x06C0,
"-54,5",0x06D0,
"-55",0x06E0,
"-55,5",0x06F0,
"-56",0x0700,
"-56,5",0x0710,
"-57",0x0720,
"-57,5",0x0730,
"-58",0x0740,
"-58,5",0x0750,
"-59",0x0760,
"-59,5",0x0770,
"-60",0x0780,
"-60,5",0x0790,
"-61",0x07A0,
"-61,5",0x07B0,
"-62",0x07C0,
"-62,5",0x07D0,
"-63",0x07E0,
"-63,5",0x07F0,
"-64",0x0800,
"-64,5",0x0810,
"-65",0x0820,
"-65,5",0x0830,
"-66",0x0840,
"-66,5",0x0850,
"-67",0x0860,
"-67,5",0x0870,
"-68",0x0880,
"-68,5",0x0890,
"-69",0x08A0,
"-69,5",0x08B0,
"-70",0x08C0,
"-70,5",0x08D0,
"-71",0x08E0,
"-71,5",0x08F0,
"-72",0x0900,
"-72,5",0x0910,
"-73",0x0920,
"-73,5",0x0930,
"-74",0x0940,
"-74,5",0x0950,
"-75",0x0960,
"-75,5",0x0970,
"-76",0x0980,
"-76,5",0x0990,
"-77",0x09A0,
"-77,5",0x09B0,
"-78",0x09C0,
"-78,5",0x09D0,
"-79",0x09E0,
"-79,5",0x09F0,
"-80",0x0A00,
"-80,5",0x0A10,
"-81",0x0A20,
"-81,5",0x0A30,
"-82",0x0A40,
"-82,5",0x0A50,
"-83",0x0A60,
"-83,5",0x0A70,
"-84",0x0A80,
"-84,5",0x0A90,
"-85",0x0AA0,
"-85,5",0x0AB0,
"-86",0x0AC0,
"-86,5",0x0AD0,
"-87",0x0AE0,
"-87,5",0x0AF0,
"-88",0x0B00,
"-88,5",0x0B10,
"-89",0x0B20,
"-89,5",0x0B30,
"-90",0x0B40,
"-90,5",0x0B50,
"-91",0x0B60,
"-91,5",0x0B70,
"-92",0x0B80,
"-92,5",0x0B90,
"-93",0x0BA0,
"-93,5",0x0BB0,
"-94",0x0BC0,
"94,5",0x0BD0,
"95",0x0BE0
    };
