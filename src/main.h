//#include <RotaryEncoder.h>
//#include "DSPControl.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_gpio.h"


#define VOL_A PA0   //ky-040 clk pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define VOL_B PA1   //ky-040 dt  pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define ENC1 PA8
#define ENC2 PA9
#define BUTTON PA10 //ky-040 sw  pin, add 100nF/0.1uF capacitors between pin & ground!!!

#define DISP_SPI_CS PA15
#define DISP_SPI_MOSI PB5
//#define DISP_SPI_MISO PB4
#define DISP_SPI_SCK PB3
#define DISP_RESET PB6

#define PowerON PB10 //aka RYAC
#define F_RLY PB12
#define SP_B_RLY PB13
#define SMUTE PB11 //XAMUTE active low in controller, active high in scheme
#define POWERKEY BUTTON
#define POWERLED PC13

uint8_t volumeposition = 79; //default volume 0dB
uint8_t oldvolumeposition;

//ini DSP pins and default value
#define SPIDSP_SCK PB14
#define SPIDSP_MOSI PB15

DSPControl DSP(SPIDSP_SCK, SPIDSP_MOSI);

enum speaker_t : uint8_t
{
SPA=1,
SPB=2,
SPAB=3,
SPKOFF=4
};

enum input_t : uint16_t
{
    CD = DSPControl::channel_tm::IN10,
    DVD = DSPControl::channel_tm::IN7,
    DVR = DSPControl::channel_tm::IN12,
    SATIN = DSPControl::channel_tm::IN9,
    AATUN = DSPControl::channel_tm::IN6,
    XML = DSPControl::channel_tm::IN2,
    BT = DSPControl::channel_tm::IN5,
    MIC = DSPControl::channel_tm::IN1
};

DSPControl::channel_tm chan = (DSPControl::channel_tm)CD;

struct inputStruct
{
    char message[6];
    input_t input;
};

inputStruct inp_select[8] =
    {
        {" CD  ", CD},
        {" DVD ", DVD},
        {" DVR ", DVR},
        {" SAT ", SATIN},
        {"TUNER", AATUN},
        {"ZONE ", XML},
        {" BT  ", BT},
        {" MIC ", MIC}
    };

uint16_t buttonCounter = 0;

//DSPControl(uint8_t dsp_sck, uint8_t dsp_mosi,uint8_t volume, Channel channel);

enum power_t : uint8_t
{
        ON=0,
        OFF=1,
        MUTE=2,
        SLEEEP=3
};

volatile uint8_t powerstatus = OFF;

volatile speaker_t speakerstatus = SPKOFF;

RotaryEncoder encoder(ENC1, ENC2);


uint8_t direction;
uint16_t selector;



void encoderISR()
{
    encoder.readAB();
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
        0x01
    };

uint8_t disp_init2[] =
    {
        0x13,
        0x11,
        0x11,
        ' ', 'P', 'I', 'O', 'B', 'E', 'E', 'R', ' ', 'W', 'O', 'R', 'K', ' ', //14 letters
        //' ', ' ', ' ', 'P', 'O', 'W', 'E', 'R', ' ', 'O', 'N', ' ', ' ', ' ', //14 letters
        0x00
    };

uint8_t disp_init3[] =
    {
        0x09,
        0x11,
        0x12,
        '0', ' ', ' ', ' ', //3 numbers, first digit 0-blank, 1-minus, 2-plus, 2 and 3 digit can take values from 0 to 99
                            //last git 0 - 0db, 5 - 5db, 1 - blank.
        0x00};

uint8_t disp_init4[] =
    {
        0x0F,
        0x11,
        0x13,
        0b00000000, //0b11111000  PQLS / S.RTRV / MULTI-ZONE / PCM / DSD
        0b00000000, //0b11111111  OVER / ATT / pict HD) / pict right fullband / UP MIX / ALC / SOUND / FULL BAND
        0b00000010, //0b11111111  pict \/ / pict /\ / iPod / SIRIUS / MUTE pict / MONO / STEREO / TUNED
        0b00000000, //0b11111000  PLUS / WMA9pro / ES / AAC / 96/24
        0b00000000, //0b11111111  _XR / _XC / _XL / MSTR / DDdigital / DDTrueHD / DTS / HD /
        0b00001010, //0b11111111  PCM / AUTO / (( around lfe )) / LFE / _L / _C / _R / _SL_SR
        0b00000000, //0b11111111  NEO:6 / x left of NEO / DD PrologicII / STREAM DIRECT / ANALOG / DIGITAL / AUTO SURROUND / HDMI
        0b00110100, //0b11111111  SLEEP / right of SP B / A / SP> / STANDARD / STEREO / ADV.SURROUND / THX
        0b00000000, //0b11111111  over text  XM / [4] / USB / [3] / [2] / HMG / VIDEO
        0b00000001, //0b11111111  DVR / TV / BD / DVD / PHONO / TUNER / CD-R / CD
        0x00};



struct vlStruct 
    { 
        char message1[6];
        char message2[5];
        uint16_t voldsp;
    };

    const vlStruct volume_m[] //238 values
        {
            "+24  ", "2240", 0x1300,
            "+23,5", "2235", 0x12F0,
            "+23  ", "2230", 0x12E0,
            "+22,5", "2225", 0x12D0,
            "+22  ", "2220", 0x12C0,
            "+21,5", "2215", 0x12B0,
            "+21  ", "2210", 0x12A0,
            "+20,5", "2205", 0x1290,
            "+20  ", "2200", 0x1280,
            "+19,5", "2195", 0x1270,
            "+19  ", "2190", 0x1260,
            "+18,5", "2185", 0x1250,
            "+18  ", "2180", 0x1240,
            "+17,5", "2175", 0x1230,
            "+17  ", "2170", 0x1220,
            "+16,5", "2165", 0x1210,
            "+16  ", "2160", 0x1200,
            "+15,5", "2155", 0x11F0,
            "+15  ", "2150", 0x11E0,
            "+14,5", "2145", 0x11D0,
            "+14  ", "2140", 0x11C0,
            "+13,5", "2135", 0x11B0,
            "+13  ", "2130", 0x11A0,
            "+12,5", "2125", 0x1190,
            "+12  ", "2120", 0x1180,
            "+11,5", "2115", 0x1170,
            "+11  ", "2110", 0x1160,
            "+10,5", "2105", 0x1150,
            "+10  ", "2100", 0x1140,
            "+9,5 ", "2 95", 0x1130,
            "+9   ", "2 90", 0x1120,
            "+8,5 ", "2 85", 0x1110,
            "+8   ", "2 80", 0x1100,
            "+7,5 ", "2 75", 0x10F0,
            "+7   ", "2 70", 0x10E0,
            "+6,5 ", "2 65", 0x10D0,
            "+6   ", "2 60", 0x10C0,
            "+5,5 ", "2 55", 0x10B0,
            "+5   ", "2 50", 0x10A0,
            "+4,5 ", "2 45", 0x1090,
            "+4   ", "2 40", 0x1080,
            "+3,5 ", "2 35", 0x1070,
            "+3   ", "2 30", 0x1060,
            "+2,5 ", "2 25", 0x1050,
            "+2   ", "2 20", 0x1040,
            "+1,5 ", "2 15", 0x1030,
            "+1   ", "2 10", 0x1020,
            "+0,5 ", "2 05", 0x1010,
            "0    ", "  00", 0x0000,
            "-0,5 ", "1 05", 0x0010,
            "-1   ", "1 10", 0x0020,
            "-1,5 ", "1 15", 0x0030,
            "-2   ", "1 20", 0x0040,
            "-2,5 ", "1 25", 0x0050,
            "-3   ", "1 30", 0x0060,
            "-3,5 ", "1 35", 0x0070,
            "-4   ", "1 40", 0x0080,
            "-4,5 ", "1 45", 0x0090,
            "-5   ", "1 50", 0x00A0,
            "-5,5 ", "1 55", 0x00B0,
            "-6   ", "1 60", 0x00C0,
            "-6,5 ", "1 65", 0x00D0,
            "-7   ", "1 70", 0x00E0,
            "-7,5 ", "1 75", 0x00F0,
            "-8   ", "1 80", 0x0100,
            "-8,5 ", "1 85", 0x0110,
            "-9   ", "1 90", 0x0120,
            "-9,5 ", "1 95", 0x0130,
            "-10  ", "1100", 0x0140,
            "-10,5", "1105", 0x0150,
            "-11  ", "1110", 0x0160,
            "-11,5", "1115", 0x0170,
            "-12  ", "1120", 0x0180,
            "-12,5", "1125", 0x0190,
            "-13  ", "1130", 0x01A0,
            "-13,5", "1135", 0x01B0,
            "-14  ", "1140", 0x01C0,
            "-14,5", "1145", 0x01D0,
            "-15  ", "1150", 0x01E0,
            "-15,5", "1155", 0x01F0,
            "-16  ", "1160", 0x0200,
            "-16,5", "1165", 0x0210,
            "-17  ", "1170", 0x0220,
            "-17,5", "1175", 0x0230,
            "-18  ", "1180", 0x0240,
            "-18,5", "1185", 0x0250,
            "-19  ", "1190", 0x0260,
            "-19,5", "1195", 0x0270,
            "-20  ", "1200", 0x0280,
            "-20,5", "1205", 0x0290,
            "-21  ", "1210", 0x02A0,
            "-21,5", "1215", 0x02B0,
            "-22  ", "1220", 0x02C0,
            "-22,5", "1225", 0x02D0,
            "-23  ", "1230", 0x02E0,
            "-23,5", "1235", 0x02F0,
            "-24  ", "1240", 0x0300,
            "-24,5", "1245", 0x0310,
            "-25  ", "1250", 0x0320,
            "-25,5", "1255", 0x0330,
            "-26  ", "1260", 0x0340,
            "-26,5", "1265", 0x0350,
            "-27  ", "1270", 0x0360,
            "-27,5", "1275", 0x0370,
            "-28  ", "1280", 0x0380,
            "-28,5", "1285", 0x0390,
            "-29  ", "1290", 0x03A0,
            "-29,5", "1295", 0x03B0,
            "-30  ", "1300", 0x03C0,
            "-30,5", "1305", 0x03D0,
            "-31  ", "1310", 0x03E0,
            "-31,5", "1315", 0x03F0,
            "-32  ", "1320", 0x0400,
            "-32,5", "1325", 0x0410,
            "-33  ", "1330", 0x0420,
            "-33,5", "1335", 0x0430,
            "-34  ", "1340", 0x0440,
            "-34,5", "1345", 0x0450,
            "-35  ", "1350", 0x0460,
            "-35,5", "1355", 0x0470,
            "-36  ", "1360", 0x0480,
            "-36,5", "1365", 0x0490,
            "-37  ", "1370", 0x04A0,
            "-37,5", "1375", 0x04B0,
            "-38  ", "1380", 0x04C0,
            "-38,5", "1385", 0x04D0,
            "-39  ", "1390", 0x04E0,
            "-39,5", "1395", 0x04F0,
            "-40  ", "1400", 0x0500,
            "-40,5", "1405", 0x0510,
            "-41  ", "1410", 0x0520,
            "-41,5", "1415", 0x0530,
            "-42  ", "1420", 0x0540,
            "-42,5", "1425", 0x0550,
            "-43  ", "1430", 0x0560,
            "-43,5", "1435", 0x0570,
            "-44  ", "1440", 0x0580,
            "-44,5", "1445", 0x0590,
            "-45  ", "1450", 0x05A0,
            "-45,5", "1455", 0x05B0,
            "-46  ", "1460", 0x05C0,
            "-46,5", "1465", 0x05D0,
            "-47  ", "1470", 0x05E0,
            "-47,5", "1475", 0x05F0,
            "-48  ", "1480", 0x0600,
            "-48,5", "1485", 0x0610,
            "-49  ", "1490", 0x0620,
            "-49,5", "1495", 0x0630,
            "-50  ", "1500", 0x0640,
            "-50,5", "1505", 0x0650,
            "-51  ", "1510", 0x0660,
            "-51,5", "1515", 0x0670,
            "-52  ", "1520", 0x0680,
            "-52,5", "1525", 0x0690,
            "-53  ", "1530", 0x06A0,
            "-53,5", "1535", 0x06B0,
            "-54  ", "1540", 0x06C0,
            "-54,5", "1545", 0x06D0,
            "-55  ", "1550", 0x06E0,
            "-55,5", "1555", 0x06F0,
            "-56  ", "1560", 0x0700,
            "-56,5", "1565", 0x0710,
            "-57  ", "1570", 0x0720,
            "-57,5", "1575", 0x0730,
            "-58  ", "1580", 0x0740,
            "-58,5", "1585", 0x0750,
            "-59  ", "1590", 0x0760,
            "-59,5", "1595", 0x0770,
            "-60  ", "1600", 0x0780,
            "-60,5", "1605", 0x0790,
            "-61  ", "1610", 0x07A0,
            "-61,5", "1615", 0x07B0,
            "-62  ", "1620", 0x07C0,
            "-62,5", "1625", 0x07D0,
            "-63  ", "1630", 0x07E0,
            "-63,5", "1635", 0x07F0,
            "-64  ", "1640", 0x0800,
            "-64,5", "1645", 0x0810,
            "-65  ", "1650", 0x0820,
            "-65,5", "1655", 0x0830,
            "-66  ", "1660", 0x0840,
            "-66,5", "1665", 0x0850,
            "-67  ", "1670", 0x0860,
            "-67,5", "1675", 0x0870,
            "-68  ", "1680", 0x0880,
            "-68,5", "1685", 0x0890,
            "-69  ", "1690", 0x08A0,
            "-69,5", "1695", 0x08B0,
            "-70  ", "1700", 0x08C0,
            "-70,5", "1705", 0x08D0,
            "-71  ", "1710", 0x08E0,
            "-71,5", "1715", 0x08F0,
            "-72  ", "1720", 0x0900,
            "-72,5", "1725", 0x0910,
            "-73  ", "1730", 0x0920,
            "-73,5", "1735", 0x0930,
            "-74  ", "1740", 0x0940,
            "-74,5", "1745", 0x0950,
            "-75  ", "1750", 0x0960,
            "-75,5", "1755", 0x0970,
            "-76  ", "1760", 0x0980,
            "-76,5", "1765", 0x0990,
            "-77  ", "1770", 0x09A0,
            "-77,5", "1775", 0x09B0,
            "-78  ", "1780", 0x09C0,
            "-78,5", "1785", 0x09D0,
            "-79  ", "1790", 0x09E0,
            "-79,5", "1795", 0x09F0,
            "-80  ", "1800", 0x0A00,
            "-80,5", "1805", 0x0A10,
            "-81  ", "1810", 0x0A20,
            "-81,5", "1815", 0x0A30,
            "-82  ", "1820", 0x0A40,
            "-82,5", "1825", 0x0A50,
            "-83  ", "1830", 0x0A60,
            "-83,5", "1835", 0x0A70,
            "-84  ", "1840", 0x0A80,
            "-84,5", "1845", 0x0A90,
            "-85  ", "1850", 0x0AA0,
            "-85,5", "1855", 0x0AB0,
            "-86  ", "1860", 0x0AC0,
            "-86,5", "1865", 0x0AD0,
            "-87  ", "1870", 0x0AE0,
            "-87,5", "1875", 0x0AF0,
            "-88  ", "1880", 0x0B00,
            "-88,5", "1885", 0x0B10,
            "-89  ", "1890", 0x0B20,
            "-89,5", "1895", 0x0B30,
            "-90  ", "1900", 0x0B40,
            "-90,5", "1905", 0x0B50,
            "-91  ", "1910", 0x0B60,
            "-91,5", "1915", 0x0B70,
            "-92  ", "1920", 0x0B80,
            "-92,5", "1925", 0x0B90,
            "-93  ", "1930", 0x0BA0,
            "-93,5", "1935", 0x0BB0,
            "-94  ", "1940", 0x0BC0,
            "-94,5", "1945", 0x0BD0,
            "-95  ", "1950", 0x0BE0

        };
