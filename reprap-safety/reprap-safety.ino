
/*
 PROGMEM string demo
 How to store a table of strings in program memory (flash),
 and retrieve them.

 Information summarized from:
 http://www.nongnu.org/avr-libc/user-manual/pgmspace.html

 Setting up a table (array) of strings in program memory is slightly complicated, but
 here is a good template to follow.

 Setting up the strings is a two-step process. First define the strings.
*/

#include <avr/pgmspace.h>

#define OVERSAMPLENR 1
#define PGM_RD_W(x)   (short)pgm_read_word(&x)


const short temptable[][2] PROGMEM = {
  {23*OVERSAMPLENR,    341},
  {25*OVERSAMPLENR,    333},
  {27*OVERSAMPLENR,    325},
  {28*OVERSAMPLENR,    322},
  {31*OVERSAMPLENR,    313},
  {33*OVERSAMPLENR,    307},
  {35*OVERSAMPLENR,    302},
  {38*OVERSAMPLENR,    295},
  {41*OVERSAMPLENR,    288},
  {44*OVERSAMPLENR,    282},
  {48*OVERSAMPLENR,    275},
  {52*OVERSAMPLENR,    269},
  {56*OVERSAMPLENR,    264},
  {61*OVERSAMPLENR,    257},
  {66*OVERSAMPLENR,    251},
  {71*OVERSAMPLENR,    246},
  {78*OVERSAMPLENR,    239},
  {84*OVERSAMPLENR,    233},
  {92*OVERSAMPLENR,    227},
  {100*OVERSAMPLENR,    221},
  {109*OVERSAMPLENR,    216},
  {120*OVERSAMPLENR,    209},
  {131*OVERSAMPLENR,    203},
  {143*OVERSAMPLENR,    198},
  {156*OVERSAMPLENR,    192},
  {171*OVERSAMPLENR,    186},
  {187*OVERSAMPLENR,    180},
  {205*OVERSAMPLENR,    174},
  {224*OVERSAMPLENR,    169},
  {245*OVERSAMPLENR,    163},
  {268*OVERSAMPLENR,    157},
  {293*OVERSAMPLENR,    152},
  {320*OVERSAMPLENR,    146},
  {348*OVERSAMPLENR,    141},
  {379*OVERSAMPLENR,    135},
  {411*OVERSAMPLENR,    129},
  {445*OVERSAMPLENR,    124},
  {480*OVERSAMPLENR,    118},
  {516*OVERSAMPLENR,    113},
  {553*OVERSAMPLENR,    108},
  {591*OVERSAMPLENR,    102},
  {628*OVERSAMPLENR,     97},
  {665*OVERSAMPLENR,     92},
  {702*OVERSAMPLENR,     86},
  {737*OVERSAMPLENR,     81},
  {770*OVERSAMPLENR,     76},
  {801*OVERSAMPLENR,     71},
  {830*OVERSAMPLENR,     65},
  {857*OVERSAMPLENR,     60},
  {881*OVERSAMPLENR,     55},
  {903*OVERSAMPLENR,     50},
  {922*OVERSAMPLENR,     45},
  {939*OVERSAMPLENR,     40},
  {954*OVERSAMPLENR,     35},
  {966*OVERSAMPLENR,     30},
  {977*OVERSAMPLENR,     25},
  {985*OVERSAMPLENR,     21},
  {993*OVERSAMPLENR,     16},
  {999*OVERSAMPLENR,     11},
  {1004*OVERSAMPLENR,      6},
  {1008*OVERSAMPLENR,      0}
};

# define heater_ttbllen_map (sizeof(temptable)/sizeof(*temptable))

short (*tt)[][2] = (short (*)[][2])(temptable);
int raw = 0;

float celsius = 0;
uint8_t i;

int analogPin = 0;


void setup()
{
  Serial.begin(9600);
  while(!Serial);
  Serial.println("OK");
}


void loop()
{
  /* Using the string table in program memory requires the use of special functions to retrieve the data.
     The strcpy_P function copies a string from program space to a string in RAM ("buffer").
     Make sure your receiving string in RAM  is large enough to hold whatever
     you are retrieving from program space. */
     raw = analogRead(analogPin);    // read the input pin
   for (i=1; i<heater_ttbllen_map; i++)
    {
      if (PGM_RD_W((*tt)[i][0]) > raw)
      {
        celsius = PGM_RD_W((*tt)[i-1][1]) + 
          (raw - PGM_RD_W((*tt)[i-1][0])) * 
          (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i-1][1])) /
          (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map) celsius = PGM_RD_W((*tt)[i-1][1]);

    
    Serial.println(celsius);
    delay( 500 );
  
}

/*




// MODIFIED FROM MARLIN https://github.com/MarlinFirmware/Marlin
// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
static float analog2temp(int raw, uint8_t e) {

  if(heater_ttbl_map[e] != NULL)
  {
    float celsius = 0;
    uint8_t i;
    short (*tt)[][2] = (short (*)[][2])(heater_ttbl_map[e]);

    for (i=1; i<heater_ttbllen_map[e]; i++)
    {
      if (PGM_RD_W((*tt)[i][0]) > raw)
      {
        celsius = PGM_RD_W((*tt)[i-1][1]) + 
          (raw - PGM_RD_W((*tt)[i-1][0])) * 
          (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i-1][1])) /
          (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]) celsius = PGM_RD_W((*tt)[i-1][1]);

    return celsius;
  }
  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
}
*/
