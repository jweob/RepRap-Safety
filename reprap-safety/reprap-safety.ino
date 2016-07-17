/*
RepRap safety cutoff
Turns off the power supply to a RepRap using a relay if any of the following three conditions are met
1. Secondary thermistor reads a temperature above a set threshold
2. Bed heat signal is on for more than a set period of time
3. Hote end is on for more than a set period of time

John O'Brien jweob@cantab.net 2016-0716
*/

#include <avr/pgmspace.h>

// Macros
#define BED_TIME_THRESH 600000 // If heated bed is on for longer than this printer will shut down
#define EXT_TIME_THRESH 60000 // If extruder is on for longer than this printer will shut down
#define EXT_TEMP_THRESH 250 // If the hotend temperature exceeds this value the printer will shut down
#define OPEN_CIRCUIT_TEMP 5 // Open circuit will read as 0C. Short circuit will read as a negative temperature Turn printer off if measured temperature goes below  this temperature


#define OVERSAMPLEBITS 4 // additional bits of precision required for temperature sampling (n)
#define OVERSAMPLENR 16 // 2^n where n is the additional bits of precision required
#define OVERSAMPLES 256 // 4^n where n is the additional bits of precision required

#define PGM_RD_W(x)   (short)pgm_read_word(&x) // Macro for reading the thermistor table
#define heater_ttbllen_map (sizeof(temptable)/sizeof(*temptable))


unsigned long raw = 0; // Holds the output of the oversample function
unsigned long currentMillis = 0; // Captures the current milliseconds since the arduino was switched on
unsigned long bedStartMillis = 0; // Captures when the bed was last turned on
unsigned long extStartMillis = 0; // Captures when the hotend was last turned on
boolean previousBedOn = false; // Was the bed on the last time the arduino checked
boolean previousExtOn = false; // Was the hotend on the last time the arduino checked
const unsigned long bedInterval = BED_TIME_THRESH; // If the interval between the last time the bed was switched on and the current milliseconds is greater than this, switch off
const unsigned long extInterval = EXT_TIME_THRESH; // Likewise for hotend
float celsius = 0; // Holds the output of analog2temp
const float tempThresh = EXT_TEMP_THRESH; // Holds the temperature threshold for cutoff

// Printer currently only has two states, could add more e.g. startup if necessary
enum printerStates {
  OK,
  PROBLEM
};

// Current and previous states
int currentState = OK;
int oldState = OK;

// Pin definitions
int extThermPin = 0; // Output from the potential divider circuit which includes the thermistor
int relayPin = 8; // Pin attached to relay driver circuit. Low signal activates relay and turns on printer
int goodLedPin = 9; // Green LED to indicate printer in "OK" state
int badLedPin = 10; // Red LED to indicate something printer in "PROBLEM" state
int bedSignalPin = 11; // The logical signal coming from the Melzi which activates the heated bed. High means the bed is on
int exPosPin = 6; // The hotend heater is driven by the voltage across two terminals on the Melzi. When the heater is off both terminals are at ~19V.
int exNegPin = 7; // When the heater is on the negative pin is driven to ground
// We need to measure both pins so that we can tell the difference between there being no power to the printer (both pins low) and the hotend being on (positive pin high, negative low)

// Function signatures
static unsigned long oversample(int pin); // Oversamples an analog pin to get better precision
static float analog2temp(unsigned long raw); // Converts the reading from the oversample function into degrees celsius
void updatePins(); // Updates the relay and LED indicator pins based on the current state of the printer

// Thermistor table. This table is setup for the Extruder thermistor: Digikey 480-3137-ND which should be included in All Huxleys shipped after 25/2/14
// Table was generated using the RepRapProFormula
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


// Pointer to thermistor table
short (*tt)[][2] = (short (*)[][2])(temptable);

void setup()
{
  Serial.begin(9600);
  while(!Serial);
  Serial.println("OK");
  pinMode(relayPin, OUTPUT);
  pinMode(goodLedPin, OUTPUT);
  pinMode(badLedPin, OUTPUT);
  pinMode(bedSignalPin, INPUT);
  pinMode(exPosPin, INPUT);
  pinMode(exNegPin, INPUT);
  updatePins();
}


void loop()
{
  currentMillis = millis(); // Get current time
    
  raw = oversample(extThermPin);    // read the input pin
  celsius = analog2temp(raw);
  Serial.println(celsius);
  
  // Check for overtemperature
  if(celsius > tempThresh){
    Serial.print("Temperature exceeded with temp of ");
    Serial.print(celsius);
    Serial.print(" degC");
    currentState = PROBLEM;
  }
  
  // Check for open or closed circuits
  if(celsius < OPEN_CIRCUIT_TEMP){
    Serial.println("Short or open circuit");
    currentState = PROBLEM;
  }

  // Check whether bed has been on too long
  if (digitalRead(bedSignalPin)==HIGH){  
    Serial.print("Bed on for ");
    Serial.println(currentMillis - bedStartMillis);
    if (previousBedOn) {
      if (currentMillis - bedStartMillis > bedInterval) {
          Serial.println("Bed heater on too long");
          currentState = PROBLEM;
        }
      }
    else {
      previousBedOn = true;
      bedStartMillis = currentMillis;
    }
  }
  else {
    Serial.println("Bed off");
    if (previousBedOn){
      previousBedOn = false;
    }
  }
  
  // Check if hotend has been on too long
  if (digitalRead(exPosPin)== HIGH && digitalRead(exNegPin)== LOW){  
    Serial.print("Hotend on for ");
    Serial.println(currentMillis - extStartMillis);
    if (previousExtOn) {
      if (currentMillis - extStartMillis > extInterval) {
          Serial.println("Hotend on too long");
          currentState = PROBLEM;
        }
      }
    else {
      previousExtOn = true;
      extStartMillis = currentMillis;
    }
  }
  else {
    Serial.println("Hotend off");
    if (previousExtOn){
      previousExtOn = false;
    }
  }

  
  // If the state has changed, update the pins
  if (currentState != oldState){
    updatePins();
    oldState = currentState;
  }

  // Perform these checks once a second
  delay( 1000 );
}

static unsigned long oversample(int pin){
// Oversampling concepts from http://www.electricrcaircraftguy.com/2014/05/using-arduino-unos-built-in-16-bit-adc.html and http://www.atmel.com/Images/doc8003.pdf
int i = 0;
unsigned long acc = 0;
// Read the thermistor muiltiple times and accumulate the result
for (i=0; i<OVERSAMPLES; i++)
{
   acc += analogRead(pin);
}
acc = acc >> OVERSAMPLEBITS; // Divide the accumulated reading by performing a right bit shift

return acc;
}


static float analog2temp(unsigned long raw) {
  // MODIFIED FROM MARLIN https://github.com/MarlinFirmware/Marlin
 // Derived from RepRap FiveD extruder::getTemperature()
 int i;
  

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

  return celsius;

}

void updatePins() {
  switch (currentState) {
    case OK:
      digitalWrite(goodLedPin, HIGH);
      digitalWrite(badLedPin, LOW);
      digitalWrite(relayPin, LOW);
      break;
    case PROBLEM:
      digitalWrite(goodLedPin, LOW);
      digitalWrite(badLedPin, HIGH);
      digitalWrite(relayPin, HIGH);
      break;
    default: 
      digitalWrite(goodLedPin, LOW);
      digitalWrite(badLedPin, HIGH);
      digitalWrite(relayPin, HIGH);

    break;
  }
}
