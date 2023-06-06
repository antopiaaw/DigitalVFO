#include <si5351.h>

#include <LiquidCrystal_I2C.h>

#include "Wire.h"

/*si5351 VFO
  By LA3PNA  27 March 2015
  Modified by NT7S  25 April 2015
  Modified to be Si5351 Arduino v2 compliant by NT7S  21 Nov 2016
  This version uses the new version of the Si5351 library from NT7S.
  see: http://arduino.cc/en/Reference/AttachInterrupt for what pins that have interrupts.
  UNO and 328 boards: Encoder on pin 2 and 3. Center pin to GND.
  Leonardo: Encoder on pin 0 and 1. Center pin to GND.
  100nF from each of the encoder pins to gnd is used to debounce
  The pushbutton goes to pin 11 to set the tuning rate.
  Pin 12 is the RX/TX pin. Put this pin LOW for RX, open or high for TX.
  Single transistor switch to +RX will work.
  VFO will NOT tune in TX.
  LCD connections for for the LinkSprite 16 X 2 LCD Keypad Shield for Arduino.
  Change as necessary for your LCD.

  IF frequency is positive for sum product (IF = RF + LO) and negative for diff (IF = RF - LO)
  VFO signal output on CLK0, BFO signal on CLK2
*/
const uint8_t encTableHalfStep[6][4] = //look up table for io 0 and 1;
{
  {0x3, 0x2, 0x1, 0x0},
  {0x23, 0x0, 0x1, 0x0},
  {0x13, 0x2, 0x0, 0x0},
  {0x3, 0x5, 0x4, 0x0},
  {0x3, 0x3, 0x4, 0x10},
  {0x3, 0x5, 0x3, 0x20}
};



// Class instantiation
Si5351 si5351;
LiquidCrystal_I2C lcd(0x27, 20, 4);
const int LONG_PRESS_TIME = 1000;
const int SHORT_PRESS_TIME = 500;

const unsigned long FreqStep[] = {50, 100, 500, 1000, 5000, 10000};
const unsigned long MhzBand[] {3500000, 7000000, 10100000, 14000000, 18068000, 21000000, 24890000, 28000000, 50000000}; // band switch
const unsigned long UpperFrequency[] {3800000, 7200000, 10150000, 14350000, 18168000, 24990000, 29700000, 52000000};
const unsigned long UsefulFrequencies[] {600000, 300000, 160000, 600000};
const int8_t BUTTON_PIN = 11;
const int8_t TX_PIN = 12;// not used
const int8_t RIT_PIN = 10;
const int8_t BAND_80_MTRS = 9;
const int8_t BAND_40_MTRS = 8;
const int8_t BAND_20_MTRS = 7;

#define FrequencyWidth       (sizeof(FreqStep) / sizeof(FreqStep[0]))
#define BandBottom       (sizeof(MhzBand) / sizeof(MhzBand[0]))
#define BandTop       (sizeof(UpperFrequency) / sizeof(UpperFrequency[0]))
// Variables will change:
unsigned int BandNumber=0;  //sets band
volatile int8_t tx; //FOR TX OUTPUT
unsigned long iffreq = 0; // set the IF frequency in Hz.
// set this to your wanted tuning rate in Hz.
unsigned long corr = 0; // this is the correction factor for the Si5351, use calibration sketch to find value.
unsigned int lastState = HIGH;  // the previous state from the input pin
unsigned int currentState;     // the current reading from the input pin
unsigned long pressedTime  = 0;
unsigned long releasedTime = 0;
int8_t freqsteps = 0; //placement for the vfo steps on the Freqsteps[]
unsigned long RitSavedRxFrequency = 0; //save frequwndy
bool IsInRit = false;//are we in rit function
unsigned long RitSavedFreqSteps = 0;
volatile unsigned long frequencyRead = 0; // VFO 200MHZ ONLY
unsigned long frequencyOldRead;


int8_t encoderPinA = 2;   // right
int8_t encoderPinB = 3;   // left
unsigned int LastSwitchPosition=0;//band switch rotary switch

void doVfoStep()
{
  static volatile int8_t encState = 0;   // volatile for interrupt

  encState = encTableHalfStep[encState & 0xF][(digitalRead(encoderPinA) << 1) | digitalRead(encoderPinB)];
  //delay(1);
  int8_t result = encState & 0x30;
  /// Serial.print(FreqStep[freqsteps]);
  //Serial.println("bandwidth" );
  //Serial.println(UpperFrequency[BandNumber] - MhzBand[BandNumber]);
  if (result == 0x10) {

    //Serial.println("Right/CW rotation");
    //Serial.println(frequencyRead);
    // give  a reading of 200khz for vfo
    if ((!tx) && (frequencyRead + FreqStep[freqsteps]) <= (UpperFrequency[BandNumber] - MhzBand[BandNumber])) frequencyRead += FreqStep[freqsteps];

  }
  else if (result == 0x20 && frequencyRead >= FreqStep[freqsteps] ) {
    //Serial.println("Left/CCW rotation");
    if (! tx ) {
      frequencyRead -= FreqStep[freqsteps];
      //Serial.println (frequencyRead);
    }

  }

}





void sprintf_seperated(char *str, unsigned long num)
{
  // We will print out the frequency as a fixed length string and pad if less than 100s of MHz
  char temp_str[6];
  int zero_pad = 0;

  // MHz
  if (num / 1000000UL > 0)
  {
    sprintf(str, "%3lu", num / 1000000UL);
    zero_pad = 1;
  }
  else
  {
    strcat(str, "   ");
  }
  num %= 1000000UL;

  // kHz
  if (zero_pad == 1)
  {
    sprintf(temp_str, ",%03lu", num / 1000UL);
    strcat(str, temp_str);
  }
  else if (num / 1000UL > 0)
  {
    sprintf(temp_str, ",%3lu", num / 1000UL);
    strcat(str, temp_str);
    zero_pad = 1;
  }
  else
  {
    strcat(str, "   ");
  }
  num %= 1000UL;

  // Hz
  if (zero_pad == 1)
  {
    sprintf(temp_str, ",%03lu", num);
    strcat(str, temp_str);
  }
  else
  {
    sprintf(temp_str, ",%3lu", num);
    strcat(str, temp_str);
  }

  strcat(str, " MHz");
}





void draw_lcd(void)
{
  char temp_str[21];

  if (IsInRit) {
    //swap output values
    lcd.setCursor(6, 3);
    int result = (MhzBand[BandNumber] + frequencyRead) - (MhzBand[BandNumber] + RitSavedRxFrequency);

    sprintf(temp_str, "%5d", result);
    lcd.print(temp_str);
  }
  else
  {
    sprintf_seperated(temp_str, MhzBand[BandNumber] + frequencyRead);
    lcd.setCursor(2, 1);
    lcd.print(temp_str);
    lcd.setCursor(6, 2);
    sprintf(temp_str, "%5u", FreqStep[freqsteps]);
    lcd.print(temp_str);
  }
}


void setup()
{
Serial.begin(9600);

  // Set GPIO
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(TX_PIN, INPUT_PULLUP);
  pinMode(RIT_PIN, INPUT_PULLUP);
  pinMode(BAND_80_MTRS, INPUT_PULLUP);
  pinMode(BAND_40_MTRS, INPUT_PULLUP);
  pinMode(BAND_20_MTRS, INPUT_PULLUP);
  // Turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  digitalWrite(TX_PIN, HIGH);
  digitalWrite(RIT_PIN, HIGH);
  digitalWrite(BAND_80_MTRS, HIGH);
  digitalWrite(BAND_40_MTRS, HIGH);
  digitalWrite(BAND_20_MTRS, HIGH);
  // encoder pin on interrupt 0 (pin 2) changed for half steps.
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doVfoStep, CHANGE);

  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doVfoStep, CHANGE);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(5, 0);
  delay(2000);

  lcd.print("Si5351 VFO");
  lcd.setCursor(0, 2);
  lcd.print("Step:           ");
  lcd.setCursor(0, 3);
  lcd.print("Rit off");

  unsigned long band = ((MhzBand[BandNumber])) + frequencyRead;

 // Serial.print(band);
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, corr);
  //si5351.set_freq(MhzBand[BandNumber] + frequencyRead * 100ULL, SI5351_CLK0);
  //si5351.set_freq(iffreq * 100ULL, SI5351_CLK2);
  draw_lcd();
}

void loop()
{
  if (digitalRead(TX_PIN)) //not used but implimented
  {
    tx = 0;
  }
  else
  {
    tx = 1;
  }

  // start rit pin reads only once IsInRit enabled
  int result = digitalRead(RIT_PIN);

  if (result == true) {

    if (IsInRit == false)
    {
      //Serial.print(result);
      RitSavedRxFrequency = frequencyRead;
      IsInRit = true; ////
      RitSavedFreqSteps = freqsteps;
      freqsteps = 0;
      lcd.setCursor(0, 3);
      lcd.print("Rit on        ");
    }
  }
  else
  {
    if (IsInRit == true)
    {
      frequencyRead = RitSavedRxFrequency;
      IsInRit = false;
      freqsteps = RitSavedFreqSteps;
      lcd.setCursor(0, 3);
      lcd.print("Rit off          ");
    }
  } //end ritpin

 //process the bandswitch
 
 unsigned int WhichSwitchPosition = ReadRotarySwitch();
  
  if (WhichSwitchPosition != LastSwitchPosition){
    // process the switch location set band to 0
    if (WhichSwitchPosition== 1){
       BandNumber = 3;
       frequencyRead=0;
       draw_lcd();
     }
    if (WhichSwitchPosition== 2){
      BandNumber = 1;
      frequencyRead=0;
      draw_lcd();
     }
    if (WhichSwitchPosition== 3){
      BandNumber = 0;
      frequencyRead=0;
      draw_lcd();
     }
    LastSwitchPosition =  WhichSwitchPosition;    
  }




  //set frequency and draw lcd.
  if (frequencyOldRead != frequencyRead) // freq changed even when in rit
  {
    frequencyOldRead = frequencyRead; // update frequency
    si5351.set_freq((MhzBand[BandNumber] + frequencyRead)*100ULL, SI5351_CLK0);
    draw_lcd(); // update screen to show new freq
  }
  // si5351.set_freq(( MhzBand[BandNumber] + frequencyRead) * 100ULL, SI5351_CLK0);
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_PIN);

  if (lastState == HIGH && currentState == LOW)       // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH) { // button is released
    releasedTime = millis();

    int pressDuration = releasedTime - pressedTime;

    if ( pressDuration > LONG_PRESS_TIME ) {
      Serial.println("A long press is detected");
    //  BandNumber += 1;
      if (BandNumber > BandBottom - 1 ) //oh dear
      {
        BandNumber = 0;
      }
      draw_lcd();
      frequencyRead = 0; //set frquency to 0
    }
    if ( pressDuration < SHORT_PRESS_TIME ) {
      Serial.println("A short press is detected");
      freqsteps += 1;
      if (freqsteps > FrequencyWidth - 1 )//same here
      {
        freqsteps = 0;
      }
      draw_lcd();

    }
  }



  // save the the last state
  lastState = currentState;


}
unsigned int ReadRotarySwitch(){
  if(digitalRead(BAND_20_MTRS)==LOW){return 1;}
  if(digitalRead(BAND_40_MTRS)==LOW){return 2;}
  if(digitalRead(BAND_80_MTRS)==LOW){return 3;}

  return 0;
}
