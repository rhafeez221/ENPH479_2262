// LCD Pin Map:
// L4 (RS) -> D7
// L6 (EN) -> D6
// L11 (DB4)-> D5
// L12 (DB5)-> D4
// L13 (DB6)-> D3
// L14 (DB7)-> D2

#include <PWM.h>
//#include <LiquidCrystal.h>
#include <string.h>

#define TA_PIN A0
#define TC_PIN A1
#define TH_PIN A2
#define THERMISTORNOMINAL 15000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
#define NUMSAMPLES_T 5
// the value of the 'reference' resistor
// Measured across ohmeter
#define SERIESRESISTOR 14600  
#define BCOEFFICIENT 3916.06   

#define NUMSAMPLES_VREF 50
#define INA240PIN A4
#define R_SENSE 0.005
#define GAIN 50.0

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
// LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// LCD Constants
int rawValue; // raw reading
const byte pwmPot = A3; 
int oldValue; // for deadband
byte Byte; // final byte
byte oldByte; // for printing


const int pwm = 10; // pin 10 as pwm
int32_t frequency = 20000; // 20kHz
const int dir = 9;  // pin 9 as dir
int i=0; 

float VREF; //reference reading for INA240
float vRead; //reading from INA240 measurements
float vSense; //sensed voltage 
float iSense; // sensed current

float TA;
float TC;
float TH;

float voltageReader(int num_samples, uint8_t ref_pin, bool isRaw) {

  uint8_t i = 0;
  float average = 0;

  for (i=0; i < num_samples; i++) {
    average += analogRead(ref_pin);
  }

  average /= num_samples;
  if (isRaw)
  {
    return average;
  }
  else
  {
    return average * (5.0 / 1023.0);
  }  
}

float tempReader(int num_samples, uint8_t ref_pin, bool isKelvin) {
  uint8_t i = 0;
  float average = 0;

  for (i=0; i < num_samples; i++) {
    average += analogRead(ref_pin);
  }

  average /= num_samples;
  // Convert to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert

  if (isKelvin)
  {
    return steinhart; //in K
  }
  else
  {
    return steinhart -= 273.15; // in C
  }

}


void setup(){
  Serial.begin(115200);
  // PWM Pin Init
  InitTimersSafe();

  bool success_pwm = SetPinFrequencySafe(pwm, frequency);
  //bool success_dir = SetPinFrequencySafe(dir, frequency);

  if(success_pwm) {
    pinMode(pwm,OUTPUT); //declare pin pwm as OUTPUT
  }
    
  pinMode(dir,OUTPUT); //declare pin dir as OUTPUT

  // LCD Init
  //lcd.begin(16, 2);

  // INA240 init
  //analogReference(DEFAULT);
  
  pwmWrite(pwm, 0);
  // raw value
  VREF = voltageReader(NUMSAMPLES_VREF, INA240PIN, false);

  Serial.print("Average V_ref ");
  Serial.println(VREF);

}


void loop(){
    // Set direction first; 
  digitalWrite(dir,LOW);  // if DIR pin is HIGH, B will HIGH ; if DIR pin is LOW, A will HIGH
    // 100% = 255
    // 75%  = 191
    // 50%  = 127
    // 25%  = 64
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  // lcd.setCursor(0, 0);
   // rawValue = analogRead(A0); // dummy read, if needed
  //rawValue = analogRead(pwmPot); // read pot
  //if (rawValue < (oldValue - 5) || rawValue > (oldValue + 5)) { // add some deadband
  //  oldValue = rawValue; // update value
  //  Byte = oldValue >> 2; // convert 10-bit to 8-bit
  //  // Byte = map(oldValue, 0 , 1023, 0, 255);
  //  //Byte = oldValue;
  //  if (oldByte != Byte) { // only print if value changes 
  //    // lcd.print("Byte is: ");
  //    if (Byte < 1000)
  //      lcd.print("   ");
  //    if (Byte < 100)
  //      lcd.print("  ");
  //    if (Byte < 10)
  //      lcd.print(" ");
  //      
  //    lcd.setCursor(0, 0);
  //    lcd.print(Byte);

  //    oldByte = Byte; // update value
  //    pwmWrite(pwm, Byte);
  //  }
  //}

  
  Serial.print(">V_ref: ");
  Serial.println(VREF);

  vRead = voltageReader(NUMSAMPLES_VREF, INA240PIN, false);
  Serial.print(">V_read: ");
  Serial.println(vRead);

  iSense = ((vRead - VREF) / (GAIN)) / R_SENSE;
  Serial.print(">Current Sensed: ");
  Serial.println(iSense); 

  TA = tempReader(NUMSAMPLES_VREF, TA_PIN, false);
  TC = tempReader(NUMSAMPLES_VREF, TC_PIN, false);
  TH = tempReader(NUMSAMPLES_VREF, TH_PIN, false);
  Serial.print(">T_ambient: ");
  Serial.println(TA);

  Serial.print(">T_cold: ");
  Serial.println(TC);

  Serial.print(">T_hot: ");
  Serial.println(TH);

  pwmWrite(pwm, 255);
  
}