#include <Arduino.h>
#include <PWM.h>
#include <string.h>
#include <pgmspace.h>
#include <PID_v1.h>

int TA_PIN = A0;
int TC_PIN = A1;
int TH_PIN = A2;
  
#define NUMSAMPLES_VREF 50
#define INA240PIN A4
#define R_SENSE 0.005
#define GAIN 50

uint8_t pwm = 10; // pin 10 as pwm
int32_t frequency = 20000; // 20kHz
uint8_t dir = 9;  // pin 9 as dir

float VREF; //reference reading for INA240
float vRead; //reading from INA240 measurements
float vSense; //sensed voltage 
float iSense; // sensed current

float TA;
float TC;
float TH;


const int num_iter = 1490;

int N_TIMER = 0;
// timer2 maxes out at 10ms. If I want a second between each interrupt then I gotta wait for 100 interrupts
int N_INTERRUPTS = 100;
int voltDex = 0;

//PID 

double tempSetpoint, tempInput, tempOutput;
//double currSetpoint, currInput, currOutput;

//Specify the links and initial tuning parameters

double tempKp=1000, tempKi=0, tempKd=0;
//double currKp=2, currKi=5, currKd=0;

int tempSamplTime = 1000;
//int currSampleTime = 100;

PID tempPID(&tempInput, &tempOutput, &tempSetpoint, tempKp, tempKi, tempKd, REVERSE);
//PID currPID(&currInput, &currOutput, &currSetpoint, currKp, currKi, currKd, DIRECT);

unsigned long previousMillis = 0;  // will store last time measurement was recorded

// constants won't change:
const long interval = 1000;  // interval at which to record (milliseconds)

// Power Resistor Test
uint8_t PRES = 2; // pin 7 for power resistor switch
// already have a timer for one second intervals -> just count 5 of them 
int N_POWER_ON = 250;
int N_POWER_OFF = 265;
int power_counter = 0;
bool isPowered = false;

int N_REJECT_START = 210;

float voltageReader(int num_samples, uint8_t ref_pin, bool isRaw) {

  float average = 0;

  for (int i=0; i < num_samples; i++) {
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
  int THERMISTORNOMINAL = 15000;      
  // temp. for nominal resistance (almost always 25 C)
  int TEMPERATURENOMINAL = 25;
// the value of the 'reference' resistor
// Measured across ohmeter
  int SERIESRESISTOR = 14600;  
  float BCOEFFICIENT = 3916.06; 
  float average = 0;

  for (int i=0; i < num_samples; i++) {
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

  if(success_pwm) {
    pinMode(pwm,OUTPUT); //declare pin pwm as OUTPUT
  }
    
  pinMode(dir,OUTPUT); //declare pin dir as OUTPUT
  pinMode(PRES, OUTPUT); // declare pin PRES as OUTPUT
  digitalWrite(PRES, HIGH);

  // INA240 init
  analogReference(EXTERNAL);
  
  pwmWrite(pwm, 0);
  // raw value
  VREF = voltageReader(NUMSAMPLES_VREF, INA240PIN, false);

  Serial.print(F("Average V_ref "));
  Serial.println(VREF);
  
  tempInput = tempReader(NUMSAMPLES_VREF, TC_PIN, false);
  tempSetpoint = 5.0;
  
  tempPID.SetMode(AUTOMATIC);
  //tempPID.SetSampleTime(tempSamplTime);
  tempPID.SetOutputLimits(-255, 255);

  Serial.println(F("PID Test Beginning"));
  Serial.println(F(">T_ambient, >T_cold, >T_hot, >iSensed, tempKp, tempKi, tempKd, pidOUT, DIR, timestamp (millis)"));
  delay(0);
  
/*   // Timer2 Setup
  TCCR2A = 0;           // Init Timer2A
  TCCR2B = 0;           // Init Timer2B
  TCCR2B |= B00000111;  // Prescaler = 1024
  OCR2A = 156;        // Timer Compare2A Register
  TIMSK2 |= B00000010;  // Enable Timer COMPA Interrupt */
}

/* int RAMP_TIMER = 0;
int RAMP_LENGTH = 5;
int ramp_dex = 0;
int INFLECT_DEX_DOWN = 255;
int INFLECT_DEX_UP = 0;
bool isDecrease = false;
bool isIncrease = false; */
/* 
ISR(TIMER2_COMPA_vect){
  OCR2A += 156; // Advance The COMPA Register
  // Handle The Timer Interrupt
  if(N_TIMER == N_INTERRUPTS){
    // 1 second has passed

    if(ramp_dex == INFLECT_DEX_UP){
      isDecrease = false;
      isIncrease = true;
    }
    if(ramp_dex == INFLECT_DEX_DOWN){
      isDecrease = true;
      isIncrease = false;
    }


    pwmWrite(pwm, ramp_dex); // make sure volt profile is in units of PWM // pwmProfile[voltDex]
    TA = tempReader(NUMSAMPLES_VREF, TA_PIN, false);
    TC = tempReader(NUMSAMPLES_VREF, TC_PIN, false);
    TH = tempReader(NUMSAMPLES_VREF, TH_PIN, false);

    vRead = voltageReader(NUMSAMPLES_VREF, INA240PIN, false);
    iSense = ((vRead - VREF) / (GAIN)) / R_SENSE;
    Serial.print(TA);
    Serial.print(F(","));
    Serial.print(TC);
    Serial.print(F(","));
    Serial.print(TH);
    Serial.print(F(","));
    Serial.print(iSense);
    Serial.print(F(","));
    Serial.print(ramp_dex);
    Serial.print(F(","));
    Serial.println(millis());
      

    N_TIMER = 0;
    voltDex += 1;
    RAMP_TIMER +=1;
    //}
  }
  else{
    // full second has not passed. 
    N_TIMER += 1;
  }
  
  if(RAMP_TIMER==RAMP_LENGTH){
    if(isDecrease){
      ramp_dex -= 1;
    }  
    if(isIncrease) {
      ramp_dex += 1;
    }
    RAMP_TIMER = 0;
  }
} 
*/

void loop() {
  
  unsigned long currentMillis = millis();
  unsigned long powerMillis = millis();

  tempInput = tempReader(NUMSAMPLES_VREF, TC_PIN, false);
  tempPID.Compute();
  if (tempOutput >= 0){
    digitalWrite(dir,LOW);
  }
  else{
    digitalWrite(dir,HIGH);
  }
  pwmWrite(pwm, abs(tempOutput));
  //currSetpoint = tempOutput;
  //currInput = ((voltageReader(NUMSAMPLES_VREF, INA240PIN, false) - VREF) / (GAIN)) / R_SENSE; 
  //currPID.Compute();
  //Serial.print(tempOutput);

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
   /*  if (power_counter >= N_REJECT_START && power_counter < N_POWER_OFF){
      pwmWrite(pwm, 255);
    }
    else {
    
      pwmWrite(pwm, abs(tempOutput));
    } */

    
    previousMillis = currentMillis;
    TA = tempReader(NUMSAMPLES_VREF, TA_PIN, false);
    TC = tempReader(NUMSAMPLES_VREF, TC_PIN, false);
    TH = tempReader(NUMSAMPLES_VREF, TH_PIN, false);
    vRead = voltageReader(NUMSAMPLES_VREF, INA240PIN, false);
    iSense = ((vRead - VREF) / (GAIN)) / R_SENSE;
    
/*     Serial.print(">T_cold: ");
    Serial.println(TC);
    Serial.print(">PWM_out: ");
    Serial.println(tempOutput); */
    Serial.print(">Cold Temp: ");
    Serial.println(TC);
  /*   Serial.print(F(","));
    Serial.print(TC); */
    Serial.print(">Hot Temp: ");
    Serial.println(TH);
    Serial.print(F(","));
    Serial.print(iSense);
    Serial.print(F(","));
    Serial.print(tempOutput);
    Serial.print(F(","));
    Serial.print(isPowered);
    Serial.print(F(","));
    Serial.print(power_counter);
    Serial.print(F(","));
    Serial.println(millis());
    
    power_counter+=1;
  }

  if (power_counter >= N_POWER_ON && power_counter < N_POWER_OFF) {
    digitalWrite(PRES, LOW);
    isPowered = true;    
  }

  if (power_counter == N_POWER_OFF){
    digitalWrite(PRES, HIGH); 
    isPowered = false;
  }
}
