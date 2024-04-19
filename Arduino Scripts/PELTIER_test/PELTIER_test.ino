#include <Arduino.h>
#include <PWM.h>
#include <string.h>
#include <pgmspace.h>

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


// const double tStart = 0;
const int num_iter = 1490;

//int pwmProfile[num_iter];

int N_TIMER = 0;
// timer2 maxes out at 10ms. If I want a second between each interrupt then I gotta wait for 100 interrupts
int N_INTERRUPTS = 100;
int voltDex = 0;

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

double current_profile(double A, double k, double t, double t0) {
    return A - A * exp(-k * (t-t0));
}

void generatePWMProfile(int pwmProfile[], int size, float amplitude, double frequency, double phase, int mode) {
    // amplitude of final voltage 
    // mode 1, 2, 3 -> step, ramp, sinusoidal
    const float maxVoltage = 9.56; // Maximum voltage of system in volts
    const int maxPWM = 255; // Maximum PWM duty cycle value (8-bit resolution)

    if(mode == 1) {
      for (int i = 0; i < size; i++) {
          // Calculate PWM duty cycle percentage
         pwmProfile[i] = (int)((amplitude/ maxVoltage) * maxPWM);
      }
    }
    if(mode == 2){
      // if size must match length of test i.e. 2000 means 2000 seconds. 
      double slope = amplitude / size;
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

  // INA240 init
  analogReference(EXTERNAL);
  
  pwmWrite(pwm, 0);
  // raw value
  VREF = voltageReader(NUMSAMPLES_VREF, INA240PIN, false);

  Serial.print(F("Average V_ref "));
  Serial.println(VREF);

  // Set up Peltier Model Arrays

  // Generate time array
  //for(int i = 0; i<num_iter; i++)
  //{
  //  t[i] = i*dt;
  //}

  // Generate current profile
  // Operating Parameters
  //float T_WATER = 15; // adjust before test, measure water bath temperature in C
  //float Th_op = 273.15 + T_WATER; // operating hotside temp in K
  //float m_op = 0.117; // weigh the aluminum block on the heat sink in KG
  //float c_op = 921.096;

  // Peltier Parameters
  //float Sm = 0.05133;
  //float Rm = 1.40110;
  //float Km = 0.74433;

  //double* voltProfile = new double[num_iter];
  //double* TcProfile = new double[num_iter];
  //double* iProfile = new double[num_iter];
  //double* delTProfile = new double[num_iter];

  //uint8_t dt = 1; // in seconds
  //for (int i = 0; i < num_iter; i++) {
  //  uint8_t ti = i*dt;
  //  iProfile[i] = current_profile(4.0, 0.05, ti, 0);
  //}

  // Initialize arrays
  //TcProfile[0] = Th_op;
  //delTProfile[0] = (Sm * TcProfile[0] * iProfile[0] - 0.5 * Rm * pow(iProfile[0], 2) - Km * (Th_op - TcProfile[0])) / (m_op * c_op);
  //voltProfile[0] = Sm * (Th_op - TcProfile[0]) + iProfile[0] * Rm;
  
  // Loop 
  //for (int x = 1; x < num_iter; x++) {
  //  TcProfile[x] = TcProfile[x - 1] - delTProfile[x - 1];
  //  delTProfile[x] = (Sm * TcProfile[x] * iProfile[x] - 0.5 * Rm * pow(iProfile[x], 2) - Km * (Th_op - TcProfile[x])) / (m_op * c_op);
  //  voltProfile[x] = Sm * (Th_op - TcProfile[x]) + iProfile[x] * Rm;
  //}

  //delete[] TcProfile;
  //delete[] delTProfile;
  //delete[] iProfile;

  //int* pwmProfile = new int[num_iter];
  //for (int i=0; i<num_iter; i++){
  //  pwmProfile[i] = 174;
  //}
  //convertToPWM(voltProfile, pwmProfile, num_iter);
  //delete[] voltProfile;

  
  Serial.println(F("Test Beginning"));
  Serial.println(F(">T_ambient, >T_cold, >T_hot, >iSensed, >ramp_dex, timestamp (millis)"));
  delay(0);
  
  // Timer2 Setup
  TCCR2A = 0;           // Init Timer2A
  TCCR2B = 0;           // Init Timer2B
  TCCR2B |= B00000111;  // Prescaler = 1024
  OCR2A = 156;        // Timer Compare2A Register
  TIMSK2 |= B00000010;  // Enable Timer COMPA Interrupt
}


int RAMP_TIMER = 0;
int RAMP_LENGTH = 5;
int ramp_dex = 191;
int INFLECT_DEX_DOWN = 255;
int INFLECT_DEX_UP = 0;
bool isDecrease = false;
bool isIncrease = false;


ISR(TIMER2_COMPA_vect){
  OCR2A += 156; // Advance The COMPA Register
  // Handle The Timer Interrupt
  if(N_TIMER == N_INTERRUPTS){
    // 1 second has passed

/*     if(ramp_dex == INFLECT_DEX_UP){
      isDecrease = false;
      isIncrease = true;
    }
    if(ramp_dex == INFLECT_DEX_DOWN){
      isDecrease = true;
      isIncrease = false;
    } */


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
  
/*   if(RAMP_TIMER==RAMP_LENGTH){
    if(isDecrease){
      ramp_dex -= 1;
    }  
    if(isIncrease) {
      ramp_dex += 1;
    }
    RAMP_TIMER = 0;
  } */
}

void loop() {
  digitalWrite(dir,LOW);

}
