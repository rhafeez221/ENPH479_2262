// SPDX-FileCopyrightText: 2011 Limor Fried/ladyada for Adafruit Industries
//
// SPDX-License-Identifier: MIT

// thermistor-2.ino Intermediate test program for a thermistor. Adafruit Learning System Tutorial
// https://learn.adafruit.com/thermistor/using-a-thermistor by Limor Fried, Adafruit Industries
// MIT License - please keep attribution and please consider buying parts from Adafruit

// which analog pin to connect
#define THERMISTORPIN A2         
// how many samples to take and average, more takes longer
// but is more 'smooth'
// resistance at 25 degrees C
#define THERMISTORNOMINAL 15000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
#define NUMSAMPLES 5
// the value of the 'reference' resistor
// Measured across ohmeter
#define SERIESRESISTOR 14600  
#define BCOEFFICIENT 3916.06   
 
int samples[NUMSAMPLES];
 
void setup(void) {
  Serial.begin(115200);
  // connect AREF to 3.3V and use that as VCC, less noisy!
  analogReference(EXTERNAL);
}
 
void loop(void) {
  uint8_t i;
  float average;
 
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
   delay(10);
  }
 
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
 
  Serial.print("Average analog reading "); 
  Serial.println(average);
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
 
  Serial.print("Thermistor resistance "); 
  Serial.println(average);
    
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C
  
  Serial.print("Temperature "); 
  Serial.print(steinhart);
  Serial.println(" *C");
  
  delay(1000);
}
