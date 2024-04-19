// LCD Pin Map:
// L4 (RS) -> D7
// L6 (EN) -> D6
// L11 (DB4)-> D5
// L12 (DB5)-> D4
// L13 (DB6)-> D3
// L14 (DB7)-> D2

#include <PWM.h>
#include <LiquidCrystal.h>
#include <string.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// LCD Constants
int rawValue; // raw reading
int oldValue; // for deadband
byte Byte; // final byte
byte oldByte; // for printing


const int pwm = 10; // pin 10 as pwm
int32_t frequency = 20000; // 20kHz
const int dir = 9;  // pin 9 as dir
int i=0; 

void setup(){
  InitTimersSafe();

  bool success_pwm = SetPinFrequencySafe(pwm, frequency);
  //bool success_dir = SetPinFrequencySafe(dir, frequency);

  if(success_pwm) {
    pinMode(pwm,OUTPUT); //declare pin pwm as OUTPUT
  }
  
  
  pinMode(dir,OUTPUT); //declare pin dir as OUTPUT

  lcd.begin(16, 2);

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
  lcd.setCursor(0, 0);
   // rawValue = analogRead(A0); // dummy read, if needed
  rawValue = analogRead(A5); // read pot
  if (rawValue < (oldValue - 5) || rawValue > (oldValue + 5)) { // add some deadband
    oldValue = rawValue; // update value
    Byte = oldValue >> 2; // convert 10-bit to 8-bit
    // Byte = map(oldValue, 0 , 1023, 0, 255);
    //Byte = oldValue;
    if (oldByte != Byte) { // only print if value changes 
      // lcd.print("Byte is: ");
      if (Byte < 1000)
        lcd.print("   ");
      if (Byte < 100)
        lcd.print("  ");
      if (Byte < 10)
        lcd.print(" ");
        
      lcd.setCursor(0, 0);
      lcd.print(Byte);

      oldByte = Byte; // update value
      pwmWrite(pwm, Byte);
    }
  }
}