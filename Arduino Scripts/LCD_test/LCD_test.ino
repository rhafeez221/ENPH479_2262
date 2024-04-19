// LCD Pin Map:
// L4 (RS) -> D7
// L6 (EN) -> D6
// L11 (DB4)-> D5
// L12 (DB5)-> D4
// L13 (DB6)-> D3
// L14 (DB7)-> D2


/*
  LiquidCrystal Library - Hello World

 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.

 This sketch prints "Hello World!" to the LCD
 and shows the time.

*/

// include the library code:
#include <LiquidCrystal.h>
#include <string.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int rawValue; // raw reading
int oldValue; // for deadband
byte Byte; // final byte
byte oldByte; // for printing

void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  //lcd.print("hello, world!");
  Serial.begin(115200);
}

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 0);
   // rawValue = analogRead(A0); // dummy read, if needed
  rawValue = analogRead(A0); // read pot
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

      Serial.print("Byte is: ");
      Serial.println(Byte);

      oldByte = Byte; // update value
    }
  }
  
  
  
  // print the number of seconds since reset:
  //lcd.print(millis() / 1000);
}