/**************************************************************************
 RH test of the RP2040 driver for the R61505 Truly LCD

 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source
 hardware by purchasing products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_R61505.h>

//#define SCREEN_WIDTH 320 // display width, in pixels
//#define SCREEN_HEIGHT 240 // display height, in pixels
#define SCREEN_WIDTH 320 // display width, in pixels
#define SCREEN_HEIGHT 240 // display height, in pixels
/*
// Declaration for display connected using software SPI (default case):
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
*/

// Comment out above, uncomment this block to use hardware SPI
#define OLED_DC     -1 // not used for this chip
#define OLED_CS     5
#define OLED_RESET  4
Adafruit_R61505 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  &SPI, OLED_DC, OLED_RESET, OLED_CS);



#define NUMFLAKES     10 // Number of snowflakes in the animation example


void setup() {
 Serial.begin(115200);
   delay(2000); // Pause for 2 seconds
Serial.println("starting up");
 // setRX(pin_size_t pin);  // assign RP2040 SPI0 pins
  SPI.setCS(5);
  SPI.setSCK(6);
  SPI.setTX(7);

Serial.println("SPI set up");
 
  if(!display.begin()) {
    Serial.println(F("display object allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  else Serial.println(F("display object created OK"));
  display.setRotation(0); // landscape for this display - portrait for Adafruit
 // SPI.setClockDivider(64); // doesn't work on Pico
display.clearDisplay();
display.display();
}

void loop() {

  // Clear the buffer
  Serial.println("display.cleardisplay");
  display.clearDisplay();
  display.display();
// delay(1000); // Pause for 2 seconds
//  display.fillRect(0,0,300,100, BLUE);

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(RED);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println("Hello, world!");

  display.setTextColor(GREEN); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(RED);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);
 display.setCursor(100,100);             // Start at top-left corner
   display.setTextColor(GREEN);
 display.println(F("Hello, world!"));

 // delay(4000);
 display.display();

  testdrawline();
 testdrawrect();
	
}

void testdrawline() {
  int16_t i;

  display.clearDisplay(); // Clear display buffer

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, RED);
    // Update screen with each newly-drawn line
    delay(1);
  }
  display.display(); 
  for(i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, GREEN);
 
    delay(1);
  }
     display.display();
  delay(2000);

  display.clearDisplay();

  for(i=0; i<display.width(); i+=10) {
    display.drawLine(0, display.height()-1, i, 0, YELLOW);
 
    delay(1);
  }
     display.display();
  for(i=display.height()-1; i>=0; i-=10) {
    display.drawLine(0, display.height()-1, display.width()-1, i, WHITE);

    delay(1);
  }
      display.display();
  delay(250);

  display.clearDisplay();

  for(i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, BLUE);
 
    delay(1);
  }
     display.display();
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, WHITE);

    delay(1);
  }
      display.display();
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, WHITE);

    delay(1);
  }
      display.display();
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, WHITE);

    delay(1);
  }
    display.display();
  delay(2000); // Pause for 2 seconds
}

void testdrawrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, BLUE);

    delay(1);
  }
    display.display(); // Update screen with each newly-drawn rectangle
  delay(2000);
}

void testfillrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=3) {
    // The INVERSE color is used so rectangles alternate white/black
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, GREEN);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testdrawcircle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=2) {
    display.drawCircle(display.width()/2, display.height()/2, i, WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillcircle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=3) {
    // The INVERSE color is used so circles alternate white/black
    display.fillCircle(display.width() / 2, display.height() / 2, i, YELLOW);
    display.display(); // Update screen with each newly-drawn circle
    delay(1);
  }

  delay(2000);
}

void testdrawroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    // The INVERSE color is used so round-rects alternate white/black
    display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, RED);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawtriangle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=5) {
    display.drawTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfilltriangle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=5) {
    // The INVERSE color is used so triangles alternate white/black
    display.fillTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, RED);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(GREEN); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}

void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(BLUE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello, world!"));

  display.setTextColor(GREEN, WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(RED);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);
 display.setCursor(100,100);             // Start at top-left corner
   display.setTextColor(GREEN);
 display.println(F("Hello, world!"));
  display.display();
  delay(2000);
}

void testscrolltext(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(GREEN);
  display.setCursor(10, 0);
  display.println(F("scroll"));
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
  delay(1000);
}

