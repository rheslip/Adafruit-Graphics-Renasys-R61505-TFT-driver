/*!
 * @file Adafruit_R61505_PICO.cpp
 *
 * @mainpage Arduino library for monochrome OLEDs based on R61505 drivers.
 *
 * @section intro_sec Introduction
 *
 * This is documentation for Adafruit's R61505 library for monochrome
 * OLED displays: http://www.adafruit.com/category/63_98
 *
 * These displays use I2C or SPI to communicate. I2C requires 2 pins
 * (SCL+SDA) and optionally a RESET pin. SPI requires 4 pins (MOSI, SCK,
 * select, data/command) and optionally a reset pin. Hardware SPI or
 * 'bitbang' software SPI are both supported.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a
 * href="https://github.com/adafruit/Adafruit-GFX-Library"> Adafruit_GFX</a>
 * being present on your system. Please make sure you have installed the latest
 * version before using this library.
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries, with
 * contributions from the open source community.
 *
 * @section license License
 *
 * BSD license, all text above, and the splash screen included below,
 * must be included in any redistribution.
 *
 */

#ifdef __AVR__
#include <avr/pgmspace.h>
#elif defined(ESP8266) || defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
#include <pgmspace.h>
#else
#define pgm_read_byte(addr)                                                    \
  (*(const unsigned char *)(addr)) ///< PROGMEM workaround for non-AVR
#endif

#if !defined(__ARM_ARCH) && !defined(ENERGIA) && !defined(ESP8266) &&          \
    !defined(ESP32) && !defined(__arc__)
#include <util/delay.h>
#endif

#include "Adafruit_R61505.h"
#include <Adafruit_GFX.h>

#define R61505_swap(a, b)                                                     \
  (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b))) ///< No-temp-var swap operation


#ifdef HAVE_PORTREG
#define R61505_SELECT *csPort &= ~csPinMask;       ///< Device select
#define R61505_DESELECT *csPort |= csPinMask;      ///< Device deselect
#define R61505_MODE_COMMAND *dcPort &= ~dcPinMask; ///< Command mode
#define R61505_MODE_DATA *dcPort |= dcPinMask;     ///< Data mode
#else
#define R61505_SELECT digitalWrite(csPin, LOW);       ///< Device select
#define R61505_DESELECT digitalWrite(csPin, HIGH);    ///< Device deselect
#define R61505_MODE_COMMAND digitalWrite(dcPin, LOW); ///< Command mode
#define R61505_MODE_DATA digitalWrite(dcPin, HIGH);   ///< Data mode
#endif


#if defined(SPI_HAS_TRANSACTION)
#define SPI_TRANSACTION_START spi->beginTransaction(spiSettings) ///< Pre-SPI
#define SPI_TRANSACTION_END spi->endTransaction()                ///< Post-SPI
#else // SPI transactions likewise not present in older Arduino SPI lib
#define SPI_TRANSACTION_START ///< Dummy stand-in define
#define SPI_TRANSACTION_END   ///< keeps compiler happy
#endif

// The definition of 'transaction' is broadened a bit in the context of
// this library -- referring not just to SPI transactions (if supported
// in the version of the SPI library being used), but also chip select
// (if SPI is being used, whether hardware or soft), and also to the
// beginning and end of I2C transfers (the Wire clock may be sped up before
// issuing data to the display, then restored to the default rate afterward
// so other I2C device types still work).  All of these are encapsulated
// in the TRANSACTION_* macros.

// Check first if Wire, then hardware SPI, then soft SPI:
#define TRANSACTION_START 							\
    SPI_TRANSACTION_START;                          \
    R61505_SELECT;                                  \
   ///< Wire, SPI or bitbang transfer setup
#define TRANSACTION_END                              \
    R61505_DESELECT;                                 \
    SPI_TRANSACTION_END;                             \
   ///< Wire, SPI or bitbang transfer end

// CONSTRUCTORS, DESTRUCTOR ------------------------------------------------



/*!
    @brief  Constructor for SPI R61505 displays, using software (bitbang)
            SPI.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  mosi_pin
            MOSI (master out, slave in) pin (using Arduino pin numbering).
            This transfers serial data from microcontroller to display.
    @param  sclk_pin
            SCLK (serial clock) pin (using Arduino pin numbering).
            This clocks each bit from MOSI.
    @param  dc_pin
            Data/command pin (using Arduino pin numbering), selects whether
            display is receiving commands (low) or data (high).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the
            bus with other devices. Active low.
    @return Adafruit_R61505 object.
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
Adafruit_R61505::Adafruit_R61505(uint16_t w, uint16_t h, int8_t mosi_pin,
                                   int8_t sclk_pin, int8_t dc_pin,
                                   int8_t rst_pin, int8_t cs_pin)
    : Adafruit_GFX(w, h), spi(NULL), wire(NULL), buffer(NULL),
      mosiPin(mosi_pin), clkPin(sclk_pin), dcPin(dc_pin), csPin(cs_pin),
      rstPin(rst_pin) {}

/*!
    @brief  Constructor for SPI R61505 displays, using native hardware SPI.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  spi_ptr
            Pointer to an existing SPIClass instance (e.g. &SPI, the
            microcontroller's primary SPI bus).
    @param  dc_pin
            Data/command pin (using Arduino pin numbering), selects whether
            display is receiving commands (low) or data (high).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the
            bus with other devices. Active low.
    @param  bitrate
            SPI clock rate for transfers to this display. Default if
            unspecified is 8000000UL (8 MHz).
    @return Adafruit_R61505 object.
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
Adafruit_R61505::Adafruit_R61505(uint16_t w, uint16_t h, SPIClass *spi_ptr,
                                   int8_t dc_pin, int8_t rst_pin, int8_t cs_pin,
                                   uint32_t bitrate)
    : Adafruit_GFX(w, h), spi(spi_ptr ? spi_ptr : &SPI), wire(NULL),
      buffer(NULL), mosiPin(-1), clkPin(-1), dcPin(dc_pin), csPin(cs_pin),
      rstPin(rst_pin) {
#ifdef SPI_HAS_TRANSACTION
  spiSettings = SPISettings(bitrate, MSBFIRST, SPI_MODE0);
#endif
}


/*!
    @brief  Destructor for Adafruit_R61505 object.
*/
Adafruit_R61505::~Adafruit_R61505(void) {

}

// LOW-LEVEL UTILS ---------------------------------------------------------

// Issue single byte out SPI, either soft or hardware as appropriate.
// SPI transaction/selection must be performed in calling function.
/*!
    @brief  Write a single byte to the SPI port.

    @param  d
                        Data byte to be written.

    @return void
    @note   See HAVE_PORTREG which defines if the method uses a port or bit-bang
   method
*/
inline void Adafruit_R61505::SPIwrite(uint8_t d) {
  if (spi) {
    (void)spi->transfer(d);
  } else {
    for (uint8_t bit = 0x80; bit; bit >>= 1) {
#ifdef HAVE_PORTREG
      if (d & bit)
        *mosiPort |= mosiPinMask;
      else
        *mosiPort &= ~mosiPinMask;
      *clkPort |= clkPinMask;  // Clock high
      *clkPort &= ~clkPinMask; // Clock low
#else
      digitalWrite(mosiPin, d & bit);
      digitalWrite(clkPin, HIGH);
      digitalWrite(clkPin, LOW);
#endif
    }
  }
}

// RH copied from my TFT driver so I can use the old code
void Adafruit_R61505::writecommand( uint8_t index, uint16_t data )
{
//  *csport &= ~cspinmask;  // assert CS
  R61505_SELECT
  SPIwrite(0x70); // device code, write index reg
  SPIwrite(0); // 
  SPIwrite(index);  // send index 
  R61505_DESELECT
//  *csport |= cspinmask;  // deassert CS  
//  *csport &= ~cspinmask;  // assert CS
  R61505_SELECT
  SPIwrite(0x72); // device code, write reg
  SPIwrite((byte)((data>>8) & 0x00ff));  // send high byte
  SPIwrite((byte)(data & 0x00ff));  // send low byte 
  R61505_DESELECT
//  *csport |= cspinmask;  // deassert CS  
//  delay(1);
}

/*!
    @brief Issue single command to R61505, using I2C or hard/soft SPI as
   needed. Because command calls are often grouped, SPI transaction and
   selection must be started/ended in calling function for efficiency. This is a
   protected function, not exposed (see R61505_command() instead).

        @param c
                   the command character to send to the display.
                   Refer to R61505 data sheet for commands
    @return None (void).
    @note
*/
void Adafruit_R61505::R61505_command1(uint8_t c) {
 // SPI (hw or soft) -- transaction started in calling function
    R61505_MODE_COMMAND
    SPIwrite(c);
}

/*!
    @brief Issue list of commands to R61505, same rules as above re:
   transactions. This is a protected function, not exposed.
        @param c
                   pointer to list of commands

        @param n
                   number of commands in the list

    @return None (void).
    @note
*/
void Adafruit_R61505::R61505_commandList(const uint8_t *c, uint8_t n) {
 // SPI -- transaction started in calling function
    R61505_MODE_COMMAND
    while (n--)
      SPIwrite(pgm_read_byte(c++));
}

// A public version of R61505_command1(), for existing user code that
// might rely on that function. This encapsulates the command transfer
// in a transaction start/end, similar to old library's handling of it.
/*!
    @brief  Issue a single low-level command directly to the R61505
            display, bypassing the library.
    @param  c
            Command to issue (0x00 to 0xFF, see datasheet).
    @return None (void).
*/
void Adafruit_R61505::R61505_command(uint8_t c) {
  TRANSACTION_START
  R61505_command1(c);
  TRANSACTION_END
}

//  INIT DISPLAY -------------------------------------------------

/*!
    @brief  initialize peripherals and pins.

    @param  reset
            If true, and if the reset pin passed to the constructor is
            valid, a hard reset will be performed before initializing the
            display. If using multiple R61505 displays on the same bus, and
            if they all share the same reset pin, you should only pass true
            on the first display being initialized, false on all others,
            else the already-initialized displays would be reset. Default if
            unspecified is true.
    @param  periphBegin
            If true, and if a hardware peripheral is being used (I2C or SPI,
            but not software SPI), call that peripheral's begin() function,
            else (false) it has already been done in one's sketch code.
            Cases where false might be used include multiple displays or
            other devices sharing a common bus, or situations on some
            platforms where a nonstandard begin() function is available
            (e.g. a TwoWire interface on non-default pins, as can be done
            on the ESP8266 and perhaps others).
    @return true on successful allocation/init, false otherwise.
            Well-behaved code should check the return value before
            proceeding.
    @note   MUST call this function before any drawing or updates!
*/
bool Adafruit_R61505::begin(bool reset,bool periphBegin) {

  clearDisplay();


  // Setup pin directions
// Using one of the SPI modes, either soft or hardware
    pinMode(dcPin, OUTPUT); // Set data/command pin as output
    pinMode(csPin, OUTPUT); // Same for chip select
#ifdef HAVE_PORTREG
    dcPort = (PortReg *)portOutputRegister(digitalPinToPort(dcPin));
    dcPinMask = digitalPinToBitMask(dcPin);
    csPort = (PortReg *)portOutputRegister(digitalPinToPort(csPin));
    csPinMask = digitalPinToBitMask(csPin);
#endif
    R61505_DESELECT
    if (spi) { // Hardware SPI
      // SPI peripheral begin same as wire check above.
      if (periphBegin)
        spi->begin();
    } else {                    // Soft SPI
      pinMode(mosiPin, OUTPUT); // MOSI and SCLK outputs
      pinMode(clkPin, OUTPUT);
#ifdef HAVE_PORTREG
      mosiPort = (PortReg *)portOutputRegister(digitalPinToPort(mosiPin));
      mosiPinMask = digitalPinToBitMask(mosiPin);
      clkPort = (PortReg *)portOutputRegister(digitalPinToPort(clkPin));
      clkPinMask = digitalPinToBitMask(clkPin);
      *clkPort &= ~clkPinMask; // Clock low
#else
      digitalWrite(clkPin, LOW); // Clock low
#endif
    }
  

  // Reset R61505 if requested and reset pin specified in constructor
  if (reset && (rstPin >= 0)) {
    pinMode(rstPin, OUTPUT);
    digitalWrite(rstPin, HIGH);
    delay(1);                   // VDD goes high at start, pause for 1 ms
    digitalWrite(rstPin, LOW);  // Bring reset low
    delay(10);                  // Wait 10 ms
    digitalWrite(rstPin, HIGH); // Bring out of reset
  }

  TRANSACTION_START

 // RH this is init from the R61505 driver code I found on the web
	writecommand( 0x00, 0x0000 );
	writecommand( 0x00, 0x0000 );
	writecommand( 0x00, 0x0000 );
	writecommand( 0x00, 0x0000 );
	
//	writecommand( 0xa4, 0x0001 ); //LCD_NvmCtrl4
	writecommand( 0x00, 0x0001 );
//	writecommand( 0x12, 0x0180 );
	writecommand( 0x07, 0x0001 );
	writecommand( 0x17, 0x0001 );
	delay(2);

	writecommand( 0x60, 0x2700 );
	writecommand( 0x08, 0x0808 );

	writecommand( 0x30, 0x0000 );
	writecommand( 0x31, 0x1103 );
	writecommand( 0x32, 0x0903 );
	writecommand( 0x33, 0x0205 );
	writecommand( 0x34, 0x0100 );
	writecommand( 0x35, 0x0401 );
	writecommand( 0x36, 0x1309 );
	writecommand( 0x37, 0x0301 );
	writecommand( 0x38, 0x0200 );
	writecommand( 0x39, 0x0310 );
	
	writecommand( 0x90, 0x001E );
	
	writecommand( 0x17, 0x0001 );
	writecommand( 0x10, 0x0530 );
	writecommand( 0x11, 0x0237 );
	writecommand( 0x12, 0x11B8 );
	delay(150);
	writecommand( 0x13, 0x0f00 );
	
	writecommand( 0x01, 0x0000 );
	writecommand( 0x02, 0x0000 );
	writecommand( 0x03, 0x1030 ); // entry mode register - bit 12 BGR appears to work opposite to what the docs say - set to 1 for RGB
//	writecommand( 0x03, 0x1088 );
	writecommand( 0x09, 0x0001 );
	writecommand( 0x0a, 0x0000 );
	writecommand( 0x0c, 0x0000 );
	writecommand( 0x0d, 0x0000 );
	writecommand( 0x0e, 0x0000 );
	writecommand( 0x0f, 0x0000 );

	writecommand( 0x50, 0x0038 ); // window set registers
	writecommand( 0x51, 0x00ef );
	writecommand( 0x52, 0x0000 );
	writecommand( 0x53, 0x013f );
	
	writecommand( 0x61, 0x0000 );
	writecommand( 0x6a, 0x0000 );
	
	writecommand( 0x80, 0x0000 );
	writecommand( 0x81, 0x0000 );
	writecommand( 0x82, 0x0000 );
	
	writecommand( 0x91, 0x0000 );
	writecommand( 0x92, 0x0100 );
	writecommand( 0x93, 0x0001 );
	writecommand( 0x94, 0x0001 );
	writecommand( 0x95, 0x001f );
	writecommand( 0x96, 0x0001 );
	writecommand( 0x97, 0x0100 );
	writecommand( 0x98, 0x0001 );
	writecommand( 0x99, 0x0001 );
	writecommand( 0x9c, 0x0043 );
	
	writecommand( 0x20, 0x0000 );
	writecommand( 0x21, 0x0000 );
	
//	writecommand( 0x22, 0x0000 );
//	writecommand( 0x07, 0x0173 );
	writecommand( 0x07, 0x0100 );
	
  TRANSACTION_END

  return true; // Success
}

void Adafruit_R61505::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1) {
/*
	writecommand( 0x20, x0 );
	writecommand( 0x21, y0 );
	writecommand( 0x50, x0 );
	writecommand( 0x52, y0 );
	writecommand( 0x51, x1 );
	writecommand( 0x53, y1 );
	*/
 // RH swapped X and Y for Truly display which is set up for landscape
	writecommand( 0x20, y0 );
	writecommand( 0x21, x0 );
	writecommand( 0x50, y0 );
	writecommand( 0x52, x0 );
	writecommand( 0x51, y1 );
	writecommand( 0x53, x1 );

  R61505_SELECT
  SPIwrite(0x70); // device code, write index reg
  SPIwrite(0); // 
  SPIwrite(0x22);  // send index 
  R61505_DESELECT

}

// DRAWING FUNCTIONS -------------------------------------------------------

/*!
    @brief  Set/clear/invert a single pixel. This is also invoked by the
            Adafruit_GFX library in generating many higher-level graphics
            primitives.
    @param  x
            Column of display -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @param  color
            Pixel color, one of: R61505_BLACK, R61505_WHITE or
            R61505_INVERSE.
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void Adafruit_R61505::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x >= 0) && (x < width()) && (y >= 0) && (y < height())) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
    case 1:
      R61505_swap(x, y);
      x = WIDTH - x - 1;
      break;
    case 2:
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
      break;
    case 3:
      R61505_swap(x, y);
      y = HEIGHT - y - 1;
      break;
    }
  setAddrWindow(x,y,x+1,y+1);

  R61505_SELECT
  SPIwrite(0x72); // device code, write reg or mem 
  SPIwrite(color >> 8);
  SPIwrite(color);
  R61505_DESELECT
  }
}


/*!
    @brief  Draw a vertical line. This is also invoked by the Adafruit_GFX
            library in generating many higher-level graphics primitives.
    @param  x
            Column of display -- 0 at left to (screen width -1) at right.
    @param  y
            Topmost row -- 0 at top to (screen height - 1) at bottom.
    @param  h
            Height of line, in pixels.
    @param  color
            Line color, one of: R61505_BLACK, R61505_WHITE or R61505_INVERSE.
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void Adafruit_R61505::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((y+h-1) >= _height) h = _height-y;
  setAddrWindow(x, y, x, y+h-1);
  uint8_t hi = color >> 8, lo = color;
  R61505_SELECT
  SPIwrite(0x72); // device code, write reg or mem 
  while (h--) {
    SPIwrite(hi);
    SPIwrite(lo);
  }
  R61505_DESELECT
}



/*!
    @brief  Draw a horizontal line. This is also invoked by the Adafruit_GFX
            library in generating many higher-level graphics primitives.
    @param  x
            Leftmost column -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @param  w
            Width of line, in pixels.
    @param  color
            Line color, one of: R61505_BLACK, R61505_WHITE or R61505_INVERSE.
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void Adafruit_R61505::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
  R61505_SELECT
  SPIwrite(0x72); // device code, write reg or mem 
  
  while (w--) {
    SPIwrite(hi);
    SPIwrite(lo);
  }
  R61505_DESELECT
}

// fill a rectangle
void Adafruit_R61505::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;
  R61505_SELECT
  SPIwrite(0x72); // device code, write reg or mem 
  
  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      SPIwrite(hi);
      SPIwrite(lo);
    }
  }
  R61505_DESELECT

}


void Adafruit_R61505::display(void) {
// does nothing, here for code that might call it
}


/*!
    @brief  Clear contents of display buffer (set all pixels to off).
    @return None (void).

*/
void Adafruit_R61505::clearDisplay(void) {
  fillRect(0, 0, WIDTH, HEIGHT, BLACK);
}

/*!
    @brief   Given 8-bit red, green and blue values, return a 'packed'
             16-bit color value in '565' RGB format (5 bits red, 6 bits
             green, 5 bits blue). This is just a mathematical operation,
             no hardware is touched.
    @param   red    8-bit red brightnesss (0 = off, 255 = max).
    @param   green  8-bit green brightnesss (0 = off, 255 = max).
    @param   blue   8-bit blue brightnesss (0 = off, 255 = max).
    @return  'Packed' 16-bit color value (565 format).
*/
uint16_t Adafruit_R61505::color565(uint8_t red, uint8_t green, uint8_t blue) {
  return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}


// SCROLLING FUNCTIONS -----------------------------------------------------
// RH stuff below won't work - have not fixed yet
/*!
    @brief  Activate a right-handed scroll for all or part of the display.
    @param  start
            First row.
    @param  stop
            Last row.
    @return None (void).
*/
// To scroll the whole display, run: display.startscrollright(0x00, 0x0F)
void Adafruit_R61505::startscrollright(uint8_t start, uint8_t stop) {
  TRANSACTION_START
  static const uint8_t PROGMEM scrollList1a[] = {
      R61505_RIGHT_HORIZONTAL_SCROLL, 0X00};
  R61505_commandList(scrollList1a, sizeof(scrollList1a));
  R61505_command1(start);
  R61505_command1(0X00);
  R61505_command1(stop);
  static const uint8_t PROGMEM scrollList1b[] = {0X00, 0XFF,
                                                 R61505_ACTIVATE_SCROLL};
  R61505_commandList(scrollList1b, sizeof(scrollList1b));
  TRANSACTION_END
}

/*!
    @brief  Activate a left-handed scroll for all or part of the display.
    @param  start
            First row.
    @param  stop
            Last row.
    @return None (void).
*/
// To scroll the whole display, run: display.startscrollleft(0x00, 0x0F)
void Adafruit_R61505::startscrollleft(uint8_t start, uint8_t stop) {
  TRANSACTION_START
  static const uint8_t PROGMEM scrollList2a[] = {R61505_LEFT_HORIZONTAL_SCROLL,
                                                 0X00};
  R61505_commandList(scrollList2a, sizeof(scrollList2a));
  R61505_command1(start);
  R61505_command1(0X00);
  R61505_command1(stop);
  static const uint8_t PROGMEM scrollList2b[] = {0X00, 0XFF,
                                                 R61505_ACTIVATE_SCROLL};
  R61505_commandList(scrollList2b, sizeof(scrollList2b));
  TRANSACTION_END
}

/*!
    @brief  Activate a diagonal scroll for all or part of the display.
    @param  start
            First row.
    @param  stop
            Last row.
    @return None (void).
*/
// display.startscrolldiagright(0x00, 0x0F)
void Adafruit_R61505::startscrolldiagright(uint8_t start, uint8_t stop) {
  TRANSACTION_START
  static const uint8_t PROGMEM scrollList3a[] = {
      R61505_SET_VERTICAL_SCROLL_AREA, 0X00};
  R61505_commandList(scrollList3a, sizeof(scrollList3a));
  R61505_command1(HEIGHT);
  static const uint8_t PROGMEM scrollList3b[] = {
      R61505_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL, 0X00};
  R61505_commandList(scrollList3b, sizeof(scrollList3b));
  R61505_command1(start);
  R61505_command1(0X00);
  R61505_command1(stop);
  static const uint8_t PROGMEM scrollList3c[] = {0X01, R61505_ACTIVATE_SCROLL};
  R61505_commandList(scrollList3c, sizeof(scrollList3c));
  TRANSACTION_END
}

/*!
    @brief  Activate alternate diagonal scroll for all or part of the display.
    @param  start
            First row.
    @param  stop
            Last row.
    @return None (void).
*/
// To scroll the whole display, run: display.startscrolldiagleft(0x00, 0x0F)
void Adafruit_R61505::startscrolldiagleft(uint8_t start, uint8_t stop) {
  TRANSACTION_START
  static const uint8_t PROGMEM scrollList4a[] = {
      R61505_SET_VERTICAL_SCROLL_AREA, 0X00};
  R61505_commandList(scrollList4a, sizeof(scrollList4a));
  R61505_command1(HEIGHT);
  static const uint8_t PROGMEM scrollList4b[] = {
      R61505_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL, 0X00};
  R61505_commandList(scrollList4b, sizeof(scrollList4b));
  R61505_command1(start);
  R61505_command1(0X00);
  R61505_command1(stop);
  static const uint8_t PROGMEM scrollList4c[] = {0X01, R61505_ACTIVATE_SCROLL};
  R61505_commandList(scrollList4c, sizeof(scrollList4c));
  TRANSACTION_END
}

/*!
    @brief  Cease a previously-begun scrolling action.
    @return None (void).
*/
void Adafruit_R61505::stopscroll(void) {
  TRANSACTION_START
  R61505_command1(R61505_DEACTIVATE_SCROLL);
  TRANSACTION_END
}

// OTHER HARDWARE SETTINGS -------------------------------------------------

/*!
    @brief  Enable or disable display invert mode (white-on-black vs
            black-on-white).
    @param  i
            If true, switch to invert mode (black-on-white), else normal
            mode (white-on-black).
    @return None (void).
    @note   This has an immediate effect on the display, no need to call the
            display() function -- buffer contents are not changed, rather a
            different pixel mode of the display hardware is used. When
            enabled, drawing R61505_BLACK (value 0) pixels will actually draw
   white, R61505_WHITE (value 1) will draw black.
*/
void Adafruit_R61505::invertDisplay(bool i) {
  TRANSACTION_START
  R61505_command1(i ? R61505_INVERTDISPLAY : R61505_NORMALDISPLAY);
  TRANSACTION_END
}

/*!
    @brief  Dim the display.
    @param  dim
            true to enable lower brightness mode, false for full brightness.
    @return None (void).
    @note   This has an immediate effect on the display, no need to call the
            display() function -- buffer contents are not changed.
*/
void Adafruit_R61505::dim(bool dim) {
  // the range of contrast to too small to be really useful
  // it is useful to dim the display
  TRANSACTION_START
  R61505_command1(R61505_SETCONTRAST);
  R61505_command1(dim ? 0 : contrast);
  TRANSACTION_END
}
