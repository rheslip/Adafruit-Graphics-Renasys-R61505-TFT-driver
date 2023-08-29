/*!
 * @file Adafruit_R61505.h
 *
 * This is part of for Adafruit's R61505 library for monochrome
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
 * Written by Limor Fried/Ladyada for Adafruit Industries, with
 * contributions from the open source community.
 *
 * BSD license, all text above, and the splash screen header file,
 * must be included in any redistribution.
 *
 */

#ifndef _Adafruit_R61505_H_
#define _Adafruit_R61505_H_

// ONE of the following three lines must be #defined:
//#define R61505_128_64 ///< DEPRECTAED: old way to specify 128x64 screen
#define R61505_128_32 ///< DEPRECATED: old way to specify 128x32 screen
//#define R61505_96_16  ///< DEPRECATED: old way to specify 96x16 screen
// This establishes the screen dimensions in old Adafruit_R61505 sketches
// (NEW CODE SHOULD IGNORE THIS, USE THE CONSTRUCTORS THAT ACCEPT WIDTH
// AND HEIGHT ARGUMENTS).

// Uncomment to disable Adafruit splash logo
//#define R61505_NO_SPLASH

#if defined(ARDUINO_STM32_FEATHER)
typedef class HardwareSPI SPIClass;
#endif

#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>

#if defined(__AVR__)
typedef volatile uint8_t PortReg;
typedef uint8_t PortMask;
#define HAVE_PORTREG
#elif defined(__SAM3X8E__)
typedef volatile RwReg PortReg;
typedef uint32_t PortMask;
#define HAVE_PORTREG
#elif (defined(__arm__) || defined(ARDUINO_FEATHER52)) &&                      \
    !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_RP2040)
typedef volatile uint32_t PortReg;
typedef uint32_t PortMask;
#define HAVE_PORTREG
#endif

/// The following "raw" color names are kept for backwards client compatability
/// They can be disabled by predefining this macro before including the Adafruit
/// header client code will then need to be modified to use the scoped enum
/// values directly
#ifndef NO_ADAFRUIT_R61505_COLOR_COMPATIBILITY
#define	BLACK   0xFFFF
#define	BLUE    0xffe0
#define	RED    0x07ff
#define	GREEN   0xf81f
#define CYAN    (BLUE & GREEN)
#define MAGENTA (BLUE & RED)
#define YELLOW  (RED & GREEN)
#define WHITE   0x0000
#endif
/// fit into the R61505_ naming scheme


#define R61505_MEMORYMODE 0x20          ///< See datasheet
#define R61505_COLUMNADDR 0x21          ///< See datasheet
#define R61505_PAGEADDR 0x22            ///< See datasheet
#define R61505_SETCONTRAST 0x81         ///< See datasheet
#define R61505_CHARGEPUMP 0x8D          ///< See datasheet
#define R61505_SEGREMAP 0xA0            ///< See datasheet
#define R61505_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define R61505_DISPLAYALLON 0xA5        ///< Not currently used
#define R61505_NORMALDISPLAY 0xA6       ///< See datasheet
#define R61505_INVERTDISPLAY 0xA7       ///< See datasheet
#define R61505_SETMULTIPLEX 0xA8        ///< See datasheet
#define R61505_DISPLAYOFF 0xAE          ///< See datasheet
#define R61505_DISPLAYON 0xAF           ///< See datasheet
#define R61505_COMSCANINC 0xC0          ///< Not currently used
#define R61505_COMSCANDEC 0xC8          ///< See datasheet
#define R61505_SETDISPLAYOFFSET 0xD3    ///< See datasheet
#define R61505_SETDISPLAYCLOCKDIV 0xD5  ///< See datasheet
#define R61505_SETPRECHARGE 0xD9        ///< See datasheet
#define R61505_SETCOMPINS 0xDA          ///< See datasheet
#define R61505_SETVCOMDETECT 0xDB       ///< See datasheet

#define R61505_SETLOWCOLUMN 0x00  ///< Not currently used
#define R61505_SETHIGHCOLUMN 0x10 ///< Not currently used
#define R61505_SETSTARTLINE 0x40  ///< See datasheet

#define R61505_EXTERNALVCC 0x01  ///< External display voltage source
#define R61505_SWITCHCAPVCC 0x02 ///< Gen. display voltage from 3.3V

#define R61505_RIGHT_HORIZONTAL_SCROLL 0x26              ///< Init rt scroll
#define R61505_LEFT_HORIZONTAL_SCROLL 0x27               ///< Init left scroll
#define R61505_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29 ///< Init diag scroll
#define R61505_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A  ///< Init diag scroll
#define R61505_DEACTIVATE_SCROLL 0x2E                    ///< Stop scroll
#define R61505_ACTIVATE_SCROLL 0x2F                      ///< Start scroll
#define R61505_SET_VERTICAL_SCROLL_AREA 0xA3             ///< Set scroll range

// Deprecated size stuff for backwards compatibility with old sketches
#if defined R61505_128_64
#define R61505_LCDWIDTH 128 ///< DEPRECATED: width w/R61505_128_64 defined
#define R61505_LCDHEIGHT 64 ///< DEPRECATED: height w/R61505_128_64 defined
#endif
#if defined R61505_128_32
#define R61505_LCDWIDTH 128 ///< DEPRECATED: width w/R61505_128_32 defined
#define R61505_LCDHEIGHT 32 ///< DEPRECATED: height w/R61505_128_32 defined
#endif
#if defined R61505_96_16
#define R61505_LCDWIDTH 96  ///< DEPRECATED: width w/R61505_96_16 defined
#define R61505_LCDHEIGHT 16 ///< DEPRECATED: height w/R61505_96_16 defined
#endif

/*!
    @brief  Class that stores state and functions for interacting with
            R61505 OLED displays.
*/
class Adafruit_R61505 : public Adafruit_GFX {
public:
  // NEW CONSTRUCTORS -- recommended for new projects
  Adafruit_R61505(uint16_t w, uint16_t h, TwoWire *twi = &Wire,
                   int8_t rst_pin = -1, uint32_t clkDuring = 400000UL,
                   uint32_t clkAfter = 100000UL);
  Adafruit_R61505(uint16_t w, uint16_t h, int8_t mosi_pin, int8_t sclk_pin,
                   int8_t dc_pin, int8_t rst_pin, int8_t cs_pin);
  Adafruit_R61505(uint16_t w, uint16_t h, SPIClass *spi, int8_t dc_pin,
                   int8_t rst_pin, int8_t cs_pin, uint32_t bitrate = 8000000UL);

  // DEPRECATED CONSTRUCTORS - for back compatibility, avoid in new projects
  Adafruit_R61505(int8_t mosi_pin, int8_t sclk_pin, int8_t dc_pin,
                   int8_t rst_pin, int8_t cs_pin);
  Adafruit_R61505(int8_t dc_pin, int8_t rst_pin, int8_t cs_pin);
  Adafruit_R61505(int8_t rst_pin = -1);

  ~Adafruit_R61505(void);

  bool begin(bool reset = true, bool periphBegin = true);
  void clearDisplay(void);
  void display(void);
  void invertDisplay(bool i);
  void dim(bool dim);
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color);
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1);
  virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  void startscrollright(uint8_t start, uint8_t stop);
  void startscrollleft(uint8_t start, uint8_t stop);
  void startscrolldiagright(uint8_t start, uint8_t stop);
  void startscrolldiagleft(uint8_t start, uint8_t stop);
  void stopscroll(void);
  void R61505_command(uint8_t c);
  uint16_t color565(uint8_t r, uint8_t g, uint8_t b);


protected:
  inline void SPIwrite(uint8_t d) __attribute__((always_inline));
  void writecommand( uint8_t index, uint16_t data );
  void R61505_command1(uint8_t c);
  void R61505_commandList(const uint8_t *c, uint8_t n);

  SPIClass *spi;   ///< Initialized during construction when using SPI. See
                   ///< SPI.cpp, SPI.h
  TwoWire *wire;   ///< Initialized during construction when using I2C. See
                   ///< Wire.cpp, Wire.h RH not used but needed by adafruit_GFX?
  uint8_t *buffer; ///< Buffer data used for display buffer. RH not used but needed by adafruit_GFX?
                  
  int8_t page_end; ///< not used
  int8_t mosiPin;  ///< (Master Out Slave In) set when using SPI set during
                   ///< construction.
  int8_t clkPin;   ///< (Clock Pin) set when using SPI set during construction.
  int8_t dcPin;    ///< (Data Pin) set when using SPI set during construction.
  int8_t
      csPin; ///< (Chip Select Pin) set when using SPI set during construction.
  int8_t rstPin; ///< Display reset pin assignment. Set during construction.

#ifdef HAVE_PORTREG
  PortReg *mosiPort, *clkPort, *dcPort, *csPort;
  PortMask mosiPinMask, clkPinMask, dcPinMask, csPinMask;
#endif
#if ARDUINO >= 157
  uint32_t wireClk;    ///< Wire speed for R61505 transfers
  uint32_t restoreClk; ///< Wire speed following R61505 transfers
#endif
  uint8_t contrast; ///< normal contrast setting for this device
#if defined(SPI_HAS_TRANSACTION)
protected:
  // Allow sub-class to change
  SPISettings spiSettings;
#endif
};

#endif // _Adafruit_R61505_H_
