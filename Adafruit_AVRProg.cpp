// Useful message printing definitions
#include "Adafruit_AVRProg.h"
#include "Adafruit_UPDIProg.h"

static byte pageBuffer[8 * 1024]; // we can megabuff

/**************************************************************************/
/*!
    @brief  Instantiate a new programmer, no pins are defined to start
*/
/**************************************************************************/
Adafruit_AVRProg::Adafruit_AVRProg() {}

/**************************************************************************/
/*!
    @brief  Set up programming to use UPDI interface
    @param  theSerial Hardware serial interface (CANNOT use software serial!)
    @param  baudrate Try 115200 or less to start
    @param  power_pin pin connected to target chip power (can help with UPDI
   since theres no reset line)
    @param invertpower Set true if the power pin is low to enable target
*/
/**************************************************************************/
void Adafruit_AVRProg::setUPDI(HardwareSerial *theSerial, uint32_t baudrate) {
  uart = theSerial;
  _baudrate = baudrate;
}

bool Adafruit_AVRProg::startProgramMode() {
  updi_init(true);
  if (!updi_check()) {
    DEBUG_VERBOSE("UPDI not initialised\n");

    if (!updi_device_force_reset()) {
      DEBUG_VERBOSE("double BREAK reset failed\n");
      return false;
    }
    updi_init(false); // re-init the UPDI interface

    if (!updi_check()) {
      DEBUG_VERBOSE("Cannot initialise UPDI, aborting.\n");
      // TODO find out why these are not already correct
      g_updi.initialized = false;
      g_updi.unlocked = false;
      return false;
    } else {
      DEBUG_VERBOSE("UPDI INITIALISED\n");
      g_updi.initialized = true;
    }
  }
  return true;
}

/*******************************************************
 * ISP high level commands
 */

/**************************************************************************/
/*!
    @brief  Read the bottom two signature bytes (if possible) and return them
    Note that the highest signature byte is the same over all AVRs so we skip it
    @returns The two bytes as one uint16_t
*/
/**************************************************************************/
uint16_t Adafruit_AVRProg::readSignature(void) {
  updi_run_tasks(UPDI_TASK_GET_INFO, NULL);

  uint16_t sig = 0;
  sig = g_updi.details.signature_bytes[1];
  sig <<= 8;
  sig |= g_updi.details.signature_bytes[2];

  return sig;
}

/**************************************************************************/
/*!
    @brief    Send the erase command, then busy wait until the chip is erased
    @returns  True if erase command succeeds
*/
/**************************************************************************/
bool Adafruit_AVRProg::eraseChip(void) {
  return updi_run_tasks(UPDI_TASK_ERASE, NULL);
}

/**************************************************************************/
/*!
    @brief    Read the fuses on a device
    @param    fuses Pointer to 4-byte array of fuses
    @param    numbytes How many fuses to read (UPDI has 10?)
    @return True if we were able to send data and get a response from the chip.
    You could still run verifyFuses() afterwards!
*/
/**************************************************************************/
bool Adafruit_AVRProg::readFuses(byte *fuses, uint8_t numbytes) {
  (void)fuses[0];
  (void)numbytes;

  if (!updi_run_tasks(UPDI_TASK_READ_FUSES, NULL)) {
    return false;
  }
  for (uint8_t i = 0; i < numbytes; i++) {
    fuses[i] = g_updi.fuses[i];
  }
  return true;
}

/**************************************************************************/
/*!
    @brief    Program the fuses on a device
    @param    fuses Pointer to byte array of fuses
    @param    num_fuses How many fuses are in the fusearray
    @return True if we were able to send data and get a response from the chip.
    You could still run verifyFuses() afterwards!
*/
/**************************************************************************/
bool Adafruit_AVRProg::programFuses(const byte *fuses, uint8_t num_fuses) {
  startProgramMode();

  uint8_t old_fuses[10];
  if (!updi_run_tasks(UPDI_TASK_READ_FUSES, NULL)) {
    return false;
  }
  for (uint8_t i = 0; i < num_fuses; i++) {
    old_fuses[i] = g_updi.fuses[i];
  }

  for (uint8_t f = 0; f < num_fuses; f++) {
    if (f == (AVR_FUSE_LOCK - AVR_FUSE_BASE)) {
      continue;
    }

    g_updi.fuses[f] = fuses[f];
  }

  if (!updi_run_tasks(UPDI_TASK_WRITE_FUSES, NULL)) {
    return false;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief    Program a single fuse (currently for UPDI only)
    @param    fuse Value to write
    @param    num Fuse address offset (start at 0)
    @return  UPDI command success status
*/
/**************************************************************************/
bool Adafruit_AVRProg::programFuse(byte fuse, uint8_t num) {
  uint8_t old_fuses[10];
  if (!updi_run_tasks(UPDI_TASK_READ_FUSES, NULL)) {
    return false;
  }
  for (uint8_t i = 0; i < 10; i++) {
    old_fuses[i] = g_updi.fuses[i];
  }

  if (num == (AVR_FUSE_LOCK - AVR_FUSE_BASE)) {
    return false;
  }

  g_updi.fuses[num] = fuse;

  if (!updi_run_tasks(UPDI_TASK_WRITE_FUSES, NULL)) {
    return false;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief    Program the chip flash
    @param    hextext A pointer to a array of bytes in INTEL HEX format
    @param    pagesize The flash-page size of this chip, in bytes. Check
   datasheet!
    @param    chipsize The flash size of this chip, in bytes. Check datasheet!
    @return True if flashing worked out, check the data with verifyImage!
*/
/**************************************************************************/
bool Adafruit_AVRProg::writeImage(const byte *hextext, uint32_t pagesize,
                                  uint32_t chipsize) {
  uint32_t flash_start = 0;
#ifdef SUPPORT_UPDI
  flash_start = g_updi.config->flash_start;
#endif

  uint32_t pageaddr = 0;

  while (pageaddr < chipsize && hextext) {
    const byte *hextextpos =
        readImagePage(hextext, pageaddr, pagesize, pageBuffer);

    bool blankpage = true;
    for (uint16_t i = 0; i < pagesize; i++) {
      if (pageBuffer[i] != 0xFF)
        blankpage = false;
    }
    if (!blankpage) {
      if (!flashPage(pageBuffer, flash_start + pageaddr, pagesize))
        return false;
    }
    hextext = hextextpos;
    pageaddr += pagesize;
  }
  return true;
}

/*
 * readImagePage
 *
 * Read a page of intel hex image from a string. Returns a pointer
 * to where we ended
 */

const byte *Adafruit_AVRProg::readImagePage(const byte *hextext,
                                            uint16_t pageaddr,
                                            uint16_t pagesize, byte *page) {
  uint16_t len;
  uint16_t page_idx = 0;
  const byte *beginning = hextext;

  byte b, cksum = 0;

  // 'empty' the page by filling it with 0xFF's
  for (uint16_t i = 0; i < pagesize; i++)
    page[i] = 0xFF;

  while (1) {
    uint16_t lineaddr;
    char c;

    // read one line!
    c = *hextext++;
    if (c == '\n' || c == '\r') {
      continue;
    }
    if (c != ':') {
      error(F(" No colon?"));
      return NULL;
    }
    // Read the byte count into 'len'
    len = hexToByte(*hextext++);
    len = (len << 4) + hexToByte(*hextext++);
    cksum = len;
    // Serial.print(len);

    // read high address byte
    b = hexToByte(*hextext++);
    b = (b << 4) + hexToByte(*hextext++);
    cksum += b;
    lineaddr = b;

    // read low address byte
    b = hexToByte(*hextext++);
    b = (b << 4) + hexToByte(*hextext++);
    cksum += b;
    lineaddr = (lineaddr << 8) + b;

    if (lineaddr >= (pageaddr + pagesize)) {
      return beginning;
    }

    b = hexToByte(*hextext++); // record type
    b = (b << 4) + hexToByte(*hextext++);
    cksum += b;
    if (b == 0x1) {
      // end record, return nullptr to indicate we're done
      hextext = nullptr;
      break;
    }

    for (byte i = 0; i < len; i++) {
      // read 'n' bytes
      b = hexToByte(*hextext++);
      b = (b << 4) + hexToByte(*hextext++);

      cksum += b;

      page[page_idx] = b;
      page_idx++;

      if (page_idx > pagesize) {
        error("Too much code!");
        return NULL;
      }
    }
    b = hexToByte(*hextext++); // chxsum
    b = (b << 4) + hexToByte(*hextext++);
    cksum += b;
    if (cksum != 0) {
      error(F("Bad checksum: "));
      return NULL;
    }
    if (*hextext++ != '\n') {
      error(F("No end of line"));
      return NULL;
    }

    if (page_idx == pagesize)
      break;
  }

  return hextext;
}

/**************************************************************************/
/*!
    @brief    Does a byte-by-byte verify of the flash hex against the chip
    Thankfully this does not have to be done by pages!
    @param    hextext A pointer to a array of bytes in INTEL HEX format
    @return True if the image is the same as the hextext, returns false on any
    error
*/
/**************************************************************************/
bool Adafruit_AVRProg::verifyImage(const byte *hextext) {
  uint32_t pageaddr = 0;
  // uint16_t pagesize = g_updi.config->flash_pagesize;
  uint16_t pagebuffersize = AVR_PAGESIZE_MAX;
  uint16_t chipsize = g_updi.config->flash_size;
  uint8_t buffer1[pagebuffersize], buffer2[pagebuffersize];

  while ((pageaddr < chipsize) && hextext) {
    const byte *hextextpos =
        readImagePage(hextext, pageaddr, pagebuffersize, buffer1);

    if (!updi_run_tasks(UPDI_TASK_READ_FLASH, buffer2,
                        g_updi.config->flash_start + pageaddr,
                        pagebuffersize)) {
      return false;
    }

    for (uint16_t x = 0; x < pagebuffersize; x++) {
      if (buffer1[x] != buffer2[x]) {
        return false;
      }
    }
    hextext = hextextpos;
    pageaddr += pagebuffersize;
  }
  return true;
}

/*******************************************************
 * Functions specific to ISP programming of an AVR
 */

// Basically, write the pagebuff (with pagesize bytes in it) into page $pageaddr
bool Adafruit_AVRProg::flashPage(byte *pagebuff, uint16_t pageaddr,
                                 uint16_t pagesize) {
  bool x = updi_run_tasks(UPDI_TASK_WRITE_FLASH, pagebuff, pageaddr, pagesize);
  return x;
}

/**************************************************************************/
/*!
 @brief  Function to write a byte to certain address in flash without
 page erase. Useful for parameters.
 @param    addr Flash address you want to write to.
 @param    pagesize The flash-page size of this chip, in bytes. Check
 datasheet!
 @param    content The byte you want to write to.
 @return True if flashing worked out.
 */
/**************************************************************************/
bool Adafruit_AVRProg::writeByteToFlash(unsigned int addr, uint16_t pagesize,
                                        uint8_t content) {
  // calculate page number and offset.
  memset(pageBuffer, 0xFF, pagesize);
  uint16_t pageOffset = addr & (pagesize - 1);
  pageBuffer[pageOffset] = content;
  return flashPage(pageBuffer, addr, pagesize);
}

/*******************************************************
 * Low level support functions
 */

/*
 * hexToByte
 * Turn a Hex digit (0..9, A..F) into the equivalent binary value (0-16)
 */
byte Adafruit_AVRProg::hexToByte(byte h) {
  if (h >= '0' && h <= '9')
    return (h - '0');
  if (h >= 'A' && h <= 'F')
    return ((h - 'A') + 10);
  if (h >= 'a' && h <= 'f')
    return ((h - 'a') + 10);
  error(F("Bad hex digit!"));
  return -1;
}

/**************************************************************************/
/*!
    @brief  Turn a pin on and off a few times; indicates life via LED
    @param  pin The arduino pin connected to the LED
    @param  times how many times to blink!
*/
/**************************************************************************/
void Adafruit_AVRProg::pulseLED(int pin, int times) {
  uint8_t PTIME = 30;
  pinMode(pin, OUTPUT);
  do {
    digitalWrite(pin, HIGH);
    delay(PTIME);
    digitalWrite(pin, LOW);
    delay(PTIME);
  } while (times--);
}

/**************************************************************************/
/*!
    @brief  Print an error, turn on the error LED and halt!
    @param  string What to print out before halting
*/
/**************************************************************************/
void Adafruit_AVRProg::error(const char *string) {
  // Serial.println(string);
  // while (1) {
  // }
}

/**************************************************************************/
/*!
    @brief  Print an error, turn on the error LED and halt!
    @param  string What to print out before halting
*/
/**************************************************************************/
void Adafruit_AVRProg::error(const __FlashStringHelper *string) {
  // Serial.println(string);
  // while (1) {
  // }
}
