#ifndef _ADAFRUIT_AVRPROG_H
#define _ADAFRUIT_AVRPROG_H

#include <Arduino.h>
#include "Adafruit_UPDIProg.h"

#define FUSE_PROT 0 ///< memory protection
#define FUSE_LOW 1  ///< Low fuse
#define FUSE_HIGH 2 ///< High fuse
#define FUSE_EXT 3  ///< Extended fuse

/**! Struct for holding one 'program' fuses & code */
typedef struct image {
  char image_name[30];      ///< i.e. "optiboot_diecimila.hex"
  char image_chipname[12];  ///< i.e. "atmega168"
  uint16_t image_chipsig;   ///< Low two bytes of signature, check datasheet!
  byte image_progfuses[10]; ///< fuses to set during programming (e.g unlock)
  byte image_normfuses[10]; ///< fuses to set after programming (e.g lock)
  byte fusemask[10];   ///< Not all bits are used in the fuses, mask the ones we
                       ///< do use
  uint16_t chipsize;   ///< Total size for flash programming, in bytes. check
                       ///< datasheet!
  byte image_pagesize; ///< Page size for flash programming, in bytes. check
                       ///< datasheet!
#if defined(__SAMD21G18A__) || defined(__SAMD21E18A__) ||                      \
    defined(TEENSYDUINO) || defined(ESP8266) || defined(ESP32) ||              \
    defined(__SAMD51__)
  byte image_hexcode[60000]; ///< Max buffer for intel hex format image (text)
#else
  byte image_hexcode[10000]; ///< Max buffer for intel hex format image (text)
#endif
} image_t;

// Useful message printing definitions

#define debug(string) // Serial.println(string);

/*!
  @brief    Class for talking to an AVR chip to program flash and fuses over UPDI
*/
class Adafruit_AVRProg {

public:
  Adafruit_AVRProg();

  void pulseLED(int pin, int times);
  bool eraseChip(void);

  bool readFuses(byte *fuses, uint8_t numbytes);
  bool programFuses(const byte *fuses, uint8_t num_fuses = 5);
  bool programFuse(byte fuse, uint8_t num);

  bool writeImage(const byte *hextext, uint32_t pagesize, uint32_t chipsize);
  bool verifyImage(const byte *hextext);

  void setUPDI(HardwareSerial *theUART, uint32_t baudrate);
  bool UPDIunlock();

private:
  void updi_serial_init(void);
  int updi_serial_read_wait(void);
  bool updi_serial_send(uint8_t *data, uint16_t size);
  bool updi_serial_send_receive(uint8_t *data, uint16_t size, uint8_t *buff,
                                uint32_t len);

  void updi_send_handshake(void);
  void updi_serial_term();
  void updi_term();

  bool udpi_stcs(uint8_t address, uint8_t value);
  uint8_t updi_ldcs(uint8_t address);
  uint8_t updi_ld(uint16_t address);
  bool updi_st(uint32_t address, uint8_t value);
  bool updi_ld_ptr_inc(uint8_t *buffer, uint16_t size);

  bool updi_write_data(uint32_t address, uint8_t *data, uint32_t len);
  bool updi_write_data_words(uint32_t address, uint8_t *data,
                             uint32_t numwords);
  bool updi_read_data(uint32_t address, uint8_t *buf, uint32_t size);

  void updi_st_ptr_inc16(uint8_t *data, uint32_t numwords);
  void updi_set_repeat(uint16_t repeats);
  bool updi_st_ptr_inc(uint8_t *data, uint32_t size);
  bool updi_st_ptr(uint32_t address);

  bool updi_check(void);
  bool updi_device_force_reset(void);
  void updi_serial_force_break(void);
  bool updiIsConnected(bool silent);
  bool updi_init(bool force);
  bool updi_run_tasks(uint16_t tasks, uint8_t *data = NULL,
                      uint32_t address = 0, uint32_t size = 0);

  bool updi_wait_flash_ready();
  bool updi_write_nvm(uint32_t address, uint8_t *data, uint32_t len,
                      uint8_t command, bool use_word_acess,
                      bool block_on_flash = true, bool verify = true);
  bool updi_execute_nvm_command(uint8_t command);

  void updi_apply_reset();
  bool updi_wait_unlocked(uint32_t timeout);
  bool updi_is_prog_mode();
  bool updi_progmode_key();
  bool updi_enter_progmode();
  void updi_leave_progmode();
  bool updi_unlock_device();
  bool updi_get_device_info();

  bool updi_write_fuse(uint8_t fuse, uint8_t value);
  uint8_t updi_read_fuse(uint8_t fuse);

  bool updi_read_page(uint16_t address, uint16_t pagesize, uint8_t *data);
  bool updi_write_page(uint16_t address, uint16_t pagesize, uint8_t *pagedata);

  bool updi_erase_chip();
  void updi_write_key(uint8_t size, uint8_t *key);
  bool updi_chip_data_init_info(uint16_t sig, char *shortname, bool format);
  DeviceIdentification *updi_chip_lookup(uint16_t sig, char *name);

public:
  bool writeByteToFlash(unsigned int addr, uint16_t pagesize, uint8_t content);

  uint16_t readSignature(void);

  void error(const char *string);
  void error(const __FlashStringHelper *string);

  UPDI g_updi; ///< Global UPDI status

private:
  bool startProgramMode();
  bool flashPage(byte *pagebuff, uint16_t pageaddr, uint16_t pagesize);
  const byte *readImagePage(const byte *hextext, uint16_t pageaddr,
                            uint16_t pagesize, byte *page);
  byte hexToByte(byte h);

  HardwareSerial *uart = NULL;
  uint32_t _baudrate;
  uint8_t _updi_serial_retry_counter = 0; // resets after success or failure
  uint16_t _updi_serial_retry_count = 0;  // used for diagnostics
  bool _updi_serial_inited = false;
};

#endif
