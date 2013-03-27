/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include <stdio.h>
#include <string.h>
#ifdef __AVR__
#include <new.h>
#include <avr/interrupt.h>
#endif

#include "w5100.h"

// W5x00 controller instance, do not access before calling initialise_wiznet_instance()
const size_t sizeof_wiznetmodule = max(sizeof(W5100Module), sizeof(W5200Module));
uint8_t _w5100_storage[sizeof_wiznetmodule];
WiznetModule &W5100 = *((WiznetModule *)_w5100_storage);

void initialise_wiznet_instance() {
  // This is pretty hacky. In order to avoid changing W5100 to a
  // pointer type we just copy our heap-allocated WiznetModule
  // instance into the W5100 object's storage space, then delete the
  // heap-allocated version.
  WiznetModule *w5100_ptr = WiznetModule::autodetect();
  memcpy(_w5100_storage, w5100_ptr, sizeof(WiznetModule));
  delete w5100_ptr;
}

const uint8_t W5200_WRITE_FLAG = 0x80;
const uint8_t W5200_READ_FLAG = 0x00;

const uint8_t W5100_WRITE_FLAG = 0xF0;
const uint8_t W5100_READ_FLAG = 0x0F;

#define TX_RX_MAX_BUF_SIZE 2048
#define TX_BUF 0x1100
#define RX_BUF (TX_BUF + TX_RX_MAX_BUF_SIZE)

#ifdef __AVR__
#define SPI_CONTINUE 0
#define SPI_LAST     0
inline static uint8_t _spi_transfer(uint8_t _data, uint8_t _mode) {
  return SPI.transfer(_data);
}

#else // ARM
#define SPI_CS 10
inline static uint8_t _spi_transfer(uint8_t _data, SPITransferMode _mode) {
  return SPI.transfer(SPI_CS, _data, _mode);
}
#endif


WiznetModule *WiznetModule::autodetect()
{
  /* Method
   *
   * Start by assuming we have a W5100, set the reset bit in the mode register and then read
   * back the mode register to check it's zeroed (ie reset complete.)
   *
   * If we don't get a zero back, assume we have a W5200 in which case
   * we're in the middle of a register read. Finish the request that
   * we've accidentally given it.
   */

#ifdef __AVR__
  initSS();
  resetSS();
  SPI.begin();
#else // ARM
  SPI.begin(SPI_CS);
  // Set clock to 4Mhz (W5100 should support up to about 14Mhz)
  SPI.setClockDivider(SPI_CS, 21);
  SPI.setDataMode(SPI_CS, SPI_MODE0);
#endif

  // To check for W5100, send a sequence of mode bits and check we read
  // back consistent mode bit values in all cases (for W5200 this will just
  // be reading the same memory address over and over, so result shouldn't change.)
  bool is_w5100 = exploratory_modewrite(RST) == 0
    && exploratory_modewrite(PINGBLOCK) == PINGBLOCK
    && exploratory_modewrite(PPOE) == PPOE
    && exploratory_modewrite(RST|PPOE) == 0;

  if(is_w5100)
    return new W5100Module();
  else
    return new W5200Module();
}

uint8_t WiznetModule::exploratory_modewrite(uint8_t mode_value)
{
  /* Writes and then reads back the mode register on W5100
     However, assumes device type is unknown and may be W5200,
     so the device also reads back additional bytes as if the same
     command had been sent to W5200 (to ensure a consistent device state)
  */

  // W5100 mode write sequence, on W5200 this is interpreted as a request
  // to read 'mode_value' bytes from address 0xF000
  setSS();
  _spi_transfer(W5100_WRITE_FLAG,       SPI_CONTINUE);
  _spi_transfer(0,                      SPI_CONTINUE);
  _spi_transfer(0,                      SPI_CONTINUE);
  _spi_transfer(mode_value, SPI_LAST);
  resetSS();

  // Read back the value, on W5200 this is interpreted as the first 4
  // reads from the sequence.
  setSS();
  _spi_transfer(W5100_READ_FLAG,       SPI_CONTINUE);
  _spi_transfer(0,                      SPI_CONTINUE);
  _spi_transfer(0,                      SPI_CONTINUE);
  uint8_t result = _spi_transfer(0, SPI_LAST);
  resetSS();

  // In case this is a W5200, complete the read cycles for the
  // device (NOPs on W5100 as no command flag will be sent)
  setSS();
  for(uint8_t i = 4; i < mode_value; i++)
    _spi_transfer(0, SPI_LAST);
  resetSS();

  return result;
}

void WiznetModule::init()
{
  delay(300);

#ifdef __AVR__
  SPI.begin();
  initSS();
#else // ARM
  SPI.begin(SPI_CS);
  // Set clock to 4Mhz (W5100 should support up to about 14Mhz)
  SPI.setClockDivider(SPI_CS, 21);
  SPI.setDataMode(SPI_CS, SPI_MODE0);
#endif

  writeMR(WiznetModule::RST);
}

void W5200Module::init()
{
  WiznetModule::init();
  for (uint8_t i=0; i< get_max_sockets() ; i++) {
    write(get_chbase() + i * 0x100 + 0x001F, 2);
    write(get_chbase() + i * 0x100 + 0x001E, 2);
  }
}

void W5100Module::init()
{
  WiznetModule::init();
  writeTMSR(0x55);
  writeRMSR(0x55);
}

uint16_t WiznetModule::getTXFreeSize(SOCKET s){

  uint16_t val=0, val1=0;
  do {
    val1 = readSnTX_FSR(s);
    if (val1 != 0)
      val = readSnTX_FSR(s);
  }
  while (val != val1);
  return val;
}

uint16_t WiznetModule::getRXReceivedSize(SOCKET s)
{
  uint16_t val=0,val1=0;
  do {
    val1 = readSnRX_RSR(s);
    if (val1 != 0)
      val = readSnRX_RSR(s);
  }
  while (val != val1);
  return val;
}


void WiznetModule::send_data_processing(SOCKET s, const uint8_t *data, uint16_t len)
{
  // This is same as having no offset in a call to send_data_processing_offset
  send_data_processing_offset(s, 0, data, len);
}

void WiznetModule::send_data_processing_offset(SOCKET s, uint16_t data_offset, const uint8_t *data, uint16_t len)
{
  uint16_t ptr = readSnTX_WR(s);
  ptr += data_offset;
  uint16_t offset = ptr & SMASK;
  uint16_t dstAddr = offset + get_sock_tx_addr(s);

  if (offset + len > SSIZE)
  {
    // Wrap around circular buffer
    uint16_t size = SSIZE - offset;
    write(dstAddr, data, size);
    write(get_sock_tx_addr(s), data + size, len - size);
  }
  else {
    write(dstAddr, data, len);
  }

  ptr += len;
  writeSnTX_WR(s, ptr);
}


void WiznetModule::recv_data_processing(SOCKET s, uint8_t *data, uint16_t len, uint8_t peek)
{
  uint16_t ptr;
  ptr = readSnRX_RD(s);
  read_data(s, ptr, data, len);
  if (!peek)
  {
    ptr += len;
    writeSnRX_RD(s, ptr);
  }
}

void WiznetModule::read_data(SOCKET s, volatile uint16_t src, volatile uint8_t *dst, uint16_t len)
{
  uint16_t size;
  uint16_t src_mask;
  uint16_t src_ptr;

  src_mask = src & RMASK;
  src_ptr = get_sock_rx_addr(s) + src_mask;

  if( (src_mask + len) > RSIZE )
  {
    size = RSIZE - src_mask;
    read(src_ptr, (uint8_t *)dst, size);
    dst += size;
    read(get_sock_rx_addr(s), (uint8_t *) dst, len - size);
  }
  else
    read(src_ptr, (uint8_t *) dst, len);
}


uint8_t W5200Module::write(uint16_t _addr, uint8_t _data)
{
  setSS();
  _spi_transfer(_addr >> 8,       SPI_CONTINUE);
  _spi_transfer(_addr & 0xFF,     SPI_CONTINUE);
  _spi_transfer(W5200_WRITE_FLAG, SPI_CONTINUE);
  _spi_transfer(0x01,             SPI_CONTINUE);
  _spi_transfer(_data,            SPI_LAST);
  resetSS();
  return 1;
}

uint8_t W5100Module::write(uint16_t _addr, uint8_t _data)
{
  setSS();
  _spi_transfer(W5100_WRITE_FLAG,  SPI_CONTINUE);
  _spi_transfer(_addr >> 8,        SPI_CONTINUE);
  _spi_transfer(_addr & 0xFF,      SPI_CONTINUE);
  _spi_transfer(_data,             SPI_LAST);
  resetSS();
  return 1;
}

uint16_t W5200Module::write(uint16_t _addr, const uint8_t *_buf, uint16_t _len)
{
  setSS();
  _spi_transfer(_addr >> 8,                                SPI_CONTINUE);
  _spi_transfer(_addr & 0xFF,                              SPI_CONTINUE);
  _spi_transfer(W5200_WRITE_FLAG | ((_len & 0x7F00) >> 8), SPI_CONTINUE);
  _spi_transfer(_len & 0xFF,                               SPI_CONTINUE);
  for (uint16_t i=0; i<_len; i++) {
    _spi_transfer(_buf[i], i==_len-1 ? SPI_LAST : SPI_CONTINUE);
  }
  resetSS();
  return _len;
}

uint16_t W5100Module::write(uint16_t _addr, const uint8_t *_buf, uint16_t _len)
{
  for (uint16_t i=0; i<_len; i++)
  {
    setSS();
    _spi_transfer(W5100_WRITE_FLAG,    SPI_CONTINUE);
    _spi_transfer(_addr >> 8,          SPI_CONTINUE);
    _spi_transfer(_addr & 0xFF,        SPI_CONTINUE);
    _addr++;
    _spi_transfer(_buf[i],             SPI_LAST);
    resetSS();
  }
  return _len;
}

uint8_t W5200Module::read(uint16_t _addr)
{
  setSS();
  _spi_transfer(_addr >> 8,            SPI_CONTINUE);
  _spi_transfer(_addr & 0xFF,          SPI_CONTINUE);
  _spi_transfer(W5200_READ_FLAG,       SPI_CONTINUE);
  _spi_transfer(0x01,                  SPI_CONTINUE);

  uint8_t _data = _spi_transfer(0,     SPI_LAST);
  resetSS();
  return _data;
}

uint8_t W5100Module::read(uint16_t _addr)
{
  setSS();
  _spi_transfer(W5100_READ_FLAG,       SPI_CONTINUE);
  _spi_transfer(_addr >> 8,            SPI_CONTINUE);
  _spi_transfer(_addr & 0xFF,          SPI_CONTINUE);

  uint8_t _data = _spi_transfer(0,     SPI_LAST);
  resetSS();
  return _data;
}

uint16_t W5200Module::read(uint16_t _addr, uint8_t *_buf, uint16_t _len)
{
  setSS();
  _spi_transfer(_addr >> 8,                               SPI_CONTINUE);
  _spi_transfer(_addr & 0xFF,                             SPI_CONTINUE);
  _spi_transfer(W5200_READ_FLAG | ((_len & 0x7F00) >> 8), SPI_CONTINUE);
  _spi_transfer(_len & 0xFF,                              SPI_CONTINUE);
  for (uint16_t i=0; i<_len; i++)
  {
    _buf[i] = _spi_transfer(0, i==_len-1 ? SPI_LAST : SPI_CONTINUE);
  }
  resetSS();
  return _len;
}

uint16_t W5100Module::read(uint16_t _addr, uint8_t *_buf, uint16_t _len)
{
  setSS();
  for (uint16_t i=0; i<_len; i++)
  {
    setSS();
    _spi_transfer(W5100_READ_FLAG,    SPI_CONTINUE);
    _spi_transfer(_addr >> 8,         SPI_CONTINUE);
    _spi_transfer(_addr & 0xFF,       SPI_CONTINUE);
    _addr++;
    _buf[i] = _spi_transfer(0,        SPI_LAST);
    resetSS();
  }
  return _len;
}

void WiznetModule::execCmdSn(SOCKET s, SockCMD _cmd) {
  // Send command to socket
  writeSnCR(s, _cmd);
  // Wait for command to complete
  while (readSnCR(s))
    ;
}
