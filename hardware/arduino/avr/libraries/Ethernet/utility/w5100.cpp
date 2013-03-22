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
#include <avr/interrupt.h>

#include "w5100.h"

// W5100 controller instance
W5200Module W5100;

#define TX_RX_MAX_BUF_SIZE 2048
#define TX_BUF 0x1100
#define RX_BUF (TX_BUF + TX_RX_MAX_BUF_SIZE)

void WiznetModule::init()
{
  delay(300);

  SPI.begin();
  initSS();

  writeMR(1<<RST);
}

void W5200Module::init()
{
  WiznetModule::init();
  for (int i=0; i< get_max_sockets() ; i++) {
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
  read_data(s, (uint8_t *)ptr, data, len);
  if (!peek)
  {
    ptr += len;
    writeSnRX_RD(s, ptr);
  }
}

void WiznetModule::read_data(SOCKET s, volatile uint8_t *src, volatile uint8_t *dst, uint16_t len)
{
  uint16_t size;
  uint16_t src_mask;
  uint16_t src_ptr;

  src_mask = (uint16_t)src & RMASK;
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
  SPI.transfer(_addr >> 8);
  SPI.transfer(_addr & 0xFF);
  SPI.transfer(0x80);
  SPI.transfer(0x01);
  SPI.transfer(_data);
  resetSS();
  return 1;
}

uint8_t W5100Module::write(uint16_t _addr, uint8_t _data)
{
  setSS();
  SPI.transfer(0xF0);
  SPI.transfer(_addr >> 8);
  SPI.transfer(_addr & 0xFF);
  SPI.transfer(_data);
  resetSS();
  return 1;
}

uint16_t W5200Module::write(uint16_t _addr, const uint8_t *_buf, uint16_t _len)
{
  setSS();
  SPI.transfer(_addr >> 8);
  SPI.transfer(_addr & 0xFF);
  SPI.transfer((0x80 | ((_len & 0x7F00) >> 8)));
  SPI.transfer(_len & 0x00FF);
  for (uint16_t i=0; i<_len; i++) {
    SPI.transfer(_buf[i]);
  }
  resetSS();
  return _len;
}

uint16_t W5100Module::write(uint16_t _addr, const uint8_t *_buf, uint16_t _len)
{
  for (uint16_t i=0; i<_len; i++)
  {
    setSS();
    SPI.transfer(0xF0);
    SPI.transfer(_addr >> 8);
    SPI.transfer(_addr & 0xFF);
    _addr++;
    SPI.transfer(_buf[i]);
    resetSS();
  }
  return _len;
}

uint8_t W5200Module::read(uint16_t _addr)
{
  setSS();
  SPI.transfer(_addr >> 8);
  SPI.transfer(_addr & 0xFF);
  SPI.transfer(0x00);
  SPI.transfer(0x01);

  uint8_t _data = SPI.transfer(0);
  resetSS();
  return _data;
}

uint8_t W5100Module::read(uint16_t _addr)
{
  setSS();
  SPI.transfer(0x0F);
  SPI.transfer(_addr >> 8);
  SPI.transfer(_addr & 0xFF);

  uint8_t _data = SPI.transfer(0);
  resetSS();
  return _data;
}

uint16_t W5200Module::read(uint16_t _addr, uint8_t *_buf, uint16_t _len)
{
  setSS();
  SPI.transfer(_addr >> 8);
  SPI.transfer(_addr & 0xFF);
  SPI.transfer((0x00 | ((_len & 0x7F00) >> 8)));
  SPI.transfer(_len & 0x00FF);
  for (uint16_t i=0; i<_len; i++)
  {
    _buf[i] = SPI.transfer(0);
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
    SPI.transfer(0x0F);
    SPI.transfer(_addr >> 8);
    SPI.transfer(_addr & 0xFF);
    _addr++;
    _buf[i] = SPI.transfer(0);
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
