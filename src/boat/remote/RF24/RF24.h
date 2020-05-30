/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * @file RF24.h
 *
 * Class declaration for RF24 and helper enums
 */

#ifndef __RF24_H__
#define __RF24_H__

#include "RF24_config.h"

#if defined (RF24_LINUX) || defined (LITTLEWIRE)
    #include "utility/includes.h"
#elif defined SOFTSPI
    #include <DigitalIO.h>
#endif

/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum {
    RF24_PA_MIN = 0,
    RF24_PA_LOW,
    RF24_PA_HIGH,
    RF24_PA_MAX,
    RF24_PA_ERROR
} rf24_pa_dbm_e;

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum {
    RF24_1MBPS = 0,
    RF24_2MBPS,
    RF24_250KBPS
} rf24_datarate_e;

/**
 * CRC Length.  How big (if any) of a CRC is included.
 *
 * For use with setCRCLength()
 */
typedef enum {
    RF24_CRC_DISABLED = 0,
    RF24_CRC_8,
    RF24_CRC_16
} rf24_crclength_e;

/**
 * Driver for nRF24L01(+) 2.4GHz Wireless Transceiver
 */

/**
* SPI transactions
*
* Common code for SPI transactions including CSN toggle
*
*/
//inline void RF24_beginTransaction();
//inline void RF24_endTransaction();

void RF24_RF24();
bool RF24_begin(void);
bool RF24_isChipConnected();
void RF24_startListening(void);
void RF24_stopListening(void);
bool available(void);
void RF24_read(void* buf, unsigned char len);
bool RF24_write(const void* buf, unsigned char len);
void RF24_openWritingPipe(const unsigned char* address);
void RF24_openReadingPipe(unsigned char number, const unsigned char* address);
void RF24_printDetails(void);
bool RF24_available_pipe(unsigned char* pipe_num);
bool RF24_rxFifoFull();
void RF24_powerDown(void);
void RF24_powerUp(void);
bool RF24_write_multi(const void* buf, unsigned char len, const bool multicast);
//bool writeFast(const void* buf, unsigned char len);
bool RF24_writeFast_multi(const void* buf, unsigned char len, const bool multicast);
bool RF24_writeFast(const void* buf, unsigned char len);
bool RF24_writeBlocking(const void* buf, unsigned char len, uint32_t timeout);
bool txStandBy();
bool RF24_txStandBy_TO(uint32_t timeout, bool startTx);
void RF24_writeAckPayload(unsigned char pipe, const void* buf, unsigned char len);
void RF24_whatHappened(bool* tx_ok, bool* tx_fail, bool* rx_ready);
void RF24_startFastWrite(const void* buf, unsigned char len, const bool multicast, bool startTx);
void RF24_startWrite(const void* buf, unsigned char len, const bool multicast);
void RF24_reUseTX();
unsigned char RF24_flush_tx(void);
bool RF24_testCarrier(void);
bool RF24_testRPD(void);
void RF24_closeReadingPipe(unsigned char pipe);
//#if defined (FAILURE_HANDLING)
bool RF24_failureDetected;
//#endif
void RF24_setAddressWidth(unsigned char a_width);
void RF24_setRetries(unsigned char delay, unsigned char count);
void RF24_setChannel(unsigned char channel);
unsigned char RF24_getChannel(void);
void RF24_setPayloadSize(unsigned char size);
unsigned char RF24_getPayloadSize(void);
unsigned char RF24_getDynamicPayloadSize(void);
void RF24_enableAckPayload(void);
void RF24_enableDynamicPayloads(void);
void RF24_disableDynamicPayloads(void);
void RF24_enableDynamicAck();
bool RF24_isPVariant(void);
//void setAutoAck(bool enable);
void RF24_setAutoAck_pipe(unsigned char pipe, bool enable);
void RF24_setAutoAck(bool enable);
void RF24_setPALevel(unsigned char level);
unsigned char RF24_getPALevel(void);
unsigned char RF24_getARC(void);
bool RF24_setDataRate(rf24_datarate_e speed);
rf24_datarate_e RF24_getDataRate(void);
void RF24_setCRCLength(rf24_crclength_e length);
rf24_crclength_e RF24_getCRCLength(void);
void RF24_disableCRC(void);
void RF24_maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready);
//void openReadingPipe(unsigned char number, uint64_t address);
//void openWritingPipe(uint64_t address);
unsigned char RF24_flush_rx(void);
//inline void RF24_csn(bool mode);
//inline void RF24_ce(bool level);
unsigned char RF24_read_register(unsigned char regis, unsigned char* buf, unsigned char len);
unsigned char RF24_read_register1(unsigned char regis);
unsigned char RF24_write_register(unsigned char regis, const unsigned char* buf, unsigned char len);
unsigned char RF24_write_register1(unsigned char regis, unsigned char value);
unsigned char RF24_write_payload(const void* buf, unsigned char len, const unsigned char writeType);
unsigned char RF24_read_payload(void* buf, unsigned char len);
unsigned char RF24_get_status(void);
bool RF24_available(void);

#if !defined (MINIMAL)
void RF24_print_status(unsigned char status);
void RF24_print_observe_tx(unsigned char value);
void RF24_print_byte_register(const char* name, unsigned char reg, unsigned char qty);
void RF24_print_address_register(const char* name, unsigned char reg, unsigned char qty);
#endif

void RF24_toggle_features(void);
unsigned char RF24_spiTrans(unsigned char cmd);

#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
void RF24_errNotify(void);
#endif


#endif // __RF24_H__
