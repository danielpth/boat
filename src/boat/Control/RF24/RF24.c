/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */
#include <atmel_start.h>
#include <string.h>
#include "nRF24L01.h"
#include "RF24_config.h"
#include "RF24.h"


unsigned short ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
unsigned short csn_pin; /**< SPI Chip select */
unsigned short spi_speed; /**< SPI Bus Speed */
#if defined (RF24_LINUX) || defined (XMEGA_D3)
unsigned char spi_rxbuff[32+1] ; //SPI receive buffer (payload max 32 bytes)
unsigned char spi_txbuff[32+1] ; //SPI transmit buffer (payload max 32 bytes + 1 byte for the command)
#endif
bool p_variant; /* False for RF24L01 and true for RF24L01P */
unsigned char payload_size; /**< Fixed size of payloads */
bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
unsigned char pipe0_reading_address[5]; /**< Last address set on pipe 0 for reading. */
unsigned char addr_width; /**< The address width to use - 3,4 or 5 bytes. */
unsigned long csDelay;
unsigned long  txDelay;

unsigned char SPI_transfer(unsigned char b);

inline void digitalWrite(unsigned char pin, bool value) 
{
	#warning "TODO"
    //digitalWrite(pin, value ? HIGH : LOW);
}

void pinMode(unsigned short pin, unsigned char dir)
{
	#warning "TODO"
	return;
}

inline static void RF24_delayMicroseconds(unsigned long us)
{
	while (us--) _delay_us (1);
}

/****************************************************************************/

/**
    * Test whether this is a real radio, or a mock shim for
    * debugging.  Setting either pin to 0xff is the way to
    * indicate that this is not a real radio.
    *
    * @return true if this is a legitimate radio
    */
bool RF24_isValid()
{
    return ce_pin != 0xff && csn_pin != 0xff;
}
	
void RF24_csn(bool mode)
{
	#warning "TODO"

    if (mode == HIGH) {
        PORTB |= (1<<PINB2);  	// SCK->CSN HIGH
        RF24_delayMicroseconds(100); // allow csn to settle.
    }
    else {
        PORTB &= ~(1<<PINB2);	// SCK->CSN LOW
        RF24_delayMicroseconds(11);  // allow csn to settle
    }
    
    // Return, CSN toggle complete
    return;
}

/****************************************************************************/

void RF24_ce(bool level)
{
	#warning "TODO"
    //Allow for 3-pin use on ATTiny
    if (ce_pin != csn_pin) {
        digitalWrite(ce_pin, level);
    }
}

/****************************************************************************/

inline void RF24_beginTransaction()
{
    #if defined(RF24_SPI_TRANSACTIONS)
    SPI_beginTransaction(SPISettings(RF24_SPI_SPEED, MSBFIRST, SPI_MODE0));
    #endif // defined(RF24_SPI_TRANSACTIONS)
    RF24_csn(LOW);
}

/****************************************************************************/

inline void RF24_endTransaction(void)
{
    RF24_csn(HIGH);
    #if defined(RF24_SPI_TRANSACTIONS)
    SPI_endTransaction();
    #endif // defined(RF24_SPI_TRANSACTIONS)
}

/****************************************************************************/

unsigned char RF24_read_register(unsigned char regis, unsigned char* buf, unsigned char len)
{
    unsigned char status;

    RF24_beginTransaction();
    status = SPI_transfer(R_REGISTER | (REGISTER_MASK & regis));
    while (len--) {
        *buf++ = SPI_transfer(0xff);
    }
    RF24_endTransaction();

    return status;
}

/****************************************************************************/
unsigned char RF24_read_register1(unsigned char regis)
{
    unsigned char result;

    #if defined(RF24_LINUX)
    RF24_beginTransaction();

    unsigned char * prx = spi_rxbuff;
    unsigned char * ptx = spi_txbuff;
    *ptx++ = ( R_REGISTER | ( REGISTER_MASK & regis ) );
    *ptx++ = RF24_NOP ; // Dummy operation, just for reading

    SPI_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);
    result = *++prx;   // result is 2nd byte of receive buffer

    RF24_endTransaction();
    #else // !defined(RF24_LINUX)

    RF24_beginTransaction();
    SPI_transfer(R_REGISTER | (REGISTER_MASK & regis));
    result = SPI_transfer(0xff);
    RF24_endTransaction();

    #endif // !defined(RF24_LINUX)

    return result;
}

/****************************************************************************/

unsigned char RF24_write_register(unsigned char regis, const unsigned char* buf, unsigned char len)
{
    unsigned char status;

    #if defined(RF24_LINUX)
    RF24_beginTransaction();
    unsigned char * prx = spi_rxbuff;
    unsigned char * ptx = spi_txbuff;
    unsigned char size = len + 1; // Add register value to transmit buffer

    *ptx++ = ( W_REGISTER | ( REGISTER_MASK & regis ) );
    while ( len-- )
      *ptx++ = *buf++;

    SPI_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);
    status = *prx; // status is 1st byte of receive buffer
    RF24_endTransaction();
    #else // !defined(RF24_LINUX)

    RF24_beginTransaction();
    status = SPI_transfer(W_REGISTER | (REGISTER_MASK & regis));
    while (len--) {
        SPI_transfer(*buf++);
    }
    RF24_endTransaction();
    #endif // !defined(RF24_LINUX)

    return status;
}

/****************************************************************************/

unsigned char RF24_write_register1(unsigned char regis, unsigned char value)
{
    unsigned char status;

    IF_SERIAL_DEBUG(printf_P(PSTR("RF24_write_register(%02x,%02x)\r\n"), regis, value));

    #if defined(RF24_LINUX)
    RF24_beginTransaction();
    unsigned char * prx = spi_rxbuff;
    unsigned char * ptx = spi_txbuff;
    *ptx++ = ( W_REGISTER | ( REGISTER_MASK & regis ) );
    *ptx = value ;

    SPI_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);
    status = *prx++; // status is 1st byte of receive buffer
    RF24_endTransaction();
    #else // !defined(RF24_LINUX)

    RF24_beginTransaction();
    status = SPI_transfer(W_REGISTER | (REGISTER_MASK & regis));
    SPI_transfer(value);
    RF24_endTransaction();

    #endif // !defined(RF24_LINUX)

    return status;
}

/****************************************************************************/

unsigned char RF24_write_payload(const void* buf, unsigned char data_len, const unsigned char writeType)
{
    unsigned char status;
    const unsigned char* current = buf;

    data_len = rf24_min(data_len, payload_size);
    unsigned char blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    //printf("[Writing %u bytes %u blanks]",data_len,blank_len);
    IF_SERIAL_DEBUG(printf("[Writing %u bytes %u blanks]\n", data_len, blank_len); );

    #if defined(RF24_LINUX)
    RF24_beginTransaction();
    unsigned char * prx = spi_rxbuff;
    unsigned char * ptx = spi_txbuff;
    unsigned char size;
    size = data_len + blank_len + 1 ; // Add register value to transmit buffer

    *ptx++ =  writeType;
    while ( data_len-- )
      *ptx++ =  *current++;
    while ( blank_len-- )
      *ptx++ =  0;

    SPI_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);
    status = *prx; // status is 1st byte of receive buffer
    RF24_endTransaction();

    #else // !defined(RF24_LINUX)

    RF24_beginTransaction();
    status = SPI_transfer(writeType);
    while (data_len--) {
        SPI_transfer(*current++);
    }
    while (blank_len--) {
        SPI_transfer(0);
    }
    RF24_endTransaction();

    #endif // !defined(RF24_LINUX)

    return status;
}

/****************************************************************************/

unsigned char RF24_read_payload(void* buf, unsigned char data_len)
{
    unsigned char status;
    unsigned char* current = buf;

    if (data_len > payload_size) {
        data_len = payload_size;
    }
    unsigned char blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    //printf("[Reading %u bytes %u blanks]",data_len,blank_len);

    IF_SERIAL_DEBUG(printf("[Reading %u bytes %u blanks]\n", data_len, blank_len); );

    #if defined(RF24_LINUX)
    RF24_beginTransaction();
    unsigned char * prx = spi_rxbuff;
    unsigned char * ptx = spi_txbuff;
    unsigned char size;
    size = data_len + blank_len + 1; // Add register value to transmit buffer

    *ptx++ =  R_RX_PAYLOAD;
    while(--size)
        *ptx++ = RF24_NOP;

    size = data_len + blank_len + 1; // Size has been lost during while, re affect

    SPI_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);

    status = *prx++; // 1st byte is status

    if (data_len > 0) {
      while ( --data_len ) // Decrement before to skip 1st status byte
          *current++ = *prx++;

      *current = *prx;
    }
    RF24_endTransaction();
    #else // !defined(RF24_LINUX)

    RF24_beginTransaction();
    status = SPI_transfer(R_RX_PAYLOAD);
    while (data_len--) {
        *current++ = SPI_transfer(0xFF);
    }
    while (blank_len--) {
        SPI_transfer(0xff);
    }
    RF24_endTransaction();

    #endif // !defined(RF24_LINUX)

    return status;
}

/****************************************************************************/

unsigned char RF24_flush_rx(void)
{
    return RF24_spiTrans(FLUSH_RX);
}

/****************************************************************************/

unsigned char RF24_flush_tx(void)
{
    return RF24_spiTrans(FLUSH_TX);
}

/****************************************************************************/

unsigned char RF24_spiTrans(unsigned char cmd)
{

    unsigned char status;

    RF24_beginTransaction();
    status = SPI_transfer(cmd);
    RF24_endTransaction();

    return status;
}

/****************************************************************************/

unsigned char RF24_get_status(void)
{
    return RF24_spiTrans(RF24_NOP);
}

/****************************************************************************/
#if !defined(MINIMAL)

void RF24_print_status(unsigned char status)
{
    printf_P(PSTR("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n"), status, (status & _BV(RX_DR)) ? 1 : 0,
            (status & _BV(TX_DS)) ? 1 : 0, (status & _BV(MAX_RT)) ? 1 : 0, ((status >> RX_P_NO) & 0x07), (status & _BV(TX_FULL)) ? 1 : 0);
}

/****************************************************************************/

void RF24_print_observe_tx(unsigned char value)
{
    printf_P(PSTR("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n"), value, (value >> PLOS_CNT) & 0x0F, (value >> ARC_CNT) & 0x0F);
}

/****************************************************************************/

void RF24_print_byte_register(const char* name, unsigned char reg, unsigned char qty)
{
    //char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
    //printf_P(PSTR(PRIPSTR"\t%c ="),name,extra_tab);
    #if defined(RF24_LINUX)
    printf("%s\t =", name);
    #else // !defined(RF24_LINUX)
    printf_P(PSTR(PRIPSTR
    "\t ="),name);
    #endif // !defined(RF24_LINUX)
    while (qty--) {
        printf_P(PSTR(" 0x%02x"), RF24_read_register1(reg++));
    }
    printf_P(PSTR("\r\n"));
}

/****************************************************************************/

void RF24_print_address_register(const char* name, unsigned char reg, unsigned char qty)
{

    #if defined(RF24_LINUX)
    printf("%s\t =",name);
    #else // !defined(RF24_LINUX)
    printf_P(PSTR(PRIPSTR"\t ="),name);
    #endif // !defined(RF24_LINUX)
    while (qty--) {
        unsigned char buffer[addr_width];
        RF24_read_register(reg++, buffer, sizeof buffer);

        printf_P(PSTR(" 0x"));
        unsigned char* bufptr = buffer + sizeof buffer;
        while (--bufptr >= buffer) {
            printf_P(PSTR("%02x"), *bufptr);
        }
    }

    printf_P(PSTR("\r\n"));
}

#endif

/****************************************************************************/

void RF24_RF24(unsigned short _cepin, unsigned short _cspin)
{
	ce_pin = _cepin;
	csn_pin = _cspin;
	p_variant = false;
	payload_size = 32;
	dynamic_payloads_enabled = false;
	addr_width = 5;
	csDelay = 5;
	//,pipe0_reading_address(0)
    pipe0_reading_address[0] = 0;
}

/****************************************************************************/

#if defined(RF24_LINUX) && !defined(MRAA)//RPi constructor

RF24_RF24(unsigned short _cepin, unsigned short _cspin, uint32_t _spi_speed)
{
	ce_pin = _cepin;
	csn_pin = _cspin;
	spi_speed = _spi_speed;
	p_variant = false;
	payload_size = 32;
	dynamic_payloads_enabled = false;
	addr_width = 5;
	//pipe0_reading_address = 0; 
  pipe0_reading_address[0]=0;
}
#endif

/****************************************************************************/

void RF24_setChannel(unsigned char channel)
{
    const unsigned char max_channel = 125;
    RF24_write_register1(RF_CH, rf24_min(channel, max_channel));
}

unsigned char RF24_getChannel()
{

    return RF24_read_register1(RF_CH);
}

/****************************************************************************/

void RF24_setPayloadSize(unsigned char size)
{
    payload_size = rf24_min(size, 32);
}

/****************************************************************************/

unsigned char RF24_getPayloadSize(void)
{
    return payload_size;
}

/****************************************************************************/

#if !defined(MINIMAL)

static const char rf24_datarate_e_str_0[] PROGMEM = "1MBPS";
static const char rf24_datarate_e_str_1[] PROGMEM = "2MBPS";
static const char rf24_datarate_e_str_2[] PROGMEM = "250KBPS";
static const char * const rf24_datarate_e_str_P[] PROGMEM = {
  rf24_datarate_e_str_0,
  rf24_datarate_e_str_1,
  rf24_datarate_e_str_2,
};
static const char rf24_model_e_str_0[] PROGMEM = "nRF24L01";
static const char rf24_model_e_str_1[] PROGMEM = "nRF24L01+";
static const char * const rf24_model_e_str_P[] PROGMEM = {
  rf24_model_e_str_0,
  rf24_model_e_str_1,
};
static const char rf24_crclength_e_str_0[] PROGMEM = "Disabled";
static const char rf24_crclength_e_str_1[] PROGMEM = "8 bits";
static const char rf24_crclength_e_str_2[] PROGMEM = "16 bits" ;
static const char * const rf24_crclength_e_str_P[] PROGMEM = {
  rf24_crclength_e_str_0,
  rf24_crclength_e_str_1,
  rf24_crclength_e_str_2,
};
static const char rf24_pa_dbm_e_str_0[] PROGMEM = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[] PROGMEM = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[] PROGMEM = "PA_HIGH";
static const char rf24_pa_dbm_e_str_3[] PROGMEM = "PA_MAX";
static const char * const rf24_pa_dbm_e_str_P[] PROGMEM = {
  rf24_pa_dbm_e_str_0,
  rf24_pa_dbm_e_str_1,
  rf24_pa_dbm_e_str_2,
  rf24_pa_dbm_e_str_3,
};

    #if defined(RF24_LINUX)
static const char rf24_csn_e_str_0[] = "CE0 (PI Hardware Driven)";
static const char rf24_csn_e_str_1[] = "CE1 (PI Hardware Driven)";
static const char rf24_csn_e_str_2[] = "CE2 (PI Hardware Driven)";
static const char rf24_csn_e_str_3[] = "Custom GPIO Software Driven";
static const char * const rf24_csn_e_str_P[] = {
  rf24_csn_e_str_0,
  rf24_csn_e_str_1,
  rf24_csn_e_str_2,
  rf24_csn_e_str_3,
};
    #endif // defined(RF24_LINUX)
#if 0
void RF24_printDetails(void)
{

    #if defined(RF24_RPi)
    printf("================ SPI Configuration ================\n" );
    if (csn_pin < BCM2835_SPI_CS_NONE ){
      printf("CSN Pin  \t = %s\n",rf24_csn_e_str_P[csn_pin]);
    }else{
      printf("CSN Pin  \t = Custom GPIO%d%s\n", csn_pin,
      csn_pin==RPI_V2_GPIO_P1_26 ? " (CE1) Software Driven" : "" );
    }
    printf("CE Pin  \t = Custom GPIO%d\n", ce_pin );
    printf("Clock Speed\t = " );
      switch (spi_speed)
      {
          case BCM2835_SPI_SPEED_64MHZ : printf("64 Mhz");	break ;
          case BCM2835_SPI_SPEED_32MHZ : printf("32 Mhz");	break ;
          case BCM2835_SPI_SPEED_16MHZ : printf("16 Mhz");	break ;
          case BCM2835_SPI_SPEED_8MHZ  : printf("8 Mhz");	break ;
          case BCM2835_SPI_SPEED_4MHZ  : printf("4 Mhz");	break ;
          case BCM2835_SPI_SPEED_2MHZ  : printf("2 Mhz");	break ;
          case BCM2835_SPI_SPEED_1MHZ  : printf("1 Mhz");	break ;
          case BCM2835_SPI_SPEED_512KHZ: printf("512 KHz");	break ;
          case BCM2835_SPI_SPEED_256KHZ: printf("256 KHz");	break ;
          case BCM2835_SPI_SPEED_128KHZ: printf("128 KHz");	break ;
          case BCM2835_SPI_SPEED_64KHZ : printf("64 KHz");	break ;
          case BCM2835_SPI_SPEED_32KHZ : printf("32 KHz");	break ;
          case BCM2835_SPI_SPEED_16KHZ : printf("16 KHz");	break ;
          case BCM2835_SPI_SPEED_8KHZ  : printf("8 KHz");	break ;
          default : printf("8 Mhz");	break ;
      }
      printf("\n================ NRF Configuration ================\n");

    #endif // defined(RF24_RPi)

    print_status(RF24_get_status());

    print_address_register(PSTR("RX_ADDR_P0-1"), RX_ADDR_P0, 2);
    print_byte_register(PSTR("RX_ADDR_P2-5"), RX_ADDR_P2, 4);
    print_address_register(PSTR("TX_ADDR\t"), TX_ADDR);

    print_byte_register(PSTR("RX_PW_P0-6"), RX_PW_P0, 6);
    print_byte_register(PSTR("EN_AA\t"), EN_AA);
    print_byte_register(PSTR("EN_RXADDR"), EN_RXADDR);
    print_byte_register(PSTR("RF_CH\t"), RF_CH);
    print_byte_register(PSTR("RF_SETUP"), RF_SETUP);
    print_byte_register(PSTR("CONFIG\t"), NRF_CONFIG);
    print_byte_register(PSTR("DYNPD/FEATURE"), DYNPD, 2);

    printf_P(PSTR("Data Rate\t = "
    PRIPSTR
    "\r\n"),pgm_read_ptr(&rf24_datarate_e_str_P[getDataRate()]));
    printf_P(PSTR("Model\t\t = "
    PRIPSTR
    "\r\n"),pgm_read_ptr(&rf24_model_e_str_P[isPVariant()]));
    printf_P(PSTR("CRC Length\t = "
    PRIPSTR
    "\r\n"),pgm_read_ptr(&rf24_crclength_e_str_P[getCRCLength()]));
    printf_P(PSTR("PA Power\t = "
    PRIPSTR
    "\r\n"),  pgm_read_ptr(&rf24_pa_dbm_e_str_P[getPALevel()]));

}
#endif

#endif // !defined(MINIMAL)

/****************************************************************************/

bool RF24_begin(void)
{

    unsigned char setup = 0;

    #if defined(RF24_LINUX)

        #if defined(MRAA)
    GPIO();
    gpio.begin(ce_pin,csn_pin);
        #endif

        #if defined(RF24_RPi)
    switch(csn_pin){     //Ensure valid hardware CS pin
      case 0: break;
      case 1: break;
      // Allow BCM2835 enums for RPi
      case 8: csn_pin = 0; break;
      case 7: csn_pin = 1; break;
      case 18: csn_pin = 10; break;	//to make it work on SPI1
      case 17: csn_pin = 11; break;
      case 16: csn_pin = 12; break;
      default: csn_pin = 0; break;
    }
        #endif // RF24_RPi

    SPI_begin(csn_pin);

    pinMode(ce_pin,OUTPUT);
    RF24_ce(LOW);

    _delay_ms (100);

    #elif defined(LITTLEWIRE)

    pinMode(csn_pin,OUTPUT);
        SPI_begin();
        csn(HIGH);

    #elif defined(XMEGA_D3)
    if (ce_pin != csn_pin) {
        pinMode(ce_pin,OUTPUT);
    };
    SPI_begin(csn_pin);
    RF24_ce(LOW);
    RF24_csn(HIGH);
    delay(200);
    #else
    // Initialize pins
    if (ce_pin != csn_pin) {
        pinMode(ce_pin, OUTPUT);
    }

        #if !defined(LITTLEWIRE)
    if (ce_pin != csn_pin)
        #endif // !defined(LITTLEWIRE)
    {
        pinMode(csn_pin, OUTPUT);
    }

    //SPI_begin();
    RF24_ce(LOW);
    RF24_csn(HIGH);
    #if defined(__ARDUINO_X86__)
    _delay_ms(100);
    #endif
	#endif //Linux

    // Must allow the radio time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too. For full coverage, we'll always assume the worst.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    _delay_ms(5);

    // Reset NRF_CONFIG and enable 16-bit CRC.
    RF24_write_register1(NRF_CONFIG, 0x0C);

    // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
    // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
    // sizes must never be used. See documentation for a more complete explanation.
    RF24_setRetries(5, 15);

    // Reset value is MAX
    //setPALevel( RF24_PA_MAX ) ;

    // check for connected module and if this is a p nRF24l01 variant
    //
    if (RF24_setDataRate(RF24_250KBPS)) {
        p_variant = true;
    }
    setup = RF24_read_register1(RF_SETUP);
    /*if( setup == 0b00001110 )     // register default for nRF24L01P
    {
      p_variant = true ;
    }*/

    // Then set the data rate to the slowest (and most reliable) speed supported by all
    // hardware.
    RF24_setDataRate(RF24_1MBPS);

    // Initialize CRC and request 2-byte (16bit) CRC
    //setCRCLength( RF24_CRC_16 ) ;

    // Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
    RF24_toggle_features();
    RF24_write_register1(FEATURE, 0);
    RF24_write_register1(DYNPD, 0);
    dynamic_payloads_enabled = false;

    // Reset current status
    // Notice reset and flush is the last thing we do
    RF24_write_register1(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    // Set up default configuration.  Callers can always change it later.
    // This channel should be universally safe and not bleed over into adjacent
    // spectrum.
    RF24_setChannel(76);

    // Flush buffers
    RF24_flush_rx();
    RF24_flush_tx();

    RF24_powerUp(); //Power up by default when begin() is called

    // Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from powerUp )
    // PTX should use only 22uA of power
    RF24_write_register1(NRF_CONFIG, (RF24_read_register1(NRF_CONFIG)) & ~_BV(PRIM_RX));

    // if setup is 0 or ff then there was no response from module
    return (setup != 0 && setup != 0xff);
}

/****************************************************************************/

bool RF24_isChipConnected()
{
    unsigned char setup = RF24_read_register1(SETUP_AW);
    if (setup >= 1 && setup <= 3) {
        return true;
    }

    return false;
}

/****************************************************************************/

void RF24_startListening(void)
{
    #if !defined(RF24_TINY) && !defined(LITTLEWIRE)
    RF24_powerUp();
    #endif
    RF24_write_register1(NRF_CONFIG, RF24_read_register1(NRF_CONFIG) | _BV(PRIM_RX));
    RF24_write_register1(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
    RF24_ce(HIGH);
    // Restore the pipe0 adddress, if exists
    if (pipe0_reading_address[0] > 0) {
        RF24_write_register(RX_ADDR_P0, pipe0_reading_address, addr_width);
    } else {
        RF24_closeReadingPipe(0);
    }

    // Flush buffers
    //flush_rx();
    if (RF24_read_register1(FEATURE) & _BV(EN_ACK_PAY)) {
        RF24_flush_tx();
    }

    // Go!
    //delayMicroseconds(100);
}

/****************************************************************************/
static const unsigned char child_pipe_enable[] = {ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5};

void RF24_stopListening(void)
{
    RF24_ce(LOW);

    RF24_delayMicroseconds(txDelay);

    if (RF24_read_register1(FEATURE) & _BV(EN_ACK_PAY)) {
        RF24_delayMicroseconds(txDelay); //200
        RF24_flush_tx();
    }
    //flush_rx();
    RF24_write_register1(NRF_CONFIG, (RF24_read_register1(NRF_CONFIG)) & ~_BV(PRIM_RX));

    #if defined(RF24_TINY) || defined(LITTLEWIRE)
    // for 3 pins solution TX mode is only left with additonal powerDown/powerUp cycle
    if (ce_pin == csn_pin) {
      RF24_powerDown();
      RF24_powerUp();
    }
    #endif
    RF24_write_register1(EN_RXADDR, RF24_read_register1(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[0]))); // Enable RX on pipe0

    //delayMicroseconds(100);

}

/****************************************************************************/

void RF24_powerDown(void)
{
    RF24_ce(LOW); // Guarantee CE is low on powerDown
    RF24_write_register1(NRF_CONFIG, RF24_read_register1(NRF_CONFIG) & ~_BV(PWR_UP));
}

/****************************************************************************/

//Power up now. Radio will not power down unless instructed by MCU for config changes etc.
void RF24_powerUp(void)
{
    unsigned char cfg = RF24_read_register1(NRF_CONFIG);

    // if not powered up then power up and wait for the radio to initialize
    if (!(cfg & _BV(PWR_UP))) {
        RF24_write_register1(NRF_CONFIG, cfg | _BV(PWR_UP));

        // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
        // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
        // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
        _delay_ms (5);
    }
}

/******************************************************************/
#if defined(FAILURE_HANDLING) || defined(RF24_LINUX)

void RF24_errNotify()
{
    #if defined(SERIAL_DEBUG) || defined(RF24_LINUX)
    printf_P(PSTR("RF24 HARDWARE FAIL: Radio not responding, verify pin connections, wiring, etc.\r\n"));
    #endif
    #if defined(FAILURE_HANDLING)
    RF24_failureDetected = 1;
    #else
    _delay_ms (5000);
    #endif
}

#endif
/******************************************************************/

//Similar to the previous write, clears the interrupt flags
bool RF24_write_multi(const void* buf, unsigned char len, const bool multicast)
{
    //Start Writing
    RF24_startFastWrite(buf, len, multicast, true);

    //Wait until complete or failed
    #if defined(FAILURE_HANDLING) || defined(RF24_LINUX)
    uint32_t timer = millis();
    #endif // defined(FAILURE_HANDLING) || defined(RF24_LINUX)

    while (!(RF24_get_status() & (_BV(TX_DS) | _BV(MAX_RT)))) {
        #if defined(FAILURE_HANDLING) || defined(RF24_LINUX)
        if (millis() - timer > 95) {
            errNotify();
            #if defined(FAILURE_HANDLING)
            return 0;
            #else
            _delay_ms(100);
            #endif
        }
        #endif
    }

    RF24_ce(LOW);

    unsigned char status = RF24_write_register1(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    //Max retries exceeded
    if (status & _BV(MAX_RT)) {
        RF24_flush_tx(); //Only going to be 1 packet int the FIFO at a time using this method, so just flush
        return 0;
    }
    //TX OK 1 or 0
    return 1;
}

bool RF24_write(const void* buf, unsigned char len)
{
    return RF24_write_multi(buf, len, 0);
}
/****************************************************************************/

//For general use, the interrupt flags are not important to clear
bool RF24_writeBlocking(const void* buf, unsigned char len, uint32_t timeout)
{
    //Block until the FIFO is NOT full.
    //Keep track of the MAX retries and set auto-retry if seeing failures
    //This way the FIFO will fill up and allow blocking until packets go through
    //The radio will auto-clear everything in the FIFO as long as CE remains high

    uint32_t timer = millis();                              //Get the time that the payload transmission started

    while ((RF24_get_status()
            & (_BV(TX_FULL)))) {          //Blocking only if FIFO is full. This will loop and block until TX is successful or timeout

        if (RF24_get_status() & _BV(MAX_RT)) {                      //If MAX Retries have been reached
            RF24_reUseTX();                                          //Set re-transmit and clear the MAX_RT interrupt flag
            if (millis() - timer > timeout) {
                return 0;
            }          //If this payload has exceeded the user-defined timeout, exit and return 0
        }
        #if defined(FAILURE_HANDLING) || defined(RF24_LINUX)
        if (millis() - timer > (timeout + 95)) {
            errNotify();
            #if defined(FAILURE_HANDLING)
            return 0;
            #endif
        }
        #endif

    }

    //Start Writing
    RF24_startFastWrite(buf, len, 0, false);                                  //Write the payload if a buffer is clear

    return 1;                                                  //Return 1 to indicate successful transmission
}

/****************************************************************************/

void RF24_reUseTX()
{
    RF24_write_register1(NRF_STATUS, _BV(MAX_RT));              //Clear max retry flag
    RF24_spiTrans(REUSE_TX_PL);
    RF24_ce(LOW);                                          //Re-Transfer packet
    RF24_ce(HIGH);
}

/****************************************************************************/

bool RF24_writeFast_multi(const void* buf, unsigned char len, const bool multicast)
{
    //Block until the FIFO is NOT full.
    //Keep track of the MAX retries and set auto-retry if seeing failures
    //Return 0 so the user can control the retrys and set a timer or failure counter if required
    //The radio will auto-clear everything in the FIFO as long as CE remains high

    #if defined(FAILURE_HANDLING) || defined(RF24_LINUX)
    uint32_t timer = millis();
    #endif

    //Blocking only if FIFO is full. This will loop and block until TX is successful or fail
    while ((RF24_get_status() & (_BV(TX_FULL)))) {
        if (RF24_get_status() & _BV(MAX_RT)) {
            //reUseTX();                                 //Set re-transmit
            RF24_write_register1(NRF_STATUS, _BV(MAX_RT));     //Clear max retry flag
            return 0;                                    //Return 0. The previous payload has been retransmitted
            // From the user perspective, if you get a 0, just keep trying to send the same payload
        }
        #if defined(FAILURE_HANDLING) || defined(RF24_LINUX)
        if (millis() - timer > 95) {
            errNotify();
            #if defined(FAILURE_HANDLING)
            return 0;
            #endif // defined(FAILURE_HANDLING)
        }
        #endif
    }
    //Start Writing
    RF24_startFastWrite(buf, len, multicast, false);

    return 1;
}

bool RF24_writeFast(const void* buf, unsigned char len)
{
    return RF24_writeFast_multi(buf, len, 0);
}

/****************************************************************************/

//Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
//In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
//Otherwise we enter Standby-II mode, which is still faster than standby mode
//Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data

void RF24_startFastWrite(const void* buf, unsigned char len, const bool multicast, bool startTx)
{ //TMRh20

    //write_payload( buf,len);
    RF24_write_payload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
    if (startTx) {
        RF24_ce(HIGH);
    }

}

/****************************************************************************/

//Added the original startWrite back in so users can still use interrupts, ack payloads, etc
//Allows the library to pass all tests
void RF24_startWrite(const void* buf, unsigned char len, const bool multicast)
{

    // Send the payload

    //write_payload( buf, len );
    RF24_write_payload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
    RF24_ce(HIGH);
    #if !defined(F_CPU) || F_CPU > 20000000
    RF24_delayMicroseconds(10);
    #endif
    RF24_ce(LOW);
}

/****************************************************************************/

bool RF24_rxFifoFull()
{
    return RF24_read_register1(FIFO_STATUS) & _BV(RX_FULL);
}

/****************************************************************************/

bool RF24_txStandBy()
{

    #if defined(FAILURE_HANDLING) || defined(RF24_LINUX)
    uint32_t timeout = millis();
    #endif
    while (!(RF24_read_register1(FIFO_STATUS) & _BV(TX_EMPTY))) {
        if (RF24_get_status() & _BV(MAX_RT)) {
            RF24_write_register1(NRF_STATUS, _BV(MAX_RT));
            RF24_ce(LOW);
            RF24_flush_tx();    //Non blocking, flush the data
            return 0;
        }
        #if defined(FAILURE_HANDLING) || defined(RF24_LINUX)
        if (millis() - timeout > 95) {
            errNotify();
            #if defined(FAILURE_HANDLING)
            return 0;
            #endif
        }
        #endif
    }

    RF24_ce(LOW);               //Set STANDBY-I mode
    return 1;
}

/****************************************************************************/

bool RF24_txStandBy_TO(uint32_t timeout, bool startTx)
{

    if (startTx) {
        RF24_stopListening();
        RF24_ce(HIGH);
    }
    uint32_t start = millis();

    while (!(RF24_read_register1(FIFO_STATUS) & _BV(TX_EMPTY))) {
        if (RF24_get_status() & _BV(MAX_RT)) {
            RF24_write_register1(NRF_STATUS, _BV(MAX_RT));
            RF24_ce(LOW); // Set re-transmit
            RF24_ce(HIGH);
            if (millis() - start >= timeout) {
                RF24_ce(LOW);
                RF24_flush_tx();
                return 0;
            }
        }
        #if defined(FAILURE_HANDLING) || defined(RF24_LINUX)
        if (millis() - start > (timeout + 95)) {
            errNotify();
            #if defined(FAILURE_HANDLING)
            return 0;
            #endif
        }
        #endif
    }

    RF24_ce(LOW);  //Set STANDBY-I mode
    return 1;

}

/****************************************************************************/

void RF24_maskIRQ(bool tx, bool fail, bool rx)
{

    unsigned char config = RF24_read_register1(NRF_CONFIG);
    /* clear the interrupt flags */
    config &= ~(1 << MASK_MAX_RT | 1 << MASK_TX_DS | 1 << MASK_RX_DR);
    /* set the specified interrupt flags */
    config |= fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR;
    RF24_write_register1(NRF_CONFIG, config);
}

/****************************************************************************/

unsigned char RF24_getDynamicPayloadSize(void)
{
    unsigned char result = 0;

    #if defined(RF24_LINUX)
    spi_txbuff[0] = R_RX_PL_WID;
    spi_txbuff[1] = 0xff;
    RF24_beginTransaction();
    SPI_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);
    result = spi_rxbuff[1];
    RF24_endTransaction();
    #else
    RF24_beginTransaction();
    SPI_transfer(R_RX_PL_WID);
    result = SPI_transfer(0xff);
    RF24_endTransaction();
    #endif

    if (result > 32) {
        RF24_flush_rx();
        _delay_ms (2);
        return 0;
    }
    return result;
}

/****************************************************************************/

bool RF24_available(void)
{
    return RF24_available_pipe(NULL);
}

/****************************************************************************/

bool RF24_available_pipe(unsigned char* pipe_num)
{
    if (!(RF24_read_register1(FIFO_STATUS) & _BV(RX_EMPTY))) {

        // If the caller wants the pipe number, include that
        if (pipe_num) {
            unsigned char status = RF24_get_status();
            *pipe_num = (status >> RX_P_NO) & 0x07;
        }
        return 1;
    }

    return 0;


}

/****************************************************************************/

void RF24_read(void* buf, unsigned char len)
{

    // Fetch the payload
    RF24_read_payload(buf, len);

    //Clear the two possible interrupt flags with one command
    RF24_write_register1(NRF_STATUS, _BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS));

}

/****************************************************************************/

void RF24_whatHappened(bool* tx_ok, bool* tx_fail, bool* rx_ready)
{
    // Read the status & reset the status in one easy call
    // Or is that such a good idea?
    unsigned char status = RF24_write_register1(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    // Report to the user what happened
    tx_ok = status & _BV(TX_DS);
    tx_fail = status & _BV(MAX_RT);
    rx_ready = status & _BV(RX_DR);
}

/****************************************************************************/
#if 0
void RF24_openWritingPipe(const unsigned char* address value)
{
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.

    RF24_write_register(RX_ADDR_P0, (unsigned char*)value, addr_width);
    RF24_write_register(TX_ADDR, (unsigned char*)value, addr_width);


    //const unsigned char max_payload_size = 32;
    //RF24_write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
    RF24_write_register(RX_PW_P0, payload_size);
}
#endif
/****************************************************************************/
void RF24_openWritingPipe(const unsigned char* address)
{
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.
    RF24_write_register(RX_ADDR_P0, address, addr_width);
    RF24_write_register(TX_ADDR, address, addr_width);

    //const unsigned char max_payload_size = 32;
    //RF24_write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
    RF24_write_register1(RX_PW_P0, payload_size);
}

/****************************************************************************/
static const unsigned char child_pipe[] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};
static const unsigned char child_payload_size[] = {RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5};

#if 0
void RF24_openReadingPipe(unsigned char child, uint64_t address)
{
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0) {
        memcpy(pipe0_reading_address, &address, addr_width);
    }

    if (child <= 6) {
        // For pipes 2-5, only write the LSB
        if (child < 2) {
            RF24_write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const unsigned char*>(&address), addr_width);
        } else {
            RF24_write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const unsigned char*>(&address), 1);
        }

        RF24_write_register(pgm_read_byte(&child_payload_size[child]), payload_size);

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        RF24_write_register(EN_RXADDR, RF24_read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
    }
}
#endif
/****************************************************************************/
void RF24_setAddressWidth(unsigned char a_width)
{

    if (a_width -= 2) {
        RF24_write_register1(SETUP_AW, a_width % 4);
        addr_width = (a_width % 4) + 2;
    } else {
        RF24_write_register1(SETUP_AW, 0);
        addr_width = 2;
    }

}

/****************************************************************************/

void RF24_openReadingPipe(unsigned char child, const unsigned char* address)
{
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0) {
        memcpy(pipe0_reading_address, address, addr_width);
    }
    if (child <= 6) {
        // For pipes 2-5, only write the LSB
        if (child < 2) {
            RF24_write_register(pgm_read_byte(&child_pipe[child]), address, addr_width);
        } else {
            RF24_write_register(pgm_read_byte(&child_pipe[child]), address, 1);
        }
        RF24_write_register1(pgm_read_byte(&child_payload_size[child]), payload_size);

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        RF24_write_register1(EN_RXADDR, RF24_read_register1(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));

    }
}

/****************************************************************************/

void RF24_closeReadingPipe(unsigned char pipe)
{
    RF24_write_register1(EN_RXADDR, RF24_read_register1(EN_RXADDR) & ~_BV(pgm_read_byte(&child_pipe_enable[pipe])));
}

/****************************************************************************/

void RF24_toggle_features(void)
{
    RF24_beginTransaction();
    SPI_transfer(ACTIVATE);
    SPI_transfer(0x73);
    RF24_endTransaction();
}

/****************************************************************************/

void RF24_enableDynamicPayloads(void)
{
    // Enable dynamic payload throughout the system

    //toggle_features();
    RF24_write_register1(FEATURE, RF24_read_register1(FEATURE) | _BV(EN_DPL));

    IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n", RF24_read_register1(FEATURE)));

    // Enable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    RF24_write_register1(DYNPD, RF24_read_register1(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

    dynamic_payloads_enabled = true;
}

/****************************************************************************/
void RF24_disableDynamicPayloads(void)
{
    // Disables dynamic payload throughout the system.  Also disables Ack Payloads

    //toggle_features();
    RF24_write_register1(FEATURE, 0);

    IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n", RF24_read_register(FEATURE)));

    // Disable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    RF24_write_register1(DYNPD, 0);

    dynamic_payloads_enabled = false;
}

/****************************************************************************/

void RF24_enableAckPayload(void)
{
    //
    // enable ack payload and dynamic payload features
    //

    //toggle_features();
    RF24_write_register1(FEATURE, RF24_read_register1(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL));

    IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n", RF24_read_register(FEATURE)));

    //
    // Enable dynamic payload on pipes 0 & 1
    //
    RF24_write_register1(DYNPD, RF24_read_register1(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
    dynamic_payloads_enabled = true;
}

/****************************************************************************/

void RF24_enableDynamicAck(void)
{
    //
    // enable dynamic ack features
    //
    //toggle_features();
    RF24_write_register1(FEATURE, RF24_read_register1(FEATURE) | _BV(EN_DYN_ACK));

    IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n", RF24_read_register(FEATURE)));


}

/****************************************************************************/

void RF24_writeAckPayload(unsigned char pipe, const void* buf, unsigned char len)
{
    const unsigned char* current = buf;

    unsigned char data_len = rf24_min(len, 32);

    #if defined(RF24_LINUX)
    RF24_beginTransaction();
    unsigned char * ptx = spi_txbuff;
    unsigned char size = data_len + 1 ; // Add register value to transmit buffer
    *ptx++ =  W_ACK_PAYLOAD | ( pipe & 0x07 );
    while ( data_len-- ){
      *ptx++ =  *current++;
    }

    SPI_transfern( (char *) spi_txbuff, size);
    RF24_endTransaction();
    #else
    RF24_beginTransaction();
    SPI_transfer(W_ACK_PAYLOAD | (pipe & 0x07));

    while (data_len--) {
        SPI_transfer(*current++);
    }
    RF24_endTransaction();

    #endif

}

/****************************************************************************/

bool RF24_isAckPayloadAvailable(void)
{
    return !(RF24_read_register1(FIFO_STATUS) & _BV(RX_EMPTY));
}

/****************************************************************************/

bool RF24_isPVariant(void)
{
    return p_variant;
}

/****************************************************************************/

void RF24_setAutoAck(bool enable)
{
    if (enable) {
        RF24_write_register1(EN_AA, 0x3F);
    } else {
        RF24_write_register1(EN_AA, 0);
    }
}

/****************************************************************************/

void RF24_setAutoAck_pipe(unsigned char pipe, bool enable)
{
    if (pipe <= 6) {
        unsigned char en_aa = RF24_read_register1(EN_AA);
        if (enable) {
            en_aa |= _BV(pipe);
        } else {
            en_aa &= ~_BV(pipe);
        }
        RF24_write_register1(EN_AA, en_aa);
    }
}

/****************************************************************************/

bool RF24_testCarrier(void)
{
    return (RF24_read_register1(CD) & 1);
}

/****************************************************************************/

bool RF24_testRPD(void)
{
    return (RF24_read_register1(RPD) & 1);
}

/****************************************************************************/

void RF24_setPALevel(unsigned char level)
{

    unsigned char setup = RF24_read_register1(RF_SETUP) & 0xF8;

    if (level > 3) {                        // If invalid level, go to max PA
        level = (RF24_PA_MAX << 1) + 1;        // +1 to support the SI24R1 chip extra bit
    } else {
        level = (level << 1) + 1;            // Else set level as requested
    }

    RF24_write_register1(RF_SETUP, setup |= level);    // Write it to the chip
}

/****************************************************************************/

unsigned char RF24_getPALevel(void)
{

    return (RF24_read_register1(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1;
}

/****************************************************************************/

unsigned char RF24_getARC(void)
{

    return RF24_read_register1(OBSERVE_TX) & 0x0F;
}

/****************************************************************************/

bool RF24_setDataRate(rf24_datarate_e speed)
{
    bool result = false;
    unsigned char setup = RF24_read_register1(RF_SETUP);

    // HIGH and LOW '00' is 1Mbs - our default
    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    #if !defined(F_CPU) || F_CPU > 20000000
    txDelay = 250;
    #else //16Mhz Arduino
    txDelay=85;
    #endif
    if (speed == RF24_250KBPS) {
        // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
        // Making it '10'.
        setup |= _BV(RF_DR_LOW);
        #if !defined(F_CPU) || F_CPU > 20000000
        txDelay = 450;
        #else //16Mhz Arduino
        txDelay = 155;
        #endif
    } else {
        // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
        // Making it '01'
        if (speed == RF24_2MBPS) {
            setup |= _BV(RF_DR_HIGH);
            #if !defined(F_CPU) || F_CPU > 20000000
            txDelay = 190;
            #else // 16Mhz Arduino
            txDelay = 65;
            #endif
        }
    }
    RF24_write_register1(RF_SETUP, setup);

    // Verify our result
    if (RF24_read_register1(RF_SETUP) == setup) {
        result = true;
    }
    return result;
}

/****************************************************************************/

rf24_datarate_e RF24_getDataRate(void)
{
    rf24_datarate_e result;
    unsigned char dr = RF24_read_register1(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    // switch uses RAM (evil!)
    // Order matters in our case below
    if (dr == _BV(RF_DR_LOW)) {
        // '10' = 250KBPS
        result = RF24_250KBPS;
    } else if (dr == _BV(RF_DR_HIGH)) {
        // '01' = 2MBPS
        result = RF24_2MBPS;
    } else {
        // '00' = 1MBPS
        result = RF24_1MBPS;
    }
    return result;
}

/****************************************************************************/

void RF24_setCRCLength(rf24_crclength_e length)
{
    unsigned char config = RF24_read_register1(NRF_CONFIG) & ~(_BV(CRCO) | _BV(EN_CRC));

    // switch uses RAM (evil!)
    if (length == RF24_CRC_DISABLED) {
        // Do nothing, we turned it off above.
    } else if (length == RF24_CRC_8) {
        config |= _BV(EN_CRC);
    } else {
        config |= _BV(EN_CRC);
        config |= _BV(CRCO);
    }
    RF24_write_register1(NRF_CONFIG, config);
}

/****************************************************************************/

rf24_crclength_e RF24_getCRCLength(void)
{
    rf24_crclength_e result = RF24_CRC_DISABLED;

    unsigned char config = RF24_read_register1(NRF_CONFIG) & (_BV(CRCO) | _BV(EN_CRC));
    unsigned char AA = RF24_read_register1(EN_AA);

    if (config & _BV(EN_CRC) || AA) {
        if (config & _BV(CRCO)) {
            result = RF24_CRC_16;
        } else {
            result = RF24_CRC_8;
        }
    }

    return result;
}

/****************************************************************************/

void RF24_disableCRC(void)
{
    unsigned char disable = RF24_read_register1(NRF_CONFIG) & ~_BV(EN_CRC);
    RF24_write_register1(NRF_CONFIG, disable);
}

/****************************************************************************/
void RF24_setRetries(unsigned char delay, unsigned char count)
{
    RF24_write_register1(SETUP_RETR, (delay & 0xf) << ARD | (count & 0xf) << ARC);
}

#if 0
void SPI_begin() {
    // set USCK and DO for output
    // set DI for input
        #if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    DDRB |= (1 << PB2) | (1 << PB1);
    DDRB &= ~(1 << PB0);
        #elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    DDRA |= (1 << PA4) | (1 << PA5);
    DDRA &= ~(1 << PA6);
        #elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny4313__)
    DDRB |= (1 << PB7) | (1 << PB6);
    DDRB &= ~(1 << PB5);
        #elif defined(__AVR_ATtiny861__)
    DDRB |= (1 << PB2) | (1 << PB1);
    DDRB &= ~(1 << PB0);
        #endif // defined(__AVR_ATtiny861__)
    USICR = _BV(USIWM0);
}
#endif

unsigned char SPI_transfer(unsigned char b)
{
#if 0
    USIDR = b;
    USISR = _BV(USIOIF);
    do {
        USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
    }
    while ((USISR & _BV(USIOIF)) == 0);
    return USIDR;
#endif
	SPDR = b;
	//  Wait until Tx register empty.
	while ( (SPSR & 0x80) != 0x80 );
	return SPDR;
}

void SPI_end() {}
void SPI_setDataMode(unsigned char mode){}
void SPI_setBitOrder(unsigned char bitOrder){}
void SPI_setClockDivider(unsigned char rate){}
