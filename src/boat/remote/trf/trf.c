#include <atmel_start.h>
#include <util/delay.h>
#include "trf.h"

unsigned char trf_conf[15] = {
0x77, // Frequency channel - RX=1 or TX=0 operation
0x4F, // Enable two channel receive mode - Communication mode ( Direct or ShockBurst) - RF data rate (1Mbps requires 16MHz crystal) - Crystal frequency (Factory default 16MHz crystal mounted) - RF output power
0x43, // Number of address bits(both RX channels) - 8 or 16 bits CRC - Enable on-chip CRC generation/checking
0x01, // Up to 5 bytes address for channel 1
0x01, // Up to 5 bytes address for channel 1
0x01, // Up to 5 bytes address for channel 1
0x01, // Up to 5 bytes address for channel 1
0x01, // Up to 5 bytes address for channel 1
0x01, // Up to 5 bytes address for channel 2
0x01, // Up to 5 bytes address for channel 2
0x01, // Up to 5 bytes address for channel 2
0x01, // Up to 5 bytes address for channel 2
0x01, // Up to 5 bytes address for channel 2
0x00, // Length of data payload section RX channel 1
0x00  // Length of data payload section RX channel 2
};

unsigned char trf_recv_buf_1[TRF_BUFFER_SIZE];
unsigned char trf_recv_buf_2[TRF_BUFFER_SIZE];
unsigned char trf_send_buf[TRF_BUFFER_SIZE];
unsigned char trf_flag;

void delay_ms(unsigned int t)
{
	while(t > 0)
	{
		_delay_ms(1);
		t--;
	}
}

void delay_us(unsigned int t)
{
	while(t > 0)
	{
		_delay_us(1);
		t--;
	}
}

void trf_put_byte(unsigned char b)
{  //msb bit first
	unsigned char i;
        
	// Define o pino de data como saida
	DTRF_DATA |= (1<<TRF_DATA);
	for(i=0 ; i < 8 ; i++)
	{
		POTRF_CLK1 &= ~(1<<TRF_CLK1);
		if((b & 0x80)==0x80)
			POTRF_DATA |= (1<<TRF_DATA);
		else
			POTRF_DATA &= ~(1<<TRF_DATA);
		delay_us(TRF_TS);
		POTRF_CLK1 |= (1<<TRF_CLK1);  // clock out on rising edge
		delay_us(TRF_TH);
		b=b<<1;
	};
	POTRF_CLK1 &= ~(1<<TRF_CLK1);
	// Define o pino de data como entrada
	DTRF_DATA &= ~(1<<TRF_DATA);
}

unsigned char trf_get_byte()
{
	unsigned char i=0, b=0;
        
	// Define o pino de data como entrada
	DTRF_DATA &= ~(1<<TRF_DATA);
        
	while(i < 8)
	{
		POTRF_CLK1 |= (1<<TRF_CLK1);
		delay_us(TRF_TCLK2DATA);
		b<<=1;
		if ((PITRF_DATA & (1<<TRF_DATA)) == (1<<TRF_DATA))
			b++;
		POTRF_CLK1 &= ~(1<<TRF_CLK1);
		delay_us(TRF_TCLK2DATA);
		i++;
	};
	return(b);
}

void trf_init()
{
//The configuration word is shifted in MSB first on positive CLK1 edges. New
//configuration is enabled on the falling edge of CS.        
        int b;

        //DATA1_W (# of bits)
        //TRF_DATA1_W = 256 - ADDR_W - CRC
        TRF_DATA1_W = TRF_BUFFER_SIZE*8; // =48 Default
        
        //DATA2_W (# of bits)
        //TRF_DATA2_W = 256 - ADDR_W - CRC
        TRF_DATA2_W = TRF_BUFFER_SIZE*8; // =48 Default
        
        //Configura os pinos como entrada ou saida
	DTRF_CS |= (1<<TRF_CS);
	DTRF_CLK1 |= (1<<TRF_CLK1);
	DTRF_CE |= (1<<TRF_CE);
	//DTRF_CLK2 |= (1<<TRF_CLK2);

	DTRF_DR1 &= ~(1<<TRF_DR1);
	//DTRF_DR2 &= ~(1<<TRF_DR2);
	DTRF_DATA &= ~(1<<TRF_DATA);
	//DTRF_DOUT2 &= ~(1<<TRF_DOUT2);

/*
	TRF_CE - saida
	TRF_CLK2 - saida
	TRF_CS - saida
	TRF_CLK1 - saida
	TRF_DATA - entrada
	TRF_DR1 - entrada
	TRF_DOUT2 - entrada
	TRF_DR2 - entrada
*/        
	POTRF_CS &= ~(1<<TRF_CS);
	POTRF_CE &= ~(1<<TRF_CE);
	POTRF_CLK1 &= ~(1<<TRF_CLK1);
	POTRF_DATA &= ~(1<<TRF_DATA);
	delay_us(TRF_TPD2A);
	delay_us(TRF_TD);
        
	POTRF_CS |= (1<<TRF_CS);
	delay_us(TRF_TCS2DATA);
               
	b = sizeof(trf_conf)-1;
	while(b >= 0)
	{
		trf_put_byte(trf_conf[b]);
		b--;
	};
	delay_us(TRF_TCS2DATA);
	POTRF_CS &= ~(1<<TRF_CS);
	delay_us(TRF_TCS2DATA);
	trf_2_rx();
		
	#ifdef atmega168
	// Configura os pinos de interrupcao externa
	// External Interrupt(s) initialization
	// INT0: On
	// INT0 Mode: Rising Edge
	// INT1: On
	// INT1 Mode: Rising Edge
	// Interrupt on any change on pins PCINT0-7: Off
	// Interrupt on any change on pins PCINT8-14: Off
	// Interrupt on any change on pins PCINT16-23: Off
	EICRA=0x0F;
	EIMSK=0x03;
	EIFR=0x03;
	PCICR=0x00;
	#endif

	#ifdef atmega8
	// External Interrupt(s) initialization
	// INT0: On
	// INT0 Mode: Rising Edge
	// INT1: On
	// INT1 Mode: Rising Edge
	GICR|=0xC0;
	MCUCR=0x0F;
	GIFR=0xC0;
	#endif

	trf_flag = 0;
}

void trf_send()
{
		int b;

        trf_2_tx();
        
        POTRF_CE |= (1<<TRF_CE);
        delay_us(TRF_TCE2DATA);
        
        // Endereco
        trf_put_byte(0x01);
        trf_put_byte(0x01);
        
        b = TRF_BUFFER_SIZE-1;
        while(b >= 0)
        {
                trf_put_byte(trf_send_buf[b]);
                b--;
        };
        
        delay_us(TRF_TCE2DATA);
        POTRF_CE &= ~(1<<TRF_CE);
        delay_us(TRF_TSBY2TXSB);
// ************ Tempo de transmissao *************
        delay_us(TOA);

        trf_2_rx();
		trf_flag |= TRF_TX_DONE;
}

void trf_copy_send(unsigned char *buf)
{
	unsigned char cont;
	cont = 0;
	while(cont < TRF_BUFFER_SIZE)
	{
		trf_send_buf[cont] = buf[cont];
		cont++;
	};
	trf_send();
}

void trf_recv()
{
        int b;
        
        b = TRF_BUFFER_SIZE-1;
        while(b >= 0)
        {
                trf_recv_buf_1[b] = trf_get_byte();
                b--;
        };
		trf_flag |= TRF_RX_DONE;
}

void trf_2_tx()
{
	POTRF_CS &= ~(1<<TRF_CS);
	POTRF_CE &= ~(1<<TRF_CE);
	POTRF_CLK1 &= ~(1<<TRF_CLK1);
	POTRF_DATA &= ~(1<<TRF_DATA);

	// Entra em Standby
	delay_us(TRF_TD);
	// Entra em Configuration
	POTRF_CS |= (1<<TRF_CS);
	delay_us(TRF_TSBY2RX); // Espera um pouco mais
	delay_us(TRF_TCS2DATA);

	// Define o pino de data como saida
	DTRF_DATA |= (1<<TRF_DATA);

	POTRF_DATA &= ~(1<<TRF_DATA); // 0 para TX, 1 para RX
	delay_us(TRF_TS);
	POTRF_CLK1 |= (1<<TRF_CLK1);
	delay_us(TRF_TH);
	POTRF_CLK1 &= ~(1<<TRF_CLK1);
	POTRF_DATA &= ~(1<<TRF_DATA);

	delay_us(TRF_TCS2DATA);
	POTRF_CS &= ~(1<<TRF_CS);
	delay_us(TRF_TD);
	POTRF_CE |= (1<<TRF_CE);

	// Define o pino de data como entrada
	DTRF_DATA &= ~(1<<TRF_DATA);
	delay_us(TRF_TSBY2TXSB);
}

void trf_2_rx()
{
	POTRF_CS &= ~(1<<TRF_CS);
	POTRF_CE &= ~(1<<TRF_CE);
	POTRF_CLK1 &= ~(1<<TRF_CLK1);
	POTRF_DATA &= ~(1<<TRF_DATA);
	delay_us(TRF_TD);
        
	POTRF_CS |= (1<<TRF_CS);
	delay_us(TRF_TSBY2RX);
	delay_us(TRF_TCS2DATA);
        
	// Define o pino de data como saida
	DTRF_DATA |= (1<<TRF_DATA);

	POTRF_DATA |= (1<<TRF_DATA); // 0 para TX, 1 para RX
	delay_us(TRF_TS);
	POTRF_CLK1 |= (1<<TRF_CLK1);
	delay_us(TRF_TH);
	POTRF_CLK1 &= ~(1<<TRF_CLK1);
	POTRF_DATA &= ~(1<<TRF_DATA);

	delay_us(600);
	//delay_us(TRF_TCS2DATA);
	POTRF_CS &= ~(1<<TRF_CS);
	delay_us(TRF_TD);
	POTRF_CE |= (1<<TRF_CE);

	// Define o pino de data como entrada
	DTRF_DATA &= ~(1<<TRF_DATA);
	delay_us(TRF_TSBY2RX);
}


