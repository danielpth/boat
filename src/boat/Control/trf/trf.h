#define TRF_TX_DONE 1
#define TRF_RX_DONE 2
#define TRF_RX2_DONE 4

#define TRF_CS          4
#define TRF_CLK1        5
#define TRF_DATA        4
#define TRF_DR1         2
//#define TRF_DOUT2       0
//#define TRF_DR2         2
#define TRF_VCC
#define TRF_GND
#define TRF_CE          3
//#define TRF_CLK2        4

#define POTRF_CS        PORTD
#define POTRF_CLK1      PORTB
#define POTRF_DATA      PORTB
#define POTRF_DR1       PORTD
#define POTRF_DOUT2     PORTC
#define POTRF_DR2       PORTD
#define POTRF_CE        PORTD
#define POTRF_CLK2      PORTD

#define PITRF_CS        PIND
#define PITRF_CLK1      PINB
#define PITRF_DATA      PINB
#define PITRF_DR1       PIND
#define PITRF_DOUT2     PINC
#define PITRF_DR2       PIND
#define PITRF_CE        PIND
#define PITRF_CLK2      PIND

#define DTRF_CS         DDRD
#define DTRF_CLK1       DDRB
#define DTRF_DATA       DDRB
#define DTRF_DR1        DDRD
#define DTRF_DOUT2      DDRC
#define DTRF_DR2        DDRD
#define DTRF_CE         DDRD
#define DTRF_CLK2       DDRD

// Valores em us
#define TRF_TPD2SBY 3000
#define TRF_TPD2A 3000
#define TRF_TD 2
#define TRF_TCS2DATA 10
#define TRF_TCE2DATA 10
#define TRF_TSBY2TXSB 200
#define TRF_TDR2CLK 2
#define TRF_TCLK2DATA 2
#define TRF_TSBY2RX 205

//TOA = 1/datarate * (#databits +1)
// datarate = 250kbps / 1Mbps
// #databits = DATAx_W + ADDR_W + CRC_W (+1 eh do preamble)
// #databits = 128 + 16 + 16 + 1 = 75
//#define TOA (128 + 16 + 16 + 1)*1000/250; // us
#define TOA 650 // us

//#define TRF_TH 5
//#define TRF_TS 5

#define TRF_TH 10
#define TRF_TS 10

#define TRF_DATA1_W trf_conf[13]
#define TRF_DATA2_W trf_conf[14]

//#define TRF_BUFFER_SIZE 16
#define TRF_BUFFER_SIZE 4

extern unsigned char trf_conf[15];
extern unsigned char trf_recv_buf_1[TRF_BUFFER_SIZE];
extern unsigned char trf_recv_buf_2[TRF_BUFFER_SIZE];
extern unsigned char trf_send_buf[TRF_BUFFER_SIZE];
extern unsigned char trf_flag;

void delay_ms(unsigned int t);
void delay_us(unsigned int t);
void trf_put_byte(unsigned char b);
unsigned char trf_get_byte();
void trf_init();
void trf_send();
void trf_copy_send(unsigned char *buf);
void trf_recv();
void trf_2_tx();
void trf_2_rx();


