#include <atmel_start.h>
#include <util/delay.h>
#include "NokiaLCD.h"

// Declare your global variables here

// Standard Input/Output functions
#include <stdio.h>

#define F_CPU  16000000
// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))
// UART
#define UART_BAUD_RATE  9600
#define UART_BAUD_REGISTERS  (((F_CPU / (UART_BAUD_RATE * 16UL))) - 1)

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
	ADMUX=adc_input | ADC_VREF_TYPE;
	// Delay needed for the stabilization of the ADC input voltage
	_delay_us(10);
	// Start the AD conversion
	ADCSRA|=(1<<ADSC);
	// Wait for the AD conversion to complete
	while ((ADCSRA & (1<<ADIF))==0);
	ADCSRA|=(1<<ADIF);
	return ADCW;
}

int usart_putchar(char character, FILE *stream)
{
	while ((UCSR0A & (1 << UDRE0)) == 0) {};

	UDR0 = character;

	return 0;
}

int usart_getchar(FILE *stream)
{
	while ((UCSR0A & (1 << RXC0)) == 0) {};

	return UDR0;
}

FILE uart_str = FDEV_SETUP_STREAM(usart_putchar, usart_getchar, _FDEV_SETUP_RW);

void main(void)
{
	// Declare your local variables here
	int i, j;
	char str[16];
	stdout = &uart_str;

	// Crystal Oscillator division factor: 1
	#pragma optsize-
	CLKPR=(1<<CLKPCE);
	CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
	#ifdef _OPTIMIZE_SIZE_
	#pragma optsize+
	#endif

	// Input/Output Ports initialization
	// Port B initialization
	// Function: Bit7=In Bit6=In Bit5=Out Bit4=In Bit3=Out Bit2=Out Bit1=Out Bit0=Out
	DDRB=(0<<DDB7) | (0<<DDB6) | (1<<DDB5) | (0<<DDB4) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1) | (1<<DDB0);
	// State: Bit7=T Bit6=T Bit5=0 Bit4=T Bit3=0 Bit2=0 Bit1=0 Bit0=1
	PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (1<<PORTB0);

	// Port C initialization
	// Function: Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=Out Bit0=Out
	DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (1<<DDC1) | (1<<DDC0);
	// State: Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=1 Bit0=1
	PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (1<<PORTC1) | (1<<PORTC0);

	// Port D initialization
	// Function: Bit7=Out Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRD=(1<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
	// State: Bit7=0 Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: Timer 0 Stopped
	// Mode: Normal top=0xFF
	// OC0A output: Disconnected
	// OC0B output: Disconnected
	TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
	TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
	TCNT0=0x00;
	OCR0A=0x00;
	OCR0B=0x00;

	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: Timer1 Stopped
	// Mode: Normal top=0xFFFF
	// OC1A output: Disconnected
	// OC1B output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1AH=0x00;
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;

	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: Timer2 Stopped
	// Mode: Normal top=0xFF
	// OC2A output: Disconnected
	// OC2B output: Disconnected
	ASSR=(0<<EXCLK) | (0<<AS2);
	TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
	TCCR2B=(0<<WGM22) | (0<<CS22) | (0<<CS21) | (0<<CS20);
	TCNT2=0x00;
	OCR2A=0x00;
	OCR2B=0x00;

	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (0<<TOIE0);

	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1);

	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (0<<TOIE2);

	// External Interrupt(s) initialization
	// INT0: Off
	// INT1: Off
	// Interrupt on any change on pins PCINT0-7: Off
	// Interrupt on any change on pins PCINT8-14: Off
	// Interrupt on any change on pins PCINT16-23: Off
	EICRA=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
	EIMSK=(0<<INT1) | (0<<INT0);
	PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);

	// USART initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART Receiver: On
	// USART Transmitter: On
	// USART Mode: Asynchronous
	// USART Baud Rate: 9600
	UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
	UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
	UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
	UBRR0H=0x00;
	UBRR0L=0x67;

	// Analog Comparator initialization
	// Analog Comparator: Off
	// The Analog Comparator's positive input is
	// connected to the AIN0 pin
	// The Analog Comparator's negative input is
	// connected to the AIN1 pin
	ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
	// Digital input buffer on AIN0: On
	// Digital input buffer on AIN1: On
	DIDR1=(0<<AIN0D) | (0<<AIN1D);

	// ADC initialization
	// ADC Clock frequency: 1000,000 kHz
	// ADC Voltage Reference: AVCC pin
	// ADC Auto Trigger Source: ADC Stopped
	// Digital input buffers on ADC0: On, ADC1: On, ADC2: On, ADC3: On
	// ADC4: On, ADC5: On
	DIDR0=(0<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (0<<ADC1D) | (0<<ADC0D);
	ADMUX=ADC_VREF_TYPE;
	ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
	ADCSRB=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

	// SPI initialization
	// SPI Type: Master
	// SPI Clock Rate: 4000,000 kHz
	// SPI Clock Phase: Cycle Start
	// SPI Clock Polarity: Low
	// SPI Data Order: MSB First
	SPCR=(0<<SPIE) | (1<<SPE) | (0<<DORD) | (1<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);
	SPSR=(0<<SPI2X);

	// TWI initialization
	// TWI disabled
	TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

	LcdInit ();

	while (1)
	{
		// Place your code here
		LcdClear ();

		i = read_adc(6);
		printf ("%d ", i);
		snprintf(str, sizeof(str), "Throttle: %d", i);
		LcdGotoXY (0, 0);
		LcdStr (FONT_1X, str);

		i = read_adc(7);
		printf ("%d\n\r", i);
		snprintf(str, sizeof(str), "Rudder: %d", i - 511);
		LcdGotoXY (0, 1);
		LcdStr (FONT_1X, str);
		
		j = scanf ("%d", &i);
		if (j > 0) {
			i = i * 19;
			snprintf(str, sizeof(str), "Boat: %d.%d V", i/1000, (i%1000)/100);
			LcdGotoXY (0, 3);
			LcdStr (FONT_1X, str);
		}

		i = read_adc(5) * 19;
		snprintf(str, sizeof(str), "Remote: %d.%d V", i/1000, (i%1000)/100);
		LcdGotoXY (0, 2);
		LcdStr (FONT_1X, str);

		LcdUpdate ();
		_delay_ms(100);

	}
}