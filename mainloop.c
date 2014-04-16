/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8 noet: */
/*
 * Generate a video clock signal.
 */
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define cbi(X, N)	((X) &= ~(1<<(N)))
#define sbi(X, N)	((X) |=  (1<<(N)))

// debug on the led
static inline void
led(int val)
{
	if (val)
		sbi(DDRD, 6);
	else
		cbi(DDRD, 6);
}

// Input buffer for commands
uint8_t rx_buf[ 32 ];
uint8_t rx_head;
uint8_t rx_tail;

// Which raster line are we drawing
uint8_t line;


#define XRES	44
#define YRES	20
#define FONT_HEIGHT 12
uint8_t fb[ YRES ][ XRES ];
uint8_t * row;

extern const uint8_t font[] PROGMEM;
const uint8_t * font_line;
uint8_t subline;

/*
 * Video "data" is on MOSI, PORTB2 with a 1 K Ohm resistor
 * Video "!sync" is on PORTB7, with a 330 Ohm resistor
 */
#define VIDEO_SYNC_PORT	PORTB
#define VIDEO_SYNC_DDR	DDRB
#define VIDEO_SYNC_PIN	7

#define VIDEO_DATA_PORT	PORTB
#define VIDEO_DATA_DDR	DDRB
#define VIDEO_DATA_PIN	2
#define VIDEO_DATA_SS_PIN	0



/** A sync pulse is 0.0 V, so both low. */
static inline void
video_sync(void)
{
	cbi( VIDEO_SYNC_PORT, VIDEO_SYNC_PIN );
	cbi( VIDEO_DATA_PORT, VIDEO_DATA_PIN );
}


/** Generate a "black" pulse, of 0.3 V */
static inline void
video_black(void)
{
	sbi( VIDEO_SYNC_PORT, VIDEO_SYNC_PIN );
	cbi( VIDEO_DATA_PORT, VIDEO_DATA_PIN );
}

/** Generate a "Gray" pulse with a mid-level signal */
static inline void
video_gray(void)
{
	sbi( VIDEO_DATA_PORT, VIDEO_DATA_PIN );
	cbi( VIDEO_SYNC_PORT, VIDEO_SYNC_PIN );
}

/** Generate a "white" pulse of 1 V with both lines high. */
static inline void
video_white(void)
{
	sbi( VIDEO_SYNC_PORT, VIDEO_SYNC_PIN );
	sbi( VIDEO_DATA_PORT, VIDEO_DATA_PIN );
}



void
delay_until(
	uint8_t stop_time
)
{
	stop_time *= 2;
	const uint8_t short_stop = stop_time - 6;

	while( TCNT0 < short_stop )
	{
		uint8_t tmp_head = rx_head;
		if( tmp_head >= sizeof( rx_buf ) )
			continue;

#ifdef CONFIG_SERIAL
		if( !bit_is_set( UCSRA, RXC ) )
			continue;

		rx_buf[ tmp_head ] = UDR;
		rx_head = tmp_head + 1;
#endif
	}

	while( TCNT0 < stop_time )
		;
}


static void
video_setup(void)
{
	// Three more us of black (finish the previous frame)
	// The lead-out of the previous frame gave us at least
	// 1 us, the interrupt has given us exactly 1 us of black,
	// for a total of 5 us.
	video_black();
	delay_until( 3 );

	// Generate a five us sync pulse
	video_sync();
	delay_until( 8 );

	// And a 5 us lead-in
	video_black();
	delay_until( 12 );
}


void
convert_fb(void)
{
	uint8_t x, y;

	for( y=0 ; y < YRES ; y++ )
	{
		for( x=0 ; x < XRES ; x ++ )
		{
			//fb[ y ][ x ] = 'A' + (x+y) % 26; // 26; // (x) & 15;
			fb[ y ][ x ] = 'A' + (x) % 26; // 26; // (x) & 15;
		}
	}

	font_line = font;
	subline = 0;
}



/*
 * Translate the characters into scanlines of pixels.  Each character
 * cell is 8-bits wide and is translated from the font_line array.
 * The loop is very carefully timed to have exactly 16 clocks --
 * the nops are there for a reason.
 * 
 * It should translate into this assembly:
 *
.L6:
        ld r21,X+			2
        movw r30,r18			1
        add r30,r21			1
        adc r31,__zero_reg__		1
        lpm r21, Z			3
        nop				1
        nop				1
        nop				1
        in r25,46-0x20			1
        sbi 46-0x20,7			1
        out 47-0x20,r21			1
        subi r20,lo8(-(-1))		1
        brne .L6			2
  					-- 17 clocks (-1 for the out)
 *
 * If we try to make it exactly 16 clocks the SPI hardware gets
 * confused and doesn't shift our data out right.  This leaves a
 * tiny small space of 1 clock between the cells.
 */

void
horizontal(void)
{
	video_setup();

	uint8_t x = XRES;

	const uint8_t * text = row;

	do {
		// Prefetch
		const uint8_t c = *text++;
		const uint8_t pixel = pgm_read_byte( &font_line[ c ] );

		asm( "nop" );		// Careful timing NOPs
		asm( "nop" );
		asm( "nop" );
		//asm( "nop" );

		//SPCR = 1 << MSTR | 1 << CPHA | 1 << SPE;
		// Enable the SPI hardare
		sbi( SPCR, SPE );

		// Write the new pixel
		SPDR = pixel;

	} while( --x );

	asm( "nop" );		// Careful timing NOPs
	asm( "nop" );

	// Wait for the last bit to finish, then shut off the SPI hardware
	// (We know that it has already finished!)
	cbi( SPCR, SPE );

	video_gray();
	video_black();
	cbi( PORTC, 2 );

	// If we have not drawn the last subline of this row,
	// rewind the row pointer to draw the next subline on the
	// next scanline and move to the next line of the font.
	if( subline != FONT_HEIGHT-1 )
	{
		font_line += 128;
		subline++;

		// Ensure that we take the same time as the
		// other branch
		asm( "nop" );
		asm( "nop" );
		asm( "nop" );
		asm( "nop" );
		asm( "nop" );
		asm( "nop" );
	} else {
		// Starting a new line
		font_line = font;
		row += XRES;
		subline = 0;
	}

	sbi( PORTC, 2 );
	video_black();
}



uint8_t row_in;
uint8_t col_in;

void
process_char(
	uint8_t			c
)
{
#ifdef CONFIG_SERIAL
	if( c == 27 )
		reset();

	fb[ row_in ][ col_in ] = c;
	if( col_in == XRES-1 )
	{
		col_in = 0;
		if( row_in == YRES-1 )
			row_in = 0;
		else
			row_in++;
	} else {
		col_in++;
	}
#endif
}

/*
 * Generate actual sync pulses with zero-voltage output.
 *
 * We will spend most of our time waiting for a character on the
 * serial port.  Since TCNT0 counts up to OCR0, we can delay for
 * 100 clock pulses less than that.
 */
void
vsync( void )
{
	video_sync();
	const uint8_t stop = OCR0A - 30;

	uint8_t rx_tail = 0;
	const uint8_t tmp_head = rx_head;

	while( TCNT0 < stop && rx_tail < tmp_head )
	{
		process_char( rx_buf[ rx_tail++ ] );
	}

	while( TCNT0 < stop )
	{
#ifdef CONFIG_SERIAL
		if( bit_is_set( UCSRA, RXC ) )
		{
			sbi( PORTC, 2 );
			process_char( UDR );
			cbi( PORTC, 2 );
		}
#endif
	}

	video_black();
	rx_head = 0;
}


/*
 * Generate black lines with no video
 */
void
vblank( void )
{
	video_setup();
}




/*
 * Wait for the horizontal sync interval timer to
 * expire (63.55 us, but since we have 0.5 us clock, 63.5 us).
 * We go to sleep, so we will wake up when it is done
 */
void
hsync(void)
{
#ifdef CONFIG_SLEEP
	sei();
	sleep_mode();
	sbi( PORTC, 2 );
	cli();
#else
	while (bit_is_clear(TIFR0, OCF0A))
		;
	sbi(TIFR0, OCF0A);
	video_black();
	sbi( PORTC, 2 );
#endif
}


// This interrupt only exists to wake us from a sleep mode.
ISR( TIMER0_COMPA_vect )
{
	video_black();
	cbi( PORTC, 2 );
}


int
main( void )
{
	// set for 16 MHz clock
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
	CPU_PRESCALE(0);
	sbi(PORTD, 6);
	led(1);

	// Disable the ADC
	ADMUX = 0;
	// Timer 0 @ clk/8 counts 0.5 us ticks up to 127 == 63.50 us
	TCCR0A = 0
		| 1 << WGM01
		| 0 << WGM00
		;
	TCCR0B = 0
		| 0 << CS02
		| 1 << CS01
		| 0 << CS00
		;
	OCR0A = 127;
	TCNT0 = 0;
	sbi( TIFR0, OCF0A );
#ifdef CONFIG_SLEEP
	sbi( TIMSK0, OCIE0A );
#endif

	// NTSC horizontal sync time is 63.55 us, we generate 63.50 us
	sbi( VIDEO_DATA_DDR, VIDEO_DATA_SS_PIN);
	sbi( VIDEO_DATA_DDR, VIDEO_DATA_PIN );
	sbi( VIDEO_SYNC_DDR, VIDEO_SYNC_PIN );
	video_black();

#ifdef CONFIG_SERIAL
	// UART is 38400
	UBRRL = 25;
	sbi( UCSRB, RXEN );
	sbi( UCSRB, TXEN );
#endif

	// Enable SPI master mode at Clk/4, but with SPI2X, so
	// we have Clk/2.  For a 16 MHz clock and a 63.55 us - 15 us
	// scan line, this gives us 388 pixels across.  With a
	// 
	// By enabling the clock phase we do not get the initial HIGH
	// that would cause a white line.
	SPCR = 1 << MSTR | 1 << CPHA;
	sbi( SPSR, SPI2X );

	// We use PORTC for debugging
	DDRC = 0xFF;
	PORTC = 0;

	set_sleep_mode( SLEEP_MODE_IDLE );

	convert_fb();

	while(1)
	{
		// 243 vertical lines: 8 header, 30 text, 8 trailer
		for( line=0 ; line < (243-YRES*FONT_HEIGHT)/2 ; line++ )
		{
			hsync();
			vblank();
		}

		led(1);
		row = &fb[0][0];
		subline = 0;

		for( line=0 ; line < YRES*FONT_HEIGHT ; line++ )
		{
			hsync();
			horizontal();
		}


		for( line=0 ; line < (243-YRES*FONT_HEIGHT)/2 ; line++ )
		{
			hsync();
			vblank();
		}

		led(0);

		// 5 lines of blanking interval
		for( line=0 ; line < 5 ; line++ )
		{
			hsync();
			vblank();
		}

		// 3 lines of vsync
		for( line=0 ; line < 3 ; line++ )
		{
			hsync();
			vsync();
		}

		// 10 lines of vertical retrace
		for( line=0 ; line < 10 ; line++ )
		{
			hsync();
			vblank();
		}

	}


}
