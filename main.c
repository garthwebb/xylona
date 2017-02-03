#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


/***************************************
* CONSTANTS                            *
***************************************/

// **************
// Input signals

// This input switch is set active low
#define is_pot_adjust() !(PINC & _BV(PC2))

#define is_tempo_1x()   !(PINC & _BV(PC3))
#define is_tempo_2x()   !(PINC & _BV(PC4))
#define is_tempo_3x()   !(PINC & _BV(PC5))

// This input is active low
#define is_count_reset()	!(PINC & _BV(PC0))

// **************
// Output signals

// Activate the solenoid
#define solenoid_on()	PORTD |= _BV(PD4)
#define solenoid_off()	PORTD &= ~_BV(PD4)

#define send_count_reset() PORTC &= ~_BV(PC0)
#define clear_count_reset() PORTC |= _BV(PC0)

// Signals SDI, CLK and LE to control the LED register
#define led_sdi_set()	PORTD |= _BV(PD0)
#define led_sdi_clear()	PORTD &= ~_BV(PD0)

#define led_clk_set()	PORTD |= _BV(PD1)
#define led_clk_clear()	PORTD &= ~_BV(PD1)

#define led_le_set()	PORTD |= _BV(PD2)
#define led_le_clear()	PORTD &= ~_BV(PD2)

// Control the LEDs connected to the LED register
#define cnt_led_on()	LIGHT_VECTOR |= _BV(0); UPDATE_LED_REGISTER = 1
#define cnt_led_off()	LIGHT_VECTOR &= ~_BV(0); UPDATE_LED_REGISTER = 1

#define beat_led_on()	LIGHT_VECTOR |= _BV(1); UPDATE_LED_REGISTER = 1
#define beat_led_off()	LIGHT_VECTOR &= ~_BV(1); UPDATE_LED_REGISTER = 1

#define btn_led_on()	LIGHT_VECTOR |= _BV(2); UPDATE_LED_REGISTER = 1
#define btn_led_off()	LIGHT_VECTOR &= ~_BV(2); UPDATE_LED_REGISTER = 1

//* Normal lights
#define m1_led_on()		LIGHT_VECTOR |= _BV(4); UPDATE_LED_REGISTER = 1
#define m1_led_off()	LIGHT_VECTOR &= ~_BV(4); UPDATE_LED_REGISTER = 1

#define m2_led_on()		LIGHT_VECTOR |= _BV(5); UPDATE_LED_REGISTER = 1
#define m2_led_off()	LIGHT_VECTOR &= ~_BV(5); UPDATE_LED_REGISTER = 1

#define m3_led_on()		LIGHT_VECTOR |= _BV(6); UPDATE_LED_REGISTER = 1
#define m3_led_off()	LIGHT_VECTOR &= ~_BV(6); UPDATE_LED_REGISTER = 1

#define m4_led_on()		LIGHT_VECTOR |= _BV(7); UPDATE_LED_REGISTER = 1
#define m4_led_off()	LIGHT_VECTOR &= ~_BV(7); UPDATE_LED_REGISTER = 1
//*/

/* Tambourine lights
#define m1_led_on()		LIGHT_VECTOR |= _BV(3); UPDATE_LED_REGISTER = 1
#define m1_led_off()	LIGHT_VECTOR &= ~_BV(3); UPDATE_LED_REGISTER = 1

#define m2_led_on()		LIGHT_VECTOR |= _BV(4); UPDATE_LED_REGISTER = 1
#define m2_led_off()	LIGHT_VECTOR &= ~_BV(4); UPDATE_LED_REGISTER = 1

#define m3_led_on()		LIGHT_VECTOR |= _BV(5); UPDATE_LED_REGISTER = 1
#define m3_led_off()	LIGHT_VECTOR &= ~_BV(5); UPDATE_LED_REGISTER = 1

#define m4_led_on()		LIGHT_VECTOR |= _BV(6); UPDATE_LED_REGISTER = 1
#define m4_led_off()	LIGHT_VECTOR &= ~_BV(6); UPDATE_LED_REGISTER = 1
//*/


//* Normal flood
#define f1_led_on()		LIGHT_VECTOR |= _BV(8); UPDATE_LED_REGISTER = 1
//*/

/* Xylo #4 flood
#define f1_led_on()		LIGHT_VECTOR |= _BV(3); UPDATE_LED_REGISTER = 1
//*/

#define f1_led_off()	LIGHT_VECTOR &= ~_BV(8); UPDATE_LED_REGISTER = 1

#define f2_led_on()		LIGHT_VECTOR |= _BV(9); UPDATE_LED_REGISTER = 1
#define f2_led_off()	LIGHT_VECTOR &= ~_BV(9); UPDATE_LED_REGISTER = 1

#define m_all_off()		LIGHT_VECTOR &= ~(_BV(4) | _BV(5) | _BV(6) | _BV(7)); UPDATE_LED_REGISTER = 1;

// **************
// Timing

// Some delays measured in number of times the interrupt is triggered
#define DELAY_30MS	   213
#define DELAY_500MS    3551

// The base tempos of the instrument based on number of times the interrupt is triggered
#define TEMPO_1X_DELAY 3558
#define TEMPO_2X_DELAY 1779
#define TEMPO_3X_DELAY 1186

// These define how far into the beat to accept input for that beat.  For example
// someone may hit the input switch early anticipating the next beat.  Be lenient
// and set the next beat rather than the current one
#define TEMPO_1X_TRIGGER (int)(TEMPO_1X_DELAY * 0.75)
#define TEMPO_2X_TRIGGER (int)(TEMPO_2X_DELAY * 0.75)
#define TEMPO_3X_TRIGGER (int)(TEMPO_3X_DELAY * 0.75)

// **************
// Misc

// The maximum number of times this instrument will be silent
#define MAX_BEATLESS_LOOPS 4

// Number of loops a beat will last before decaying
#define BEAT_TTL 5

// Whether this is the timing master or slave
#define TIMING_MASTER 0
#define XYLO_POT 1

#define SOLENOID_WOOD_BLOCK_SM 3
#define SOLENOID_WOOD_BLOCK_MED 2
#define SOLENOID_WOOD_BLOCK_LG 3
#define SOLENOID_TAMBOURINE 3
#define SOLENOID_BOWL 1

/***************************************
* FUNCTION PROTOTYPES                  *
***************************************/

void io_init (void);
void init_timer(void);

void led_test(void);
void tempo_test(void);
void pot_test(void);

void clear_beats(void);
void update_lights (void);

void delay_us(int x);
void delay_ms(uint16_t x);

void set_tempo(void);
int is_beat_hit (void);
void pot_adjust_mode(void);
void set_solenoid_on_time (void);

/***************************************
* GLOBAL VARIABLES                     *
***************************************/

volatile int BEAT_BUTTON_DOWN = 0;
volatile int BEAT_BUTTON_COUNTER = 0;

volatile int IGNORE_RESET = 0;

// Keep track of what beats have been set
volatile int SET_BEATS[16];

// What to play when there is no one playing

// Blank beat
volatile int MAINTENANCE_BEAT[16] = {0, 0, 0, 0,
									 0, 0, 0, 0,
									 0, 0, 0, 0,
									 0, 0, 0, 0};

// Every measure beat
//volatile int MAINTENANCE_BEAT[16] = {1, 0, 0, 0,
//									 1, 0, 0, 0,
//									 1, 0, 0, 0,
//									 1, 0, 0, 0};

// Once per loop beat
//volatile int MAINTENANCE_BEAT[16] = {1, 0, 0, 0,
//									 0, 0, 0, 0,
//									 0, 0, 0, 0,
//									 0, 0, 0, 0};


// Count how many beatless loops go by
volatile int BEATLESS_LOOPS = 0;

// Count how many beats have been in the current loop
volatile int BEATS_IN_LOOP  = 0;

// The current beat of the 16 looped through
volatile int BEAT = 0;

// Count how many times the timing interrupt has been run
volatile uint16_t TIME_COUNTER = 0;

// Counters to keep track of things that have been turned on so that they can be turned off again
volatile uint16_t BEAT_LED_ON_COUNTER = 0;
volatile uint16_t SOLENOID_COUNTER = 0;

// This tells us how long to wait before advancing the beat
volatile uint16_t TEMPO_DELAY = TEMPO_1X_DELAY;
volatile uint16_t BEAT_TRIGGER = TEMPO_1X_TRIGGER;

// The light vector contains 16 bits, one for every LED controlled by the LED register
volatile uint16_t LIGHT_VECTOR = 0;

// Set when the LED register needs to be written to
volatile int UPDATE_LED_REGISTER = 0;

uint16_t SOLENOID_ON_TIME = DELAY_30MS*SOLENOID_WOOD_BLOCK_SM;//SOLENOID_WOOD_BLOCK_SM;

/***************************************
* MAIN PROGRAM                         *
***************************************/

int main (void) {
	io_init();

	// This blocks indefinitely -- not for normal startup testing
	//pot_test();

	if (XYLO_POT) {
 		set_solenoid_on_time();
 	}

	// Before setting up the interrupts, test the LEDs
	led_test();

	// Test the tempo switch
	tempo_test();

    init_timer();
	clear_beats();

/*
	uint16_t x = 10000;
	while (x) {
		test_on();
		TIFR2 |= 0x01;//Clear any interrupt flags on Timer2
		TCNT2 = 99; //256 - 157 = 99 : Preload timer 2 for 99 clicks == 1ms.
		while(!(TIFR2 & 0x01));

		test_off();
		TIFR2 |= 0x01;//Clear any interrupt flags on Timer2
		TCNT2 = 99; //256 - 157 = 99 : Preload timer 2 for 99 clicks == 1ms.
		while(!(TIFR2 & 0x01));
	}
*/

	// Turn on the facade & button lights
	btn_led_on();
	f1_led_on();
	f2_led_on();

	while (1) {
		//pot_adjust_mode();
		//continue;

		// If the set beat switch is thrown, pull the solenoid pin high
		if (is_beat_hit()) {

			// If this beat hasn't been set and the beat button is not being held down
			// then record this beat and fire up the solenoid
			if (!SET_BEATS[TIME_COUNTER <= BEAT_TRIGGER ? BEAT : (BEAT+1)%16] &&
			    !BEAT_BUTTON_DOWN) {
				BEAT_BUTTON_DOWN = 1;
				BEAT_BUTTON_COUNTER = 0;

                // If we're a bit before the next beat, record this hit
                // on that next beat (for those off rhythym times people have)
				if (TIME_COUNTER <= BEAT_TRIGGER) {
				    SET_BEATS[BEAT] = BEAT_TTL;
				} else {
					SET_BEATS[(BEAT+1)%16] = BEAT_TTL;
				}

				solenoid_on();
				beat_led_on();
				SOLENOID_COUNTER = SOLENOID_ON_TIME;

				// Reset the beatless loop counter as soon as we have some input
				BEATLESS_LOOPS = 0;
			}
		} else {
			// Ignore button up events for about 60ms
			if (BEAT_BUTTON_COUNTER > 500) {
				BEAT_BUTTON_DOWN = 0;
			}
		}

		// If we're not the timing master, listen for the timing signal
		if (!TIMING_MASTER) {

			// If we aren't ignoring reset currently and we get the signal then
			// reset the counters and the lights.
			if (!IGNORE_RESET && is_count_reset()) {
				// Set the counters to roll over at the next interrupt
				TIME_COUNTER = TEMPO_DELAY;
				BEAT = 15;

				// Reset the lights
				m1_led_on();
				m2_led_off();
				m3_led_off();
				m4_led_off();

				// Don't listen to this signal for the next 30ms
				IGNORE_RESET = DELAY_30MS;
			}
		}

		// Send serial output to the LED register if there's been a change
		if (UPDATE_LED_REGISTER) {
			update_lights();
			UPDATE_LED_REGISTER = 0;
		}

		// Check to see if the tempo switch has changed
		set_tempo();

	}

    return 1;
}

void io_init (void) {

	// Set PD0-PD4 to outputs and PD5-PD7 to inputs on port D
	DDRD = _BV(PD0) | _BV(PD1) | _BV(PD2) | _BV(PD3) | _BV(PD4);
	// Set the inital value of port D to be zero
	PORTD = 0;

	if (TIMING_MASTER) {
		// All pins of PORTC are by default input pins, make PC0 an output. Write a 1 to PC2 - PC5 to
		// activate the internal pullup resistor
		DDRC = _BV(PC0);
		PORTC = _BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5);
	} else {
		// All pins of PORTC are by default input pins. Write a 1 to PC0, PC2 - PC5 to
		// activate the internal pullup resistor
		PORTC = _BV(PC0) | _BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5);
	}

//	TCCR2B = (1<<CS21); //Set Prescaler to 8. CS21=1
//	TCCR2B = (1<<CS20); //Set Prescaler to 0. CS21=1
	TCCR2B = _BV(CS20) | _BV(CS22); //Set Prescaler to 128.

	// Enable ADC and set 128 prescale
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

	// Enable ADC6
	ADMUX = _BV(MUX0);

	// Start new conversion
	ADCSRA |= _BV(ADSC);
}

void init_timer (void) {
    TIMSK0 = _BV(OCIE0A) | _BV(OCIE0B);  // Enable Interrupt TimerCounter0 Compare Match A (SIG_OUTPUT_COMPARE0A)
    TCCR0A = _BV(WGM01);  // Mode = CTC

    TCCR0B = _BV(CS02);   // Clock/256, 0.0000128 seconds per tick (12.8us)
//    TCCR0B = _BV(CS02) | _BV(CS00);   // Clock/1024, 0.001024 seconds per tick (actually 2ms)

    OCR0A = 10;          // 0.0000002*125 ~= .25 SIG_OUTPUT_COMPARE0A will be triggered 4 times per second.
    OCR0B = 125;          // 

    sei();
}

ISR(SIG_OUTPUT_COMPARE0A) {
	// Start by clearing the count reset if we're the master.  Below if we set
	// the count reset, this allows us one full interrupt period to have the
	// reset be set.
	if (TIMING_MASTER) {
		clear_count_reset();
	}

	TIME_COUNTER++;
	BEAT_LED_ON_COUNTER++;

	// Update the count of cycles the beat button is down
	if (BEAT_BUTTON_DOWN) {
		BEAT_BUTTON_COUNTER++;
	}

	// Mark of each beat as a number of counts of TIME_COUNTER
	if (TIME_COUNTER >= TEMPO_DELAY) {
		cnt_led_on();
		BEAT_LED_ON_COUNTER = 0;
		TIME_COUNTER = 0;

		// Increment the beat counter
		BEAT = (BEAT+1)%16;

		// Set the count reset.  We'll clear it below
		if (TIMING_MASTER && (BEAT == 0)) {
			send_count_reset();
		}

		// See if we need to engage the solenoid
		if (SET_BEATS[BEAT]) {
			solenoid_on();
			beat_led_on();
			SOLENOID_COUNTER = SOLENOID_ON_TIME;

			// Decay the beat
			SET_BEATS[BEAT]--;

			// Count how many beats have been in this loop
			BEATS_IN_LOOP++;
		}

		// If we've had too many beatleass loops, start playing the maintenence
		// beats to fill the space
		if (BEATLESS_LOOPS >= MAX_BEATLESS_LOOPS) {
		    
			// Keep this from incrementing past the size of the data type when we're
			// in this mode for a while
			BEATLESS_LOOPS = MAX_BEATLESS_LOOPS;

			if (MAINTENANCE_BEAT[BEAT]) {
				solenoid_on();
				beat_led_on();
				SOLENOID_COUNTER = SOLENOID_ON_TIME;
			}
		}

		// If this is the last BEAT and we still have no beats played, increment the
		// beatless loop counter 
		if (BEAT == 15) {
			if (BEATS_IN_LOOP == 0) {
				BEATLESS_LOOPS++;
			} else {
				// We're only counting consecutive beatless loops.  If we get just one
				// beat then clear everything out
				BEATS_IN_LOOP = 0;
				BEATLESS_LOOPS = 0;
			}
		}
		
		// Count off the measures
		if (BEAT == 0) {
			m_all_off();
		}
		if (BEAT >= 0) {
			m1_led_on();
		}
		if (BEAT >= 4) {
			m2_led_on();
		}
		if (BEAT >= 8) {
			m3_led_on();
		}
		if (BEAT >= 12) {
			m4_led_on();
		}
	}

	// 213 counts is about 30 ms
	if (BEAT_LED_ON_COUNTER >= DELAY_30MS) {
		cnt_led_off();
	}

	// If the solenoid counter is non-zero decrement it, turning off the
	// solenoid when it hits 1
	if (SOLENOID_COUNTER > 0) {
		if (SOLENOID_COUNTER == 1) {
			solenoid_off();
			beat_led_off();
		}
		SOLENOID_COUNTER--;
	}

	// Count down the ignore reset
	if (IGNORE_RESET > 0) {
		IGNORE_RESET--;
	}
}

void led_test (void) {
	for (int x = 0; x < 8; x++) {

		LIGHT_VECTOR |= _BV(x);
		update_lights();
		delay_ms(200);

		LIGHT_VECTOR &= ~_BV(x);
		update_lights();
		delay_ms(200);
	}
}

void tempo_test (void) {

	for (int x = 0; x < 3; x++) {
		if (is_tempo_3x()) {
			m1_led_on();
		
		} else if (is_tempo_2x()) {
			m1_led_on();
			m2_led_on();
	
		} else if (is_tempo_1x()) {
			m1_led_on();
			m2_led_on();
			m3_led_on();
	
		} else {
			m1_led_on();
			m2_led_on();
			m3_led_on();
			m4_led_on();
	
		}

		update_lights();
		delay_ms(100);
		m_all_off();	
		update_lights();
		delay_ms(100);
	}
}

int is_beat_hit (void) {
	int adc_num;
	int retval = 0;

	// See if there is an ADC conversion done
	if (ADCSRA & (_BV(ADIF))) {
		// Clear ADIF by writing a 1 (this sets the value to 0)
		ADCSRA |= _BV(ADIF);

		// Read the current value
		adc_num = ADC;

		if (adc_num > 512) {
			retval = 1;
		}

		// Start new conversion
		ADCSRA |= _BV(ADSC);
	}

	return retval;
}

void set_solenoid_on_time (void) {
	int adc_num;
	float adjust = 0.0;

	// Enable a check for the solenoid pot, ADC6
	ADMUX = _BV(MUX1) | _BV(MUX2);

	// Run the set several times to make sure it "takes"
	int x;
	for (x=0; x <= 10; x++) {

		// Wait for an ADC conversion to be done
		while (!(ADCSRA & _BV(ADIF))) {}

		// Clear ADIF by writing a 1 (this sets the value to 0)
		ADCSRA |= _BV(ADIF);

		// Read the current value which is between 0 and 1023
		adc_num = ADC;

		// Make this number go from -512 to 511 instead
		adc_num -= 512;
			
		// Adjust will be based on DELAY_30MS and will range from
		// -1/2 the value to 1/2 the value
		adjust = (DELAY_30MS/1.2)*(adc_num/512.0);

		SOLENOID_ON_TIME = DELAY_30MS + (int) adjust;

		// Start new conversion
		ADCSRA |= _BV(ADSC);
	}

	// Set ADC input to the piezo
	ADMUX = _BV(MUX0);

	// Start new conversion
	ADCSRA |= _BV(ADSC);

	// Wait one more to clear things out (somehow the trim pot still gets picked up here)
	while (!(ADCSRA & _BV(ADIF))) {}

	// Clear ADIF by writing a 1 (this sets the value to 0)
	ADCSRA |= _BV(ADIF);
		
	// Start new conversion
	ADCSRA |= _BV(ADSC);

}

void pot_adjust_mode(void) {
	int adc_num;
	float adjust = 0.0;

	// See if there is an ADC conversion done
	if (ADCSRA & (_BV(ADIF))) {
		// Clear ADIF by writing a 1 (this sets the value to 0)
		ADCSRA |= _BV(ADIF);

		// Read the current value which is between 0 and 1023
		adc_num = ADC;

		// Make this number go from -512 to 511 instead
		adc_num -= 512;
			
		// Adjust will be based on DELAY_30MS and will range from
		// -1/2 the value to 1/2 the value
		adjust = (DELAY_30MS/1.0)*(adc_num/512.0);

		SOLENOID_ON_TIME = DELAY_30MS + (int) adjust;

		// Wait a second before testing this new value
		delay_ms(1000);

		solenoid_on();
		beat_led_on();
		SOLENOID_COUNTER = SOLENOID_ON_TIME;

		// Enable ADC6
		ADMUX = _BV(MUX1) | _BV(MUX2);

		// Start new conversion
		ADCSRA |= _BV(ADSC);
	}
}

void pot_test (void) {
	int adc_num;

	while (1) {

		// See if there is an ADC conversion done
		if (ADCSRA & (_BV(ADIF))) {
			// Clear ADIF by writing a 1 (this sets the value to 0)
			ADCSRA |= _BV(ADIF);

			// Read the current value
			adc_num = ADC;

			m_all_off();
			if (adc_num < 255) {
				m1_led_on();
			} else if (adc_num < 511) {
				m2_led_on();
			} else if (adc_num < 767) {
				m3_led_on();
			} else {
				m4_led_on();
			}

			update_lights();

			// Enable ADC6
			ADMUX = _BV(MUX1) | _BV(MUX2);

			// Start new conversion
			ADCSRA |= _BV(ADSC);
		}
	}
}

void clear_beats(void) {
	int i;
	for (i=0; i<16; i++) {
		SET_BEATS[i] = 0;
	}
}

// Update the LEDs connected to the sink register
void update_lights (void) {
	//int mask = 32768;
	//int mask = 128;
	int mask = 512;
	for (int x = 0; x < 10; x++) {
		if (LIGHT_VECTOR & mask) {
			led_sdi_set();
		} else {
			led_sdi_clear();
		}
		led_clk_set();
		led_clk_clear();

		mask >>= 1;
	}
	led_le_set();
	led_le_clear();
}

void set_tempo (void) {
	if (is_tempo_3x()) {
		TEMPO_DELAY = TEMPO_3X_DELAY;
	} else if (is_tempo_2x()) {
		TEMPO_DELAY = TEMPO_2X_DELAY;
	} else if (is_tempo_1x()) {
		TEMPO_DELAY = TEMPO_1X_DELAY;
	} else {
		TEMPO_DELAY = TEMPO_1X_DELAY;
	}
}

//General short delays
void delay_us(int x) {
/*
	int y, z, a;
	
	y = x/256;
	z = x - y * 256;
	
	for (a = 0; a < y; a++)
	{
		TIFR2 |= 0x01;//Clear any interrupt flags on Timer2
		
		TCNT2 = 0; //256 - 125 = 131 : Preload timer 2 for x clicks. Should be 1us per click
	
		while(!(TIFR2 & 0x01));
		
	}
	
	TIFR2 |= 0x01;//Clear any interrupt flags on Timer2
	
	TCNT2 = 256-z; //256 - 125 = 131 : Preload timer 2 for x clicks. Should be 1us per click

	while(!(TIFR2 & 0x01));
*/
}

// General short delays
void delay_ms(uint16_t x) {
	for (; x > 0 ; x--) {
		TIFR2 |= 0x01;//Clear any interrupt flags on Timer2
		TCNT2 = 99; //256 - 157 = 99 : Preload timer 2 for 99 clicks == 1ms.
		while(!(TIFR2 & 0x01));
	}
/*
	for (; x > 0 ; x--) {
        delay_us(250);
        delay_us(250);
        delay_us(250);
        delay_us(250);
    }
*/
}

