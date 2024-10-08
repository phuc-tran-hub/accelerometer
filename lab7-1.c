/* Taking an accelerometer's X and Y values to calculate and display the tilt value
 * 
 * Name:			Phuc Tran
 * Assignment:		Engs 28 22W
 *
 * Program name:	lab7.c
 * Date created:	23 February 2022
 *
 * I/O pins:  (what ports are used, and what are they wired to?)
 * 
 * Digital Inputs:
 * 1. Bit 0 of I/O Port B: Left LED
 * 2. Bit 1 of I/O Port B: Top LED
 * 3. Bit 2 of I/O Port B: Right LED
 * 4. Bit 3 of I/O Port B: Bottom LED
 * 5. SDA/SCL: SevenSeg, LSM303AGR
 * 
 * Target device:	ATmega328p / Arduino UNO
 * Dependencies:	ioE28.c, USARTE28.c, SevenSeg.c, i2c.c, lsm303agr.c
 */

/* INCLUDE FILES */
#include <avr/io.h>				// All the port definitions are here
#include <util/delay.h>			// For the _delay_ms macro
#include <i2c.h>
#include <lsm303agr.h>			// Holds accelerometer's driver functions
#include <USARTE28.h>   		// UART initializations
#include <ioE28.h>      		// tiny_printf as well as some reading
#include <math.h>				// for trig functions
#include <SevenSeg.h>			// SevenSeg library
#include <avr/interrupt.h> 		// Interrupt library

/* FUNCTIONS */
void timer1_init(uint16_t timeout);
void led_init(void);
void turn_led_orientation(int16_t angleDisplayed);
void check_negative(int16_t angleDisplayed, uint16_t *display_buffer);

/* CONSTANTS */
const uint16_t bitsDecimal = 32768;
const uint16_t full_range = 2;	// g
volatile uint8_t timerFlag;


/* CODE */
int main(void)
{

	// Storing SevenSeg buffer array
	uint16_t display_buffer[HT16K33_NBUF];

	// Storing accelerometer's calculated numbers
	float accel_x, accel_y;
    float radiansToDegree = 180/M_PI;
    float angleTheta;
    int16_t angleDisplayed;
	int16_t timeout = 62500;

	printf("LSM303AGR test program \n\r");

    // initialize the LEDs' ports
    led_init();
	// initialize timer interrupt
	timer1_init(timeout);
    // Initialize i2c communincation
	i2cInit();
    // Initialize USART communication
	USART_Init();
    // Initialize SevenSeg communication
    SevenSeg_init();
	// enable global interrupt
	sei();
	
	if( lsm303_AccelInit() ) {
		printf("Accelerometer initialized! \n\r");
	}
	else {
		printf("Could not connect to accelerometer \n\r");
	}
  	
  	printf("Control Register 1 = %x \n\r", lsm303_AccelRegisterRead(LSM303_CTRL_REG1_A));
  	printf("Control Register 4 = %x \n\r", lsm303_AccelRegisterRead(LSM303_CTRL_REG4_A));

	// Main code (runs in an infinite loop) goes here:
	lsm303AccelData_s accel_raw;
	
	// printf("accel_x, accel_y, accel_z \n");
	while(1) {
		if (timerFlag){
			lsm303_AccelReadRaw(&accel_raw);

			// Calculating the acceleleration x's, y's and z's in g
			accel_x = ((float) accel_raw.x) * full_range / bitsDecimal;                   // 32768 = 2^15
			accel_y = ((float) accel_raw.y) * full_range / bitsDecimal;        

			// Calculating angle as well as converting it, so it is printable (int16_t only)
			angleTheta = atan2(accel_x, accel_y) * radiansToDegree;
			angleDisplayed = (int16_t) round(angleTheta);
			printf("tilt = %d\n\r", angleDisplayed);
		
			// SevenSeg communication
			SevenSeg_number((int16_t)(fabs(angleTheta)), display_buffer);
			check_negative(angleDisplayed, display_buffer);
			SevenSeg_write(display_buffer);
			
			// find LED orientation and show that on the breadboard
			turn_led_orientation(angleDisplayed);
			timerFlag = 0;
		}
	}

	return 0;		/* never reached */
}

// ADC timer flag vector
ISR(TIMER1_COMPA_vect){
	printf("hello");
	timerFlag = 1;
}

void led_init(){
    // Set bit 0, 1, 2, 3 of Port B as outputs (all the LEDs)
    DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << PORTB2) | (1 << PORTB3);
}

// Setting up Timer 0 configuration for CTC, 125Hz (0.008s) in 1024 prescaler
void timer1_init(uint16_t timeout){

	// Setting to CTC: "Clear Timer on Compare"
	TCCR1B |= (1 << WGM12);
	TIMSK1 |= (1 << OCIE1A);
	OCR1A = timeout;
	TCCR1B |= (1 << CS12);
}

void turn_led_orientation(int16_t angleDisplayed){

	// Angle boundaries
	int16_t topRight = 45;
	int16_t botRight = 135;
	int16_t topLeft = -45;
	int16_t botLeft = -135;

	int16_t leftBound = -180;
	int16_t rightBound = 180;

	// turn off all LEDs
    const uint8_t led_mask = (1 << PORTB0) | (1 << PORTB1) | (1 << PORTB2) | (1 << PORTB3);
	PORTB &= ~led_mask;
	// Top quadrant: first portrait orientation
	if (angleDisplayed <= topRight && angleDisplayed > topLeft){
		PORTB |= (1 << PORTB1);	
	}
	// Right quadrant: right landscape orientation
	else if (angleDisplayed <= botRight && angleDisplayed > topRight){
		PORTB |= (1 << PORTB0);	
	}
	// Left quadrant: left landscape orientation
	else if (angleDisplayed <= topLeft && angleDisplayed > botLeft){
		PORTB |= (1 << PORTB2);	
	}
	// Bot quadrant: bottom portrait orientation
	else if ((angleDisplayed <= botLeft && angleDisplayed >= leftBound) ||  (angleDisplayed > botRight && angleDisplayed <= rightBound)){
		PORTB |= (1 << PORTB3);	
	}

}

void check_negative(int16_t angleDisplayed, uint16_t *display_buffer){
	// If it's negative 
	if (angleDisplayed < 0){
		int16_t negative = numbertable[10];
		// update display buffer
		display_buffer[0] = negative;
	} else{
		display_buffer[0] = 0;
	}
}
