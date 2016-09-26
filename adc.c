/*
 * adc.c
 *
 *  Created on: 21/08/2012
 *      Author: M. J. Cree
 *      Modified: D. O. Corlett
 *
 *  Modified 8/8/2013 to indicate 8-bit ADC read is broken.
 *
 *  Modified 6/09/16	to display the 8-bit approximation
 *  of the analogue input pin A0 on an LCD, by: D. O. Corlett
 */

#include "lcd.h"
#include <avr/io.h>
#include "adc.h"
#include "my_lcd.h"
#include <util/delay.h>
#include <stdio.h>
#include <math.h>		// log() <-- natural log (ln())




/*
 * adc_init()
 *
 * Power up the ADC hardware and select the ADC input pin.
 * pin must be set to a value listed in Table 23-4 of the ATmega manual
 */
void adc_init(int pin)
{
	pin &= 0xf;

	/* Make sure the ADC circuit is powered up */
	PRR &= ~(1<<PRADC);		// Disable Power Reduction ADC bit

	/* Turn off GPIO on the specified analogue input pin */
	if (pin <= 7) {
		/* Only the first eight inputs correspond to GPIO on Port C */
		DIDR0 = 1<<pin;		// Disable digital input on the pin to be used as ADC.
	}

	/* Set up ADC control registers ready for conversions */
        /* Enable ADC and set the lowest bit clock rate. */
	ADCSRA = (1<<ADEN) | 0x07;

	/* Unused so lets just set ADCSRB to 0 */
	ADCSRB = 0;

	/* Select ADC input source, and reference. */
	if (pin == 8) {
		/* Voltage ref is 1.1V for CPU temperature sensor */
		ADMUX = (1<<REFS1) | (1<<REFS0) | pin;
	}else{
		/* Voltage ref is AVcc (5V) for all other inputs */
		ADMUX = (1<<REFS0) | pin;
	}
}


/*
 * adc_read()
 *
 * Perform an analogue to digital conversion and return
 * the converted value.
 * Input source is specified by calling adc_init() first.
 */

int adc_read(void)
{
	int val = 0;				// Initalise ADC value to 0.

	ADCSRA |= 1<<ADSC; 			//Starts conversion (ADSC = 1).

	// While conversion is still in progress:
	while ( ADCSRA & (1 << ADSC) )
	{
		/* do nothing */
	}

	/* Do a 10-bit conversion:	*/
	if ( ADMUX & (1 << ADLAR) )
	{
		// if ADLAR == 1:
		val = ADCL>>6;		// Do a 10-bit conversion by combining ADCL and ADCH making sure to read ADCL before ADCH.
		val |= ADCH<<2;
	}
	else
	{
		// if ADLAR == 0:
		val = ADCL;			// Do a 10-bit conversion by combining ADCL and ADCH making sure to read ADCL before ADCH.
		val |= ADCH<<8;
	}

	return val;
}

/* Calculate the temperature in K given an adc value of a thermistor voltage divider circuit*/
float adcToTempC(int A0)
{
	int Vmax = 5;
	int ADCmax = 1023;
	int R1 = 4700;
	int Rref = 4700;

	float Va = ((A0*Vmax)/ADCmax);
	float R2 = (Va*R1)/(Vmax*(1-(Va/Vmax)));

	float A1 = 0.003354016;					// 3.354016E-3
	float B1 = 0.0002569355;				// 2.569355E-4 K^-1
	float C1 = 0.000002626311;				// 2.626311E-6 K^-2
	float D1 = 0.0000000675278;				// 0.675278E-7 K^-3

	float R2Rref = R2/Rref;
	float lnR2Rref = log(R2Rref);
	float temp = 1/(A1 + B1*lnR2Rref + C1*lnR2Rref*lnR2Rref + D1*lnR2Rref*lnR2Rref*lnR2Rref);

	return temp - 273.15;	// + 0.5;			// add 0.5 to round correctly
}

/* converts a number of type double
 * to an integer with 4 decimal places.*/
char * floatTostring(float FloatPntNum)
{
	char str[100];

	int IntPart = (int)FloatPntNum;
	int FloatPart = (int)((FloatPntNum - IntPart)*1000);

	snprintf (str, 100, "%i.%i", IntPart, FloatPart);

	return str;
}

int main()
{
	adc_init(0);		// Initalise pin 0 on portC for ADC read.
	//my_lcd_init(4);		// Initalise 4 bit LCD on port D
	lcd_init(LCD_CTRL_PORT_C | LCD_DATA_PORT_D);


	char con[33];		// Define display string

	int A0;
	while(1)
	{
		A0 = adc_read();
		snprintf( con, 33, "%s%i%c%s%s", "A0 = ", A0, '\n',"R2: = ", floatTostring(adcToTempC(A0)) );		// Create a string with the A0 ADC value.
		//my_lcd_display(con);																// Display A0 ADC value on LCD.

		lcd_clear();
		lcd_display(con);

		_delay_ms(100);
	}

	return 0;
}
