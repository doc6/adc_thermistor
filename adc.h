/*
 * adc.h
 *
 * Functions for interfacing to the analogue to digital converter (ADC).
 *
 * By M. J. Cree
 */

/*
 * Call adc_init(pin) to power up the ADC circuitry and initialise
 * which input source to use.  Pin should be a value between 0 and 15
 * inclusive as specified in Table 23-4 of the ATmega CPU hardware
 * manual.
 *
 * Then call adc_read() to perform ADC conversions.  Returns an int
 * that is the converted value.
 *
 * If you want to change the input source call adc_init() again with
 * the new pin number.
 */

extern void adc_init(int pin);
extern int adc_read(void);
