/*
 *  Module: my_lcd
 *  Author: M. J. Cree,
 *  Modified by D. O. Corlett,
 *
 *  (C) 2009-2014 The University of Waikato
 *
 *  Routines to interface to the 16x2 text LCD display.
 *  Modified to use D. O. Corlett's functions for
 *  displaying text to 16x2 LCD display
 */

#ifndef _LIBRARY_LCD_H_
#define _LIBRARY_LCD_H_

#include <stdio.h>

/* 	includes:
#include <util/delay.h>
#include <avr/io.h>
#include <util/delay_basic.h>
#include "my_lcd.h"

 	 defines:
#define LCD_RS 0x20
#define LCD_RW 0x10
#define LCD_EN 0x08
*/

// Initialise the LCD for Control on portC and Data on portD for 8 bit and 4 bit mode depending on the argument (8 or 4).
extern void my_lcd_init(int bit);
// Clears the LCD.
extern void my_lcd_clear(void);
// Displays the argument string of characters on the LCD.
extern void my_lcd_display(char string[]);
// Displays a string of words on the LCD without
// splitting words between a line on the display.
extern void my_lcd_display_AutoWrap(int stringSize, char string[]);


#endif
