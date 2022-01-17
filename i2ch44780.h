/**
 * @file i2ch44780.h
 * Contains the header for LCD operation via i2c bus and H44780 controller
 *
 *    Copyright (c) 2022 Hagen Heckel
 *
 ***********************************************************************
 *    This is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    It is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#ifndef I2CH44780_H
#define I2CH44780_H

#define I2C_ADAPTER	1		///< RPi has 0 or 1, usually 1
#define I2C_ADDR	0x27		
#define NO_OF_ROWS	4
#define NO_OF_CLMS	20
#define LINE1		0x80		///< Start address of 1st row
#define LINE2		0xC0
#define LINE3		0x94		///< 3rd row = 1st row + 0x14
#define LINE4		0xD4		///< 4th row = 2nd row + 0x14
#define ENABLE		0b00000100

/// Struct that holds state variables
typedef struct {
	unsigned int display:1;		///< display:  1 ON or 0 OFF
	unsigned int backlight:1;	///< backlight: 1 ON or 0 OFF
	unsigned int cursor:2;		///< cursor 0 OFF, 1 ON, 2 BLINK 
	unsigned int cx;		///< column of cursor position starting with 0
	unsigned int cy;		///< address of row of cursor position
} state;

extern state actstate;				///< holds the actual state values
extern int fd;					///< global filedescriptor of device file

void dev_init(void);
void lcd_init(void);
void lcd_write_cmd(int);
void lcd_write_data(int);
void lcd_enable(int);
void cursor_to_pos(short, short);
void cursor_shift(int, int);
void cursor_on(void);
void cursor_blink(void);
void cursor_off(void);
void cursor_home(void);
void display_on(void);
void display_off(void);
void background_on(void);
void background_off(void);
void lcd_clear(void);
void type_char(char);
void type_string(const char*);
unsigned int get_display_state(void);
unsigned int get_cursor_state(void);

#endif
