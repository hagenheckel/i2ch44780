/**
 * @file i2ch44780.c
 * Contains the code for LCD operation via i2c bus and the H44780 controller
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

#include "i2ch44780.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

int fd;
state actstate;

/**
 * Initializes the Linux Devicefile for accessing the i2c bus.
 */
void dev_init(void){
	/// open device file
	char filename[20];

	snprintf(filename, 19, "/dev/i2c-%d", I2C_ADAPTER);
	fd = open(filename, O_RDWR);
	if (fd < 0) {
		perror("[FATAL:] Unable to open device-file.(open)");
		exit(1);
	}

	/// specify the device address
	if (ioctl( fd, I2C_SLAVE, I2C_ADDR) < 0) {
		perror("[FATAL:] Unable to contact device.(ioctl)");
 		exit(1);
	}
}

/**
 * Initialises the LCD display by sending commands to the HD44780 chip.
 * Data length is set to the 4 bit mode, number of lines is 2 (the other 2 are in the same 
 * address space) and the font size 8x5 dots. 
 *
 */
void lcd_init(void){
	lcd_write_cmd(0x33);	/*< Initialise */
	lcd_write_cmd(0x32);	/*< Initialise */
	lcd_write_cmd(0x06);	/*< Cursor move direction */
	lcd_write_cmd(0x0C);	/*< Blink Off (On: 0x0F) */
	lcd_write_cmd(0x28);	/*< Data length, number of lines, font size */
	lcd_write_cmd(0x01);	/*< Clear display */
	lcd_write_cmd(0x08);	/*< Backlight ON */
	actstate.display = 1;
	actstate.cursor = 1;
	actstate.backlight = 1;
	actstate.cx = 0;
	actstate.cy = 0;
	usleep(500);
} 

/** Write a command byte to the HD44780.
 *  As the HD44780 is connected to the I2C-Bus via a PCF8574 8bit-I/O Expander each command is split
 *  into a high byte and a low byte each 4 bit long. The 4 bits are assembled with the RS (register select), 
 *  R/W (read/write) and E (enable) bits and sent one after the other.
 *  Connection: 
 *		 PCF8547			HD44780
 *		
 *		P0				DB4
 *		P1				DB5
 *		P2				DB6
 *		P3				DB7
 *		P4				not connected
 *		P5				E
 *		P6				R/W
 *		P7				RS
 * @param[in] bits: 8bit command byte for the HD44780
 */
void lcd_write_cmd(int bits){
	int bits_high, bits_low;
	__s32 res;
	bits_high = bits & 0xF0;
	bits_low = (bits<<4) & 0xF0;
	res = i2c_smbus_write_byte(fd, bits_high);
	if( res<0 ){
		perror("Unable to write (cmd) bits_high.");
		exit(1);
	}
	lcd_enable(bits_high);
	res = i2c_smbus_write_byte(fd, bits_low);
	if( res<0 ){
		perror("Unable to write (cmd) bits_low.");
		exit(1);
	}
	lcd_enable(bits_low);
}

/** Writes a data byte to the HD44780.
 *  The only difference to lcd_write_cmd is, that the RS bit is set to 1.
 * @param[in] bits: 8bit data byte for the HD44780
 */
void lcd_write_data(int bits){
	int bits_high, bits_low;
	__s32 res;
	bits_high = (bits & 0xF0) | 0x01;
	bits_low = ((bits<<4) & 0xF0) | 0x01;
	res = i2c_smbus_write_byte(fd, bits_high);
	if( res<0 ){
		perror("Unable to write (data) bits_high.");
		exit(1);
	}
	lcd_enable(bits_high);
	res = i2c_smbus_write_byte(fd, bits_low);
	if( res<0 ){
		perror("Unable to write (data) bits_low.");
		exit(1);
	}
	lcd_enable(bits_low);
}

/** Toggle enable pin on LCD display.
 *  The ENABLE byte must be toggled HIGH/LOW to make the data appear on the display or to execute the command.
 *  @param[in] high byte or low byte */
void lcd_enable(int bits) {
	usleep(500);
	i2c_smbus_write_byte(fd, (bits | ENABLE));
	usleep(500);
	i2c_smbus_write_byte(fd, (bits | ~ENABLE));
	usleep(500);
}

/** Sets cursor to the indicated position.
 *  @param[in] row: start address of the associated row e.g. 0x80
 *  @param[in] column: starts with 0
 */
void cursor_to_pos(short row, short column){
	int pos;
	if (column < NO_OF_CLMS) 
		pos = row + column;
	else
		perror("Columns out of range.");
	lcd_write_cmd(pos);
}

/** Shifts cursor.
 * @param[in] steps number of columns the cursor shall be shifted
 * @param[in] rl direction of shift right 1, left 0
 */
void cursor_shift(int steps, int rl){
	if( rl == 1 ){
		for(int i; i<steps; i++){
			lcd_write_cmd(0b00010100);
		}
	}
	else if( rl == 0 ){
		for(int i; i<steps; i++){
			lcd_write_cmd(0b00010000);
		}
	}
}

/** Switches on the cursor.
 */
void cursor_on(void){
	lcd_write_cmd(0b00001110);
	actstate.cursor = 1;
}

/** Switches on the cursor and let him blink.
 */
void cursor_blink(void){
	lcd_write_cmd(0b00001111);
	actstate.cursor = 2;
}

/** Switches the cursor off
 */
void cursor_off(void){
	lcd_write_cmd(0b00001100);
	actstate.cursor = 0;
}

/** Returns the cursor to the home position, meaning the top line column 0.
 */
void cursor_home(void){
	lcd_write_cmd(0b00000010);
}

/** Switches the display on.
 */
void display_on(void){
	lcd_write_cmd(0b00001100);
	actstate.display = 1;
}

/** Switches the display off.
 */
void display_off(void){
	lcd_write_cmd(0b00001000);
	actstate.display = 0;
}

/** Switches the background light on.
 */
void background_on(void){
	lcd_write_cmd(0x08);
	actstate.backlight = 1;
}

/** Switches the background light off.
 */
void background_off(void){
	lcd_write_cmd(0x04);
	actstate.backlight = 0;
}

/** Clear LCD and return cursor home. 
 */
void lcd_clear(void){
	lcd_write_cmd(0x01);
	lcd_write_cmd(0x02);
}

/** Print one character to LCD at current position.
 *  @param[in] val character
 */
void type_char(char val){
	lcd_write_data(val);
}

/** Print a string to LCD at current position 
 * @param[in] s pointer to string
 */
void type_string(const char *s){
	while (*s) lcd_write_data(*(s++));
}

/** Returns wether the display is on or off
 * @param[out] ON:1 OFF:0
 */
unsigned int get_display_state(void){
	return actstate.display;
}

/** Returns wether the cursor-state
 * @param[out] OFF:0 ON:1 BLINK:2
 */
unsigned int get_cursor_state(void){
	return actstate.cursor;
}
