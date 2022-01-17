# Makefile for producing library and documentation for operating the
# H44780 controller via the i2c bus
#
#    Copyright (c) 2022 Hagen Heckel
####################################################################
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as
#    published by the Free Software Foundation, either version 3 of the
#    License, or (at your option) any later version.
#
#    It is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public
#    License along with wiringPi.
#    If not, see <http://www.gnu.org/licenses/>.
#####################################################################

all: i2ch44780.c
	gcc -c i2ch44780.c -o i2ch44780.o
	ar -rcs libi2ch44780.a i2ch44780.o
	doxygen Doxyfile
