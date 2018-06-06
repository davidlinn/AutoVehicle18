/*
 * LCD2.cpp
 *
 *  Created on: Jun 1, 2018
 *      Author: David Linn
 */

#include "LCD.h"
#include <iosys.h>
#include <serial.h>

LCD::LCD() :
fd(0)
{ }

LCD::~LCD() { }

LCD::LCD(int serialPort, int baud) { //Initialize an LCD object with a serial file descriptor
	fd = SimpleOpenSerial(serialPort, baud);
}

void LCD::moveCursor(int x, int y) {
	if((x>15) || (y>1)) return;
	uint8_t b[2];
	b[0]=254;
	b[1]=128+x+64*y;
	write(fd,(char *)b,2);
}

void LCD::clear() {
	 moveCursor(0,0);
	 write(fd,"                    ",16);
	 write(fd,"                    ",16);
	 moveCursor(0,0);
}

void LCD::print(const char* buf) {//prints buffer to screen starting from cursor position
							//writes all chars of buffer to screen
	writestring(fd, buf);
}

void LCD::print(const char* buf, int nchars) { //optional arg nchars only prints the first nchars
										//characters to screen
	write(fd, buf, nchars);
}
