/*
 * LCD2.h
 *  For use with Netburner NNDK v2.8
 *  Created on: Jun 1, 2018
 *      Author: David Linn
 */

#ifndef LCD_H_
#define LCD_H_

class LCD {
public:
	LCD();
	LCD(int serialPort,int baud); //Instatiate LCD object by passing in serial port number and baud rate
					//Constructor opens serial port using SimpleOpenSerial()
	virtual ~LCD();
	void moveCursor(int x, int y); //Moves cursor to position (x,y)
								//where 0<=x<=15 and 0<=y<=1
	void clear(); //Clears screen
	void print(const char* buf); //prints buffer to screen starting from cursor position
						//writes all chars of buffer to screen
	void print(const char* buf, int nchars); //optional arg nchars only prints the first nchars
										//characters to screen

private:
	int fd; //File descriptor returned by call to SimpleOpenSerial()
};

#endif /* LCD_H_ */
