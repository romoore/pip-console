/*****************************************************************************************
 * Hand-coded SPI interface between the Raspberry Pi and the B-PIP
 *
 * Written by Bernhard Firner (c) 2014
 ****************************************************************************************/

#ifndef __SPI_H__
#define __SPI_H__

//Include GPIO and SPI library for the Pi
#include <bcm2835.h>

//Bit banging SPI code

//GPIO 10 is MOSI, pin is 19
//GPIO  9 is MISO, pin is 21
//GPIO 11 is SCLK, pin is 23
#define MOSI RPI_GPIO_P1_19
#define MISO RPI_GPIO_P1_21
#define SCLK RPI_GPIO_P1_23

#define REQ_DROPPED 0xFC
#define REQ_REALIGN 0xFD
#define REQ_PACKET 0xFE
#define REQ_NULL 0x1

//The SPI rate in microseconds
//#define SPI_RATE 60
#define SPI_RATE 100

void setupSPI();

void tearDownSPI();

//Sends and reads a bit, delaying clock_period/2 milliseconds after writing and again after reading
//Going to change data on falling edges, so read on rising edges
uint8_t exchangeBit(uint8_t value, unsigned int clock_period);

uint8_t exchangeByte(uint8_t byte, unsigned int clock_period);

#endif //__SPI_H__

