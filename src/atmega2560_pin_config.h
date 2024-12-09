#pragma once

#include <Arduino.h>

// Define pins of arduino as like name of AD7705 pins
#define    AD7705__CS1     36
#define    AD7705_SCLK1    37
#define    AD7705_DIN1     51
#define    AD7705__RESET1  33
#define    AD7705__DRDY1   32
#define    AD7705_DOUT1    50

#define    AD7705__CS2     47
#define    AD7705_SCLK2    46
#define    AD7705_DIN2     45
#define    AD7705__RESET2  44
#define    AD7705__DRDY2   43
#define    AD7705_DOUT2    42

#define    AD7705__CS3     41
#define    AD7705_SCLK3    40
#define    AD7705_DIN3     39
#define    AD7705__RESET3  38
#define    AD7705__DRDY3   37
#define    AD7705_DOUT3    36

// Function declaration for pin initialization
void initializePinsForAD7705();