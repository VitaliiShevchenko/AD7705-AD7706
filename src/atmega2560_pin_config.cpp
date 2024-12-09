#include "atmega2560_pin_config.h"

void initializePinsForAD7705() {

  // initialize OUTPUT pins
  pinMode(AD7705__CS1,     OUTPUT);
  pinMode(AD7705_SCLK1,    OUTPUT);
  pinMode(AD7705_DIN1,     OUTPUT);
  pinMode(AD7705__RESET1,  OUTPUT);

  pinMode(AD7705__CS2,     OUTPUT);
  pinMode(AD7705_SCLK2,    OUTPUT);
  pinMode(AD7705_DIN2,     OUTPUT);
  pinMode(AD7705__RESET2,  OUTPUT);

  pinMode(AD7705__CS3,     OUTPUT);
  pinMode(AD7705_SCLK3,    OUTPUT);
  pinMode(AD7705_DIN3,     OUTPUT);
  pinMode(AD7705__RESET3,  OUTPUT);


  // initialize INPUT pins
  pinMode(AD7705__DRDY1,  INPUT);
  // pinMode(AD7705_DOUT1,   INPUT);

  pinMode(AD7705__DRDY2,  INPUT);
  pinMode(AD7705_DOUT2,   INPUT);

  pinMode(AD7705__DRDY3,  INPUT);
  pinMode(AD7705_DOUT3,   INPUT);

  
}
