#include <ModuleAD7705.h>
#include <atmega2560_pin_config.h>

// Perform connect chip. Pins SPI Bus must have to connect in accordance of board.
// For example, board ATMega 2560 have such pins of SPI Bus: MOSI(DIN) = 51, MISO(DOUT) = 50 and SCK(SCLK) = 52

int data;

ModuleAD7705 AD7705_idOne(AD7705__CS1, AD7705__RESET1, AD7705__DRDY1);

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // are defined necessary functions for writing and reading logic levels of the microcontroller board pins
  AD7705_idOne.setFuncDigitalWrite(digitalWrite);
  AD7705_idOne.setFuncDigitalRead(digitalRead);
  
  AD7705_idOne.set_max_range_in_volts(1.25);
  AD7705_idOne.set_max_range_in_kgs(485);
  AD7705_idOne.set_max_range_in_Nm(152.917);
  AD7705_idOne.set_zero(0);

  AD7705_idOne.custom_init(ADC_4915kHZ, OUR_20_50HZ, ZERO_CALIBR, STRG_GAIN16_543, UNIPOLAR, BUFFER_DISABLE);
}

// the loop routine runs over and over again forever:
void loop() {
    
    data = AD7705_idOne.read_channel(ONE);

    Serial.println(data);

    Serial.print(" Volt, mV = ");
    Serial.println( AD7705_idOne.val_into_millivolts(data), 4 );

    Serial.print(" Weight, kgs = ");
    Serial.println(AD7705_idOne.val_into_kgs(data), 4);

    Serial.print(" Moment, Nm = ");
    Serial.println(AD7705_idOne.val_into_Nm(data), 4);
    delay(1000);
   
}
