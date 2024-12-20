/*
  This file is part of the ModuleAD7705 library.
  Copyright (c) 2024 EXCEL LLC (VitaliiShevchenko). All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

#include "atmega2560_pin_config.h"
#include <stdint.h>
#include <SPI.h>

// Instruction See page 16 of doc by https://www.analog.com/media/en/technical-documentation/data-sheets/AD7705_7706.pdf
// COMMUNUCATON REGISTER
#define ZERO_DRDY_7       0x80
#define COMM_REG_456      0x00
#define SETUP_REG_456     0x10
#define CLOCK_REG_456     0x20
#define DATA_REG_456      0x30
#define TEST_REG_456      0x40
#define OFFSET_REG_456    0x60
#define GAIN_REG_456      0x70
#define READ_3            0x08
#define WRITE_3           0x00
#define NORMAL_OP_MODE_2  0x00
#define POWER_DOWN_MODE_2 0x04
//                                           AD7705        |     AD7706
#define CH0_01            0x00 //       (AIN1+) + (AIN1-)  |   AIN1 + COMMON
#define CH1_01            0x01 //       (AIN2+) + (AIN2-)  |   AIN2 + COMMON
#define CH2_01            0x02 //       (AIN1-) + (AIN1-)  |   COMMON
#define CH3_01            0x03 //       (AIN1-) + (AIN2-)  |   AIN3 + COMMON

enum channel {
  ONE = CH0_01,                //       [Register Pair 0]
  TWO = CH1_01,                //       [Register Pair 1]
  COMMON = CH2_01,             //       [Register Pair 0]
  THREE = CH3_01               //       [Register Pair 2]
};

// SETUP REGISTER. The numbers in the end of the names indicate on the bits of the register.
#define STRG_MD_NORMAL_MODE_76     0x00
#define STRG_MD_SELF_CALIBR_76     0x40
#define STRG_MD_ZERO_CALIBR_76     0x80
#define STRG_MD_FULL_CALIBR_76     0xC0
#define STRG_GAIN1_543             0x00
#define STRG_GAIN2_543             0x08
#define STRG_GAIN4_543             0x10
#define STRG_GAIN8_543             0x18
#define STRG_GAIN16_543            0x20
#define STRG_GAIN32_543            0x28
#define STRG_GAIN64_543            0x30
#define STRG_GAIN128_543           0x38
#define STRG_BIPOLAR_OPERATION_2   0x00
#define STRG_UNIPOLAR_OPERATION_2  0x04
#define STRG_BUFFER_ENABLE_1       0x02
#define STRG_FSYNC_IN_RESET_STATE  0x01

enum calibration{
  NORMAL_MODE = STRG_MD_NORMAL_MODE_76,
  SELF_CALIBR = STRG_MD_SELF_CALIBR_76,
  ZERO_CALIBR = STRG_MD_ZERO_CALIBR_76,
  FULL_CALIBR = STRG_MD_FULL_CALIBR_76
};

// CLOCK REGISTER
#define CLRG_FOR_CORRECT_OP_765     0x00
#define CLRG_MASTER_CLOCK_DISABLE_4 0x10
#define CLRG_CLOCK_DIVIDER_BY2_3    0x08  //CLKDIV
#define CLRG_CLOCK_BIT_2            0x04  //Clock Bit. This bit should be set in accordance with the operating frequency of the AD7705/AD7706. 
                                     /*If the device has a master clock frequency of 2.4576 MHz (CLKDIV = 0) or 4.9152 MHz (CLKDIV = 1), 
                                     this bit should be set to Logic 1. If the device has a master clock frequency of 1 MHz (CLKDIV = 0) 
                                     or 2 MHz (CLKDIV = 1), this bit should be set to Logic 0. This bit sets up the appropriate scaling currents 
                                     for a given operating frequency and, together with FS1 and FS0, chooses the output update rate for the device. 
                                     If this bit is not set correctly for the master clock frequency of the device, the AD7705/AD7706 might not 
                                     operate to specification.*/
//CLK1 FS1 FS0 [Output Update Rate] [−3 dB Filter Cutoff](last value)
#define     CLRG_FS_CLK_IS_LOW_20Hz_5Hz 0x00
#define     CLRG_FS_CLK_IS_LOW_25Hz_7Hz 0x01
#define   CLRG_FS_CLK_IS_LOW_100Hz_26Hz 0x02
#define   CLRG_FS_CLK_IS_LOW_200Hz_52Hz 0x03
#define   CLRG_FS_CLK_IS_HIGH_50Hz_13Hz 0x00
#define   CLRG_FS_CLK_IS_HIGH_60Hz_16Hz 0x01
#define  CLRG_FS_CLK_IS_HIGH_250Hz_66Hz 0x02
#define CLRG_FS_CLK_IS_HIGH_500Hz_131Hz 0x03

enum fregADC {
  ADC_1MHZ    = CLRG_CLOCK_DIVIDER_BY2_3 & 0 | CLRG_CLOCK_BIT_2 & 0,
  ADC_2MHz    =     CLRG_CLOCK_DIVIDER_BY2_3 | CLRG_CLOCK_BIT_2 & 0,
  ADC_2457kHZ = CLRG_CLOCK_DIVIDER_BY2_3 & 0 | CLRG_CLOCK_BIT_2,
  ADC_4915kHZ =     CLRG_CLOCK_DIVIDER_BY2_3 | CLRG_CLOCK_BIT_2
};

enum outputUpdateRate{
  OUR_20_50HZ   = 0x00,
  OUR_25_60HZ   = 0x01,
  OUR_100_250HZ = 0x02,
  OUR_250_500HZ = 0x03
};

enum polar{
  UNIPOLAR = STRG_UNIPOLAR_OPERATION_2,
  BIPOLAR  = STRG_BIPOLAR_OPERATION_2
};

enum buffer{
  BUFFER_ENABLE = STRG_BUFFER_ENABLE_1,
  BUFFER_DISABLE = STRG_BUFFER_ENABLE_1 & 0
};  

#define CHANNEL_MASK  0x03
#define FREG_ADC_MASK 0x06
#define CALIBR_MASK   0xC0
#define GAIN_MASK     0x38
#define OUR_MASK      0x03
#define POLAR_MASK    0x04
#define BUFFER_MASK   0x02

class ModuleAD7705 {

public:
  ModuleAD7705(uint8_t cs_pin, uint8_t reset_pin, uint8_t drdy_pin); // Existing constructor
  ModuleAD7705(); // Default constructor

  /**
   * Performs a standart initialization for AD7705/AD7706 where frequency of oscillator is 4.91512 MHz.
   *
   * @return void
   */
  void std_init();

    /**
   * Performs customizing initialization for AD7705/AD7706 with specifics peculiarities .
   * 
   * @param fregADC choose frequency of oscillator using keywords: ADC_1MHZ, ADC_2MHz, ADC_2457kHZ, ADC_4915kHZ
   * @param output_rate choose output update rate using keywords: OUR_20_50HZ, OUR_25_60HZ, OUR_100_250HZ, OUR_250_500HZ
   * @param calibr choose calibration using keywords: NORMAL_MODE, SELF_CALIBR, ZERO_CALIBR, FULL_CALIBR
   * @param gain choose gain using keywords: STRG_GAIN[1,2,4,8,16,64,128]_543 
   * @param uni_bipolar choose UNIPOLAR or BIPOLAR operation
   * @param buf_state choose BUFFER_ENABLE or BUFFER_DISABLE
   *
   * @return void
   */
  void custom_init( uint8_t fregADC_Hz, uint8_t output_rate, uint8_t calibr, uint8_t gain, uint8_t uni_bipolar, uint8_t buf_state);

  /**
   * Perform a "RESET ADC" operation for AD7705/AD7706.
   *
   * @return void
   */
  void reset_adc();

  /**
   * Perform a "Read Data Ready status (DRDY)" operation for the specified id
   *
   * @return recieved data
   */
  bool isDataReady();

   /**
   * Perform a "Read data from the Serial pin DOUT of AD7705/AD7706 by byte"
   * 
   * @param bytes amount bytes for reading.Using such keywords: ONE_BYTE, TWO_BYTES, THREE_BYTES
   *
   * @return recieved data
   */
  int ModuleAD7705::read_serial_data_byByte(size_t bytes);

   /**
   * Perform a "Write 24-bit data to the Serial pin DIN of AD7705/AD7706"
   *
   * @return void
   */
  int write_register_24(uint32_t data);

   /**
   * Reading data from the Data Register of the determined AD7705/AD7706 channel
   * 
   * @param channel - choose channel. Use one from of such: ONE, TWO, THREE
   *
   * @return 16-bit value of the choose channel
   */
  int read_channel(uint8_t channel);

   /**
   * Reading data from the Clock Register of the determined AD7705/AD7706 channel
   * 
   * @param channel - choose channel. Use one from of such: ONE, TWO, THREE
   *
   * @return 8-bit value of the choose channel
   */
  int read_clock_channel(uint8_t channel);

     /**
   * Reading data from the Setup Register of the determined AD7705/AD7706 channel
   * 
   * @param channel - choose channel. Use one from of such: ONE, TWO, THREE
   *
   * @return 8-bit value of the choose channel
   */
  int read_setup_channel(uint8_t channel);


  /**
   * Reading data from the Offset Register of the determined AD7705/AD7706 channel
   * 
   * @param channel - choose channel. Use one from of such: ONE, TWO, THREE
   *
   * @return 24-bit value of the choose channel
   */
  int ModuleAD7705::read_offset_channel(uint8_t channel);

  /**
   * Reading data from the Gain Register of the determined AD7705/AD7706 channel
   * 
   * @param channel - choose channel. Use one from of such: ONE, TWO, THREE
   *
   * @return 24-bit value of the choose channel
   */
  int ModuleAD7705::read_gain_channel(uint8_t channel);


   /**
   * Define external function by using setter
   * This feature alowed to use this module for another programming microcontrollers
   *
   * @param func external function, for instance digitalWrite in the Arduino
   *
   * @return void
   */
  void setFuncDigitalWrite(int (*func)(int, int));

   /**
   * Define external function  by using setter
   * This feature alowed to use this module for another programming microcontrollers
   *
   * @param func external function, for instance digitalRead in the Arduino
   *
   * @return void
   */
  void setFuncDigitalRead(int (*func)(int));

   /**
   * Call external function
   *
   * @param pin the number of necessary pin
   * @param val the value (LOW or HIGH)
   *
   * @return 0 if success, -1 if wrong
   */
  int digitalWrite(uint8_t pin, bool val);

     /**
   * Call external function
   *
   * @param pin the real number of necessary pin
   *
   * @return boolean value of the pin
   */
  bool digitalRead(uint8_t pin);

  /**
   * Perform a "Write specified AD7705's register". See page 16 of doc by https://www.analog.com/media/en/technical-documentation/data-sheets/AD7705_7706.pdf
   *
   * @param instruction a 8-bit command for managing the states of AD7705/AD7706
   *
   * @return void
   */
  void write_register(uint8_t instruction);

    /**
     * Public methods which print data to the external environment
     */
    void print(char txt[]){
      Serial.print(txt);
    }
    void print(int num){
      Serial.print(num);
    }
    void println(char txt[]){
      Serial.println(txt);
    }
    void println(int num){
      Serial.println(num);
    }

    void   select_adc();
    void unselect_adc();

    /**
     * Function for set maximum range of measuring in volts
     */
    void set_max_range_in_volts(float volts);

    /**
     * Function for converting bit-value into millivolts
     */
    float val_into_millivolts(float data);


    /**
     * Function for set maximum range of measuring in mA
     */
    void set_max_range_in_mA(float mA);

    /**
     * Function for converting bit-value into mA
     */
    float val_into_mA(float data);


    /**
     * Function for set maximum range of measuring in kgs
     */
    void set_max_range_in_kgs(float kgs);

    /**
     * Function for converting bit-value into kgs
     */
    float val_into_kgs(float data);


    /**
     * Function for set maximum range of measuring in Nm
     */
    void set_max_range_in_Nm(float Nm);

    /**
     * Function for converting bit-value into Nm
     */
    float val_into_Nm(float data);

    
    /**
     * Function for set zero
     */
    void set_zero(int zero_value);

private:
    uint8_t _cs, _reset, _drdy;
    float _max_range_in_volts;
    float _max_range_in_mA;
    float _max_range_in_kgs;
    float _max_range_in_Nm;
    int   _zero;


    /*
    * Setter for the external function for Writing Digital Pin by microcontroller
    */
    int (*writeDigitalPin)(int, int);

        /*
    * Setter for the external function for Reading Digital Pin by microcontroller
    */
    bool (*readDigitalPin)(int);

    /**
    * Private method which wait to set pin _DRDY of AD7705 | AD7706 to LOW level
    *
    * @return 0 if successed and -1 if appear a runtime error
    */
    int ModuleAD7705::waitingOnDataReady();
};