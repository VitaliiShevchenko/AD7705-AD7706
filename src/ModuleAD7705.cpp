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

#include "ModuleAD7705.h"

#define LOW  0x00
#define HIGH 0x01

#define CYCLES_LIMIT int     (320000)                    // 1s / 50tacts * 16000000Hz = 320000

#define DELAY_BY_RESET        50

#define VOLTS_2_MILLIVOLTS(value)           value * 1000  
#define ADC_BITRATE                        4096  

enum {
    ONE_BYTE = 1,
    TWO_BYTES = 2,
    THREE_BYTES = 3
};

ModuleAD7705::ModuleAD7705(uint8_t cs_pin, uint8_t reset_pin, uint8_t drdy_pin):
    _cs(cs_pin),
    _reset(reset_pin),
    _drdy(drdy_pin)
{
    // initialize OUTPUT pins
    pinMode(_cs,     OUTPUT);
    pinMode(_reset,  OUTPUT);
    
    // initialize INPUT pins
    pinMode(_drdy,    INPUT);
}


void ModuleAD7705::std_init()   
{
    // Write to the CLOCK REGISTER with oscilator = 4.9152 MHz, output update rate is 50 Hz and −3 dB Filter Cutoff is 13 Hz
    // Write to the SETUP REGISTER: self-calibration, unipolar mode(0 to 5v) and buffer disable
    custom_init( ADC_4915kHZ, OUR_20_50HZ, SELF_CALIBR, STRG_GAIN1_543, UNIPOLAR, BUFFER_ENABLE );
}

void ModuleAD7705::custom_init( uint8_t fregADC_Hz, uint8_t output_rate, uint8_t calibr, uint8_t gain, uint8_t uni_bipolar, uint8_t buf_state)
{
    digitalWrite(_reset, HIGH);  // put _RESET to HIGH level
    SPI.begin();
                  // Choose CLOCK REGISTER with mode 'WRITE' next operation 
    write_register(ZERO_DRDY_7&0 | CLOCK_REG_456 | WRITE_3 | NORMAL_OP_MODE_2 | CH0_01); 
    write_register(CLRG_FOR_CORRECT_OP_765 | CLRG_MASTER_CLOCK_DISABLE_4&0 | fregADC_Hz & FREG_ADC_MASK | output_rate & OUR_MASK);
                  // Choose SETUP REGISTER
    write_register(ZERO_DRDY_7&0 | SETUP_REG_456 | WRITE_3 | NORMAL_OP_MODE_2 | CH0_01);
    write_register(calibr & CALIBR_MASK | gain & GAIN_MASK | uni_bipolar & POLAR_MASK| buf_state & BUFFER_MASK| STRG_FSYNC_IN_RESET_STATE & 0);
}


void ModuleAD7705::reset_adc()
{
    digitalWrite(_reset, LOW);   // put _RESET to LOW level
    delayMicroseconds(DELAY_BY_RESET);
    digitalWrite(_reset, HIGH);  // put _RESET to HIGH level
};

 bool ModuleAD7705::isDataReady()
 {
    return digitalRead(_drdy) == LOW;
 }

void ModuleAD7705::setFuncDigitalWrite(int (*func)(int, int))
{
    writeDigitalPin = func;
}


int ModuleAD7705::digitalWrite(uint8_t pin, bool val)
{
    if (!writeDigitalPin) {
        Serial.println("Error: No function assigned!");
        return -1; // Return an error code or default value
    }
    return writeDigitalPin(pin, val);
}

void ModuleAD7705::setFuncDigitalRead(int (*func)(int))
{
    readDigitalPin = func;
}


bool ModuleAD7705::digitalRead(uint8_t pin)
{
    if (!readDigitalPin) {
        Serial.println("Error: No function assigned!");
        return -1; // Return an error code or default value
    }

    return readDigitalPin(pin);
}


int ModuleAD7705::read_serial_data_byByte(size_t bytes)
{
    waitingOnDataReady();
    int recieved_data = SPI.transfer(0x00);
    while(--bytes){
        recieved_data = recieved_data << 8;
        recieved_data |= SPI.transfer(0x00);
    }

    return recieved_data;
}



void ModuleAD7705::write_register(uint8_t instruction)
{
    SPI.transfer(instruction);
}

int ModuleAD7705::write_register_24(uint32_t data)
{
    uint8_t buf[3] = {(data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF};
    size_t   bytes = 3;

    SPI.transfer(buf, bytes);

    return buf[0] << 16 | buf[1] << 8 | buf[2];
}

int ModuleAD7705::waitingOnDataReady()
{
    int cycles = 0;
    
    while(!ModuleAD7705::isDataReady())
    {
        if (cycles++ == CYCLES_LIMIT)
        {
            print("Runtime Error: flag _DRDY is not ready during ");
            print(CYCLES_LIMIT);
            println(" cycles. Reinitialization!");
            return -1; // this is where you can place the error code if a corresponding handler is created
        }
    }

    return 0;
}

int ModuleAD7705::read_channel(uint8_t channel)
{
    select_adc();
    write_register(ZERO_DRDY_7 & 0 | DATA_REG_456 | READ_3 | NORMAL_OP_MODE_2 | channel & CHANNEL_MASK);
    uint16_t data = read_serial_data_byByte(TWO_BYTES);
    unselect_adc();

    return data;
}

int ModuleAD7705::read_clock_channel(uint8_t channel)
{
    select_adc();
    write_register(ZERO_DRDY_7 & 0x00 | CLOCK_REG_456 | READ_3 | NORMAL_OP_MODE_2 | channel & CHANNEL_MASK);
    uint8_t data = read_serial_data_byByte(ONE_BYTE);
    unselect_adc();

    return data;
}

int ModuleAD7705::read_setup_channel(uint8_t channel)
{
    select_adc();
    write_register(ZERO_DRDY_7 & 0x00 | SETUP_REG_456 | READ_3 | NORMAL_OP_MODE_2 | channel & CHANNEL_MASK);
    uint8_t data = read_serial_data_byByte(ONE_BYTE);
    unselect_adc();

    return data;
}

int ModuleAD7705::read_offset_channel(uint8_t channel)
{
    select_adc();
    write_register(ZERO_DRDY_7 & 0x00 | OFFSET_REG_456 | READ_3 | NORMAL_OP_MODE_2 | channel & CHANNEL_MASK);
    int data = read_serial_data_byByte(THREE_BYTES);
    unselect_adc();

    return data;
}

int ModuleAD7705::read_gain_channel(uint8_t channel)
{
    select_adc();
    write_register(ZERO_DRDY_7 & 0x00 | GAIN_REG_456 | READ_3 | NORMAL_OP_MODE_2 | channel & CHANNEL_MASK);
    int data = read_serial_data_byByte(THREE_BYTES);
    unselect_adc();

    return data;
}

void ModuleAD7705::select_adc()
{
    SPI.beginTransaction(SPISettings(F_CPU, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
}

void ModuleAD7705::unselect_adc()
{
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
}

 void ModuleAD7705::set_max_range_in_volts(float volts)
 {
    _max_range_in_volts = volts;
 }

 float ModuleAD7705::val_into_millivolts(float data)
 {
    data = data - _zero;
    return VOLTS_2_MILLIVOLTS(data * _max_range_in_volts/ADC_BITRATE);
 }


 void ModuleAD7705::set_max_range_in_mA(float mA)
 {
    _max_range_in_mA = mA;
 }

 float ModuleAD7705::val_into_mA(float data)
 {
    data = data - _zero;
    return data * _max_range_in_mA/ADC_BITRATE;
 }
 

 void ModuleAD7705::set_max_range_in_kgs(float kgs)
 {
    _max_range_in_kgs = kgs;
 }

 float ModuleAD7705::val_into_kgs(float data)
 {
    data = data - _zero;
    return data * _max_range_in_kgs/ADC_BITRATE;
 }

  

 void ModuleAD7705::set_max_range_in_Nm(float Nm)
 {
    _max_range_in_Nm = Nm;
 }

 float ModuleAD7705::val_into_Nm(float data)
 {
    data = data - _zero;
    return data * _max_range_in_Nm/ADC_BITRATE;
 }

  void ModuleAD7705::set_zero(int zero_value)
 {
    _zero = zero_value;
 }