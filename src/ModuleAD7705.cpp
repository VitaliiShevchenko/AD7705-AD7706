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


void ModuleAD7705::init()
{
    reset_adc();
    // Choose CLOCK REGISTER with mode 'WRITE' next operation 
    write_register(ZERO_DRDY_7&0 | CLOCK_REG_456 | WRITE_3 | NORMAL_OP_MODE_2 | CH0_01); //0x20
    // Write to the CLOCK REGISTER with oscilator = 4.9152 MHz, output update rate is 50 Hz and −3 dB Filter Cutoff is 13 Hz
    write_register(CLRG_FOR_CORRECT_OP_765 | CLRG_MASTER_CLOCK_DISABLE_4&0 | CLRG_CLOCK_DIVIDER_BY2_3 | CLRG_CLOCK_BIT_2 | CLRG_FS_CLK_IS_HIGH_50Hz_13Hz);//0x0C
    // Choose SETUP REGISTER
    write_register(ZERO_DRDY_7&0 | SETUP_REG_456 | WRITE_3 | NORMAL_OP_MODE_2 | CH0_01) ;    //0x10
    // Write to the SETUP REGISTER: self-calibration, unipolar mode(0 to 5v) and buffer disable
    write_register(STRG_MD_SELF_CALIBR_76 | STRG_GAIN1_543 | STRG_UNIPOLAR_OPERATION_2 | STRG_BUFFER_ENABLE_1 &0| STRG_FSYNC_IN_RESET_STATE&0);//0x44
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

void ModuleAD7705::write_register(uint8_t instruction)
{
    SPI.transfer(instruction);
}

uint8_t ModuleAD7705::read_serial_data()
{
    uint8_t data = 0x00;
    waitingOnDataReady();
    data = SPI.transfer(data);  // read 8-bits Register
    return data;
}

uint16_t ModuleAD7705::read_serial_data_16()
{
    digitalWrite(_cs, LOW);
    uint16_t data = 0x00;
    waitingOnDataReady();
    data = SPI.transfer16(data);  //read 16-bits Register

    return data; 
}

int ModuleAD7705::read_serial_data_24()
{
    int data = 0x00;
    waitingOnDataReady();
    SPI.transfer(data, 3);    //read 24-bits Register

    return data;
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
    uint16_t data = read_serial_data_16();
    unselect_adc();

    return data;
}

int ModuleAD7705::read_clock_channel(uint8_t channel)
{
    select_adc();
    write_register(ZERO_DRDY_7 & 0x00 | CLOCK_REG_456 | READ_3 | NORMAL_OP_MODE_2 | channel & CHANNEL_MASK);
    uint8_t data = read_serial_data();
    unselect_adc();

    return data;
}

int ModuleAD7705::read_setup_channel(uint8_t channel)
{
    select_adc();
    write_register(ZERO_DRDY_7 & 0x00 | SETUP_REG_456 | READ_3 | NORMAL_OP_MODE_2 | channel & CHANNEL_MASK);
    uint8_t data = read_serial_data();
    unselect_adc();

    return data;
}

void ModuleAD7705::select_adc()
{
    digitalWrite(_cs, LOW);
}

void ModuleAD7705::unselect_adc()
{
    digitalWrite(_cs, HIGH);
}
