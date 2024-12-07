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
#include <stdint.h>
#include <SPI.h>

#define LOW  0x00
#define HIGH 0x01

#define READ_BIT(REG, BIT)   (((REG) >> (BIT)) & 1U)     // Read a bit
#define SET_BIT(REG, BIT)    ((REG) |= (1U << (BIT)))    // Set a bit
#define float NS2US(VAL)     (VAL/1000)                  // Convert nanoseconds(ns) to microseconds(ms)

// Constants for better readability
#define MILLISECS_PER_SEC 1000
#define USECS_PER_SEC (1000 * 1000)
#define NOPS_ns     62.5f                             // Arduino running at 16 MHz (NOP takes 62.5 ns)

#define HZ2US(hz)   (1000000 / (hz))
#define HZ2MS(hz)   (1000 / (hz))
#define US2S(us)    ((us) * 1e-6f)
#define US2MS(us)   ((us) * 1e-3f)
#define MS2US(ms)   ((ms) * 1000)
#define MS2S(ms)    ((ms) * 1e-3f)
#define S2MS(s)     ((s) * MILLISECS_PER_SEC)
#define DS2MS(ds)   ((ds) * 100)
#define HZ2S(hz)    US2S(HZ2US(hz))

#define TIME_WRITE           1                          // delay 1 ms
#define CYCLES_LIMIT         (32767)                    // 1s / 50tacts * 16000000Hz = 320000
#define TIME_READ_DELAY      1                          // delay 1 ms
#define START_TIME_DELAY_us  6                          // delay 6 us by 4.9512 MHz | 16 us by 2.4576 MHz | 20 us by 1MHz
#define F_CLKIN_HZ           1.45 * 1e6                 // 400kHz-2.5MHz (For specified performance) 45-55% is the best
#define T_FCLKIN_US double   (HZ2US(F_CLKIN_HZ/2))      //0.69us
#define T_DRDY_us            (500*NS2US(T_FCLKIN_ns))   // During this time (345us), we need to get all the data from the serial output DOUT
#define T2_ns                100                        // RESET pulse width (min value)

// Read Operation 
#define T3_ns             0                 // delay when DRDY to CS setup time (0ns - min value)
#define T4_ns             120               // time between SCLK's HIGH LEVELS (CS falling edge to SCLK rising edge setup time) (120ns - min value)
#define T5_ns             0                 // time between SCLK's HIGH LEVELS (SCLK falling edge to data valid delay 0-80ns(5V), 0-100ns(3V))
#define T6_ns             100               // time when SCLK set to HIGH LEVELS (SCLK high pulse width (100ns - min value))
#define T7_ns             100               // time when SCLK set to LOW  (SCLK low pulse width (100ns - min value))
#define T8_ns             0                 // min time when SCLK set to HIGH LEVEL and start _CS to HIGH LEVEL (CS rising edge to SCLK rising edge hold time)
#define T9_ns             50                // Bus relinquish time after SCLK rising edge 10-60ns(5V), 10-100ns(3V) 
#define T10_ns            100               // SCLK falling edge to DRDY high (0-100ms)

// Write operation
#define T11_ns            120               // CS falling edge to SCLK rising edge setup time (120ns - min value)
#define T12_ns            30                // Data valid to SCLK rising edge setup time (30ns - min value)
#define T13_ns            20                // Data valid to SCLK rising edge hold time (20ns - min value)
#define T14_ns            100               // SCLK high pulse width  (100ns - min value)
#define T15_ns            100               // SCLK low pulse width (100ns- min value)
#define T16_ns            0                 // CS rising edge to SCLK rising edge hold time (0ns - min value)

   

ModuleAD7705::ModuleAD7705(uint8_t cs, uint8_t sclk, uint8_t din, uint8_t reset, uint8_t drdy, uint8_t dout):
    _cs(cs),
    sclk(sclk),
    din(din),
    _reset(reset),
    _drdy(drdy),
    dout(dout)
{
    // Constructor
    // init();
}


void ModuleAD7705::init()
{
    // Serial.println("init");
    reset_adc();
    // Choose CLOCK REGISTER with mode 'WRITE' next operation 
    write_register(ZERO_DRDY_7&0 | CLOCK_REG_456 | WRITE_3 | NORMAL_OP_MODE_2 | CH0_01); //0x20
    // Write to the CLOCK REGISTER with oscilator = 4.9152 MHz, output update rate is 50 Hz and −3 dB Filter Cutoff is 13 Hz
    write_register(CLRG_FOR_CORRECT_OP_765 | CLRG_MASTER_CLOCK_DISABLE_4&0 | CLRG_CLOCK_DIVIDER_BY2_3 | CLRG_CLOCK_BIT_2 | CLRG_FS_CLK_IS_HIGH_50Hz_13Hz);//0x0C
    // Choose SETUP REGISTER
    write_register(ZERO_DRDY_7&0 | SETUP_REG_456 | WRITE_3 | NORMAL_OP_MODE_2 | CH0_01) ;    //0x10
    // Write to the SETUP REGISTER: self-calibration, unipolar mode(0 to 5v) and buffer disable
    write_register(STRG_MD_SELF_CALIBR_76 | STRG_GAIN1_543 | STRG_UNIPOLAR_OPERATION_2 | STRG_BUFFER_ENABLE_1 &0| STRG_FSYNC_IN_RESET_STATE&0);//0x44
      SPI.begin();
}


void ModuleAD7705::reset_adc()
{
    //digitalWrite(_cs, LOW);    // put _CS to LOW level
    digitalWrite(_reset, LOW); // put _RESET to LOW level
    delayMicroseconds(100);
    digitalWrite(_reset, HIGH);// put _RESET to HIGH level
    //digitalWrite(_cs, HIGH);   // put _CS to HIGH level
    //digitalWrite(sclk, HIGH);  // put SCLK to high level
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
    // digitalWrite(_cs, LOW);
    for(uint8_t bit=0; bit < 8; bit++)
    {
        //delayMicroseconds(12);//
        // delay(1);
        digitalWrite(sclk, LOW);
        //delayMicroseconds(12);
        // delay(1);
        digitalWrite(din, READ_BIT(instruction, 7-bit));
        digitalWrite(sclk, HIGH);
    }
    //delayMicroseconds(12);
    // delay(1);
    // digitalWrite(_cs, HIGH);
}

int ModuleAD7705::read_serial_data()
{
    uint8_t q_bit = 8;
    read_serial_data(q_bit);
}

int ModuleAD7705::read_serial_data_16()
{
    uint8_t q_bit = 16;
    return read_serial_data(q_bit);
}

int ModuleAD7705::read_serial_data_24()
{
    uint8_t q_bit = 24;
    return read_serial_data(q_bit);
}

int ModuleAD7705::read_serial_data(uint8_t q_bit)
{
    uint16_t data = 0x00 ; // 1 cycle. Sum = 6.5cycles (406ns)
    //digitalWrite(_cs, LOW); // 1-2 cycles. Sum = 8cycles
    //digitalWrite(sclk, HIGH); // 1-2 cycles. Sum = 9.5cycles
    //delayMicroseconds(12);
    // delay(1);
    uint8_t sreg = SREG;
    noInterrupts();
    for(uint8_t bit = 0; bit < q_bit; bit++)                     // 1cycle
    {
        
        
        digitalWrite(sclk, LOW);                                 //1-2 cycles
        //delayMicroseconds(12);
        // delay(1);
        Serial.print(digitalRead(dout));
        if ( digitalRead(dout) & 1 ) SET_BIT(data, q_bit-bit-1);   //8cycles
        digitalWrite(sclk, HIGH);                                //1-2 cycles
        //delayMicroseconds(12);
        // delay(1);
    }                                                            // 2 cycles
    //digitalWrite(_cs, HIGH);
    SREG = sreg;
    return data;
}

int ModuleAD7705::waitDataReady()
{
    int cycles = 0;
    
    while(!ModuleAD7705::isDataReady())
    {
        if (cycles++ == CYCLES_LIMIT)
        {
            Serial.print("Runtime Error: flag _DRDY is not ready during ");
            Serial.print(CYCLES_LIMIT);
            Serial.println(" cycles. Reinitialization!");
            init();
            return 0;
        }
    }

    return 0;
}

int ModuleAD7705::read_channel_one()
{
    // while(digitalRead(_drdy ) == 1){}
    //waitDataReady();
    write_register(ZERO_DRDY_7 & 0 | DATA_REG_456 | READ_3 | NORMAL_OP_MODE_2 | CH0_01);  //0x38  1-2 cycles
    return read_serial_data_16(); // 4 cycles. Sum = 5.5cycles
}

int ModuleAD7705::read_channel_two()
{
    //while(digitalRead(_drdy)){}
    write_register(ZERO_DRDY_7 & 0 | DATA_REG_456 | READ_3 | NORMAL_OP_MODE_2 | CH1_01);  //0x39
    return read_serial_data_16();
}

int ModuleAD7705::read_channel_three()
{
    while(digitalRead(_drdy)){}
    write_register(ZERO_DRDY_7 & 0 | DATA_REG_456 | READ_3 | NORMAL_OP_MODE_2 | CH3_01);  //0x3B
    return read_serial_data_16();
}

