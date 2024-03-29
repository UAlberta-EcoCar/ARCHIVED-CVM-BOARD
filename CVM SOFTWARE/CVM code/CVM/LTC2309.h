/*!
LTC2309: 8-channel, 12-Bit SAR ADC with I2C interface
LTC2301: 1-Channel, 12-Bit ADCs with I2C Compatible Interface

@verbatim

The LTC2309 is a low noise, low power, 8-channel, 12-bit successive
approximation ADC with an I2C compatible serial interface. This ADC includes an
internal reference and a fully differential sample-and-hold circuit to reduce
common mode noise. The LTC2309 operates from an internal clock to achieve a fast
1.3 microsecond conversion time.

The LTC2309 operates from a single 5V supply and draws just 300 microamps at a
throughput rate of 1ksps. The ADC enters nap mode when not converting, reducing
the power dissipation.

I2C DATA FORMAT (MSB First):


       Byte #1                             Byte #2
START  SA6 SA5 SA4 SA3 SA2 SA1 SA0 W SACK  SD OS S1 S0 UNI SLP X X SACK

             Byte #3                             Byte #4                             Byte #5
Repeat Start SA6 SA5 SA4 SA3 SA2 SA1 SA0 R SACK  D11 D10 D9  D8  D7  D6  D5 D4 MACK  D3 D2 D1 D0 X  X  X  X  MNACK  STOP

SACK  : Slave Acknowledge
MACK  : Master Acknowledge
MNACK : Master Not Acknowledge
SD    : Single, Differential# Bit
OS    : ODD, Sign# Bit
Sx    : Address Select Bit
COM   : CH7/COM Configuration Bit
UNI   : Unipolar, Bipolar# Bit
SLP   : Sleep Mode Bit
Dx    : Data Bits
X     : Don't care

Example Code:

Read Channel 0 in Single-Ended Unipolar mode

    adc_command = LTC2309_CH0 | LTC2309_UNIPOLAR_MODE;                  // Build ADC command for channel 0

    ack |= LTC2309_read(LTC2309_I2C_ADDRESS, adc_command, &adc_code);   // Throws out last reading
    ack |= LTC2309_read(LTC2309_I2C_ADDRESS, adc_command, &adc_code);   // Obtains the current reading and stores to adc_code variable

    // Convert adc_code to voltage
    adc_voltage = LTC2309_code_to_voltage(adc_code, vref, LTC2309_UNIPOLAR_MODE );

@endverbatim

http://www.linear.com/product/LTC2309
http://www.linear.com/product/LTC2301

http://www.linear.com/product/LTC2309#demoboards
http://www.linear.com/product/LTC2301#demoboards

REVISION HISTORY
$Revision: 4775 $
$Date: 2016-03-14 10:38:42 -0700 (Mon, 14 Mar 2016) $

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.
*/

/*! @file
    @ingroup LTC2309
    Header for LTC2309: 8-channel, 12-Bit SAR ADC with I2C interface
*/

#ifndef LTC2309_H
#define LTC2309_H

#include <Wire.h>

//! @name I2C addresses
//! @{
//! Un-comment the address corresponding to the LTC2309's address
//                                     //  Pin State
// LTC2309 I2C Address                 AD1       AD0

#define LTC2309_I2C_ADDRESS_1 0x08    //  LOW       LOW
#define LTC2309_I2C_ADDRESS_2 0x09   `//  LOW       Float
#define LTC2309_I2C_ADDRESS_3 0x0A    //  LOW       HIGH
#define LTC2309_I2C_ADDRESS_4 0x0B    //  Float     HIGH
#define LTC2309_I2C_ADDRESS_5 0x18    //  Float     Float
#define LTC2309_I2C_ADDRESS_6 0x19    //  Float     LOW
// #define LTC2309_I2C_ADDRESS 0x1A    //  HIGH      LOW
// #define LTC2309_I2C_ADDRESS 0x1B    //  HIGH      Float
// #define LTC2309_I2C_ADDRESS 0x14    //  High      HIGH
//!@}

//! @name Single-Ended Channel Configuration
//! @{
// Single-Ended Channel Configuration
#define LTC2309_CH0                0x80
#define LTC2309_CH1                0xC0
#define LTC2309_CH2                0x90
#define LTC2309_CH3                0xD0
#define LTC2309_CH4                0xA0
#define LTC2309_CH5                0xE0
#define LTC2309_CH6                0xB0
#define LTC2309_CH7                0xF0
//!@}

//! @name Differential Channel Configuration
//! @{
// Differential Channel Configuration
#define LTC2309_P0_N1              0x00
#define LTC2309_P1_N0              0x40

#define LTC2309_P2_N3              0x10
#define LTC2309_P3_N2              0x50

#define LTC2309_P4_N5              0x20
#define LTC2309_P5_N4              0x60

#define LTC2309_P6_N7              0x30
#define LTC2309_P7_N6              0x70
//!@}

//! @name LTC2309 Configuration Bits
//! @{
// LTC1867 Configuration Bits
#define LTC2309_SLEEP_MODE         0x04
#define LTC2309_EXIT_SLEEP_MODE    0x00
#define LTC2309_UNIPOLAR_MODE      0x08
#define LTC2309_BIPOLAR_MODE       0x00
#define LTC2309_SINGLE_ENDED_MODE  0x80
#define LTC2309_DIFFERENTIAL_MODE  0x00
//!@}

// Commands
// Construct a channel / uni/bipolar by bitwise ORing one choice from the channel configuration
// and one choice from the command.

// Example - read channel 3 single-ended
// adc_command = LTC2309_CH3 | LTC2309_UNIPOLAR_MODE;

// Example - read voltage between channels 5 and 4 with 4 as positive polarity and in bipolar mode.
// adc_command = LTC2309_P4_N5 | LTC2309_BIPOLAR_MODE;


//! Reads 12-bit code from LTC2309, programs channel and mode for next conversion.
//! @return Returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
int8_t LTC2309_read(uint8_t i2c_address,    //!< I2C address of device
                    uint8_t adc_command,    //!< ADC command / address bits
                    uint16_t *ptr_adc_code      //!< Returns code read from ADC
                   );


//! Calculates the LTC2309 input voltage.
//! @return Calculated voltage
float LTC2309_code_to_voltage(uint16_t adc_code,           //!< Code read from ADC
                              float LTC2309_vref,           //!< LSB value (volts)
                              uint8_t uni_bipolar
                             );

#endif  // LTC2309_H
