/*
 * Emutex_MAX9657
 * 
 * An Arduino library for the Emutex 56-Port GPIO shield, based on
 * the Maxim Integrated MAX9657 I/O Expander and LED Display Driver.
 *
 * Copyright © 2014, Emutex Ltd.
 * All rights reserved.
 * http://www.emutex.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * 3. All modifications to the source code must be clearly marked as
 *    such. Binary redistributions based on modified source code must
 *    be clearly marked as modified versions in the documentation and/or
 *    other materials provided with the distribution.
 *
 * 4. The name of Emutex Ltd. may not be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <Arduino.h>
#include <SPI.h>

#include "Emutex_MAX9657.h"

#define ADDR_NO_OP          0x00
#define ADDR_G_CURRENT      0x02
#define ADDR_CONFIG         0x04
#define ADDR_DISP_TEST      0x07
#define ADDR_CFG_P7_4       0x09
#define ADDR_CFG_P11_8      0x0A
#define ADDR_CFG_P15_12     0x0B
#define ADDR_CFG_P19_16     0x0C
#define ADDR_CFG_P23_20     0x0D
#define ADDR_CFG_P27_24     0x0E
#define ADDR_CFG_P31_28     0x0F
#define ADDR_DATA_P11_4     0x44
#define ADDR_DATA_P19_12    0x4C
#define ADDR_DATA_P27_20    0x54
#define ADDR_DATA_P31_28    0x5C

#define CURRENT_DRIVE_DFLT  0X0C // 19.5mA
#define CURRENT_DRIVE_MAX   0X0F // 24mA
#define PORT_CFG_LED_DRIVER 0x00
#define PORT_CFG_GPIO_OUT   0x55
#define CONFIG_SHUTDOWN_ON  0x00
#define CONFIG_SHUTDOWN_OFF 0x01
#define DISPLAY_TEST_ON     0x01
#define DISPLAY_TEST_OFF    0x00

#define PORT_DATA_REG_INDEX(p) ((p) >> 3)
#define PORT_DATA_BIT_MASK(p)  (1 << ((p) & 0x7))

#define NUM_PORT_DATA_REG (PORT_DATA_REG_INDEX(MAX9657_PORTS_PER_DEVICE - 1) + 1)  

static byte port_data_reg_addr[] = {
    ADDR_DATA_P11_4, ADDR_DATA_P19_12, ADDR_DATA_P27_20, ADDR_DATA_P31_28
};
static byte output_state[MAX9657_MAX_NUM_DEVICES][sizeof(port_data_reg_addr)];
static byte do_refresh[MAX9657_MAX_NUM_DEVICES][sizeof(port_data_reg_addr)];

/******************************************************************************
 * Private methods
 *****************************************************************************/

void Emutex_MAX9657::spiSendData(byte len) {
    digitalWrite(cs, LOW);
#ifdef ARDUINO_LINUX
    /* Optimise SPI throughput if running on Intel x86 platform */
    SPI.transferBuffer(spidata, NULL, len);
#else
    for (byte i = 0; i < len; i++)
        SPI.transfer(spidata[i]);
#endif
    digitalWrite(cs, HIGH);
}

void Emutex_MAX9657::writeConfigReg(byte addr, byte val) {
    byte len = 0;

    for (byte i = 0; i < ndev; i++) {
        spidata[len++] = addr;
        spidata[len++] = val;
    }
    spiSendData(len);
}

int Emutex_MAX9657::setOutputNextState(int port, boolean state) {
    byte reg_index;
    byte bit_mask;
    byte device = 0;

    if (!initialised)
        return -1;

    while (port >= MAX9657_PORTS_PER_DEVICE) {
        device++;
        port -= MAX9657_PORTS_PER_DEVICE;
    }

    if (device >= ndev)
        return -1;

    reg_index = PORT_DATA_REG_INDEX(port);
    bit_mask = PORT_DATA_BIT_MASK(port);

    if (state)
        output_state[device][reg_index] |= bit_mask;
    else
        output_state[device][reg_index] &= ~bit_mask;

    do_refresh[device][reg_index] = true;

    return 0;
}

/******************************************************************************
 * Public methods
 *****************************************************************************/

Emutex_MAX9657::Emutex_MAX9657(byte csPin, byte numDevices, MAX9657_Mode portMode) {
    ndev = numDevices;
    cs = csPin;
    mode = portMode;
    initialised = false;
}

int Emutex_MAX9657::getDeviceCount() {
    return ndev;
}

void Emutex_MAX9657::begin() {
    if (!initialised) {
        // Configure the SPI bus
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);
        SPI.begin();
        SPI.setDataMode(SPI_MODE0);
        SPI.setBitOrder(MSBFIRST);
        SPI.setClockDivider(SPI_CLOCK_DIV4); // Using 4MHz default, but MAX9657 can support up to 26MHz

        if (mode == MAX9657_MODE_CCLED) {
            // Configure all outputs as full-strength (24mA) constant-current LED drivers
            // NOTE 1: In this mode, the pins are open-drain current sink, for connection to LED cathodes
            // NOTE 2: If using a 5V supply, the datasheet recommends using an 82ohm resistor in series
            //         with each LED and limiting the current drive strength to 20mA. So we set default
            //         drive strength here at 19.5mA.  Use ::setIntensity(15) to change if needed.
            //         If using a 3.3V supply, 24mA drive strength can be used with no series resistors
            writeConfigReg(ADDR_G_CURRENT,  CURRENT_DRIVE_DFLT);
            writeConfigReg(ADDR_CFG_P7_4,   PORT_CFG_LED_DRIVER);
            writeConfigReg(ADDR_CFG_P11_8,  PORT_CFG_LED_DRIVER);
            writeConfigReg(ADDR_CFG_P15_12, PORT_CFG_LED_DRIVER);
            writeConfigReg(ADDR_CFG_P19_16, PORT_CFG_LED_DRIVER);
            writeConfigReg(ADDR_CFG_P23_20, PORT_CFG_LED_DRIVER);
            writeConfigReg(ADDR_CFG_P27_24, PORT_CFG_LED_DRIVER);
            writeConfigReg(ADDR_CFG_P31_28, PORT_CFG_LED_DRIVER);
        } else if (mode == MAX9657_MODE_OUTPUT) {
            writeConfigReg(ADDR_CFG_P7_4,   PORT_CFG_GPIO_OUT);
            writeConfigReg(ADDR_CFG_P11_8,  PORT_CFG_GPIO_OUT);
            writeConfigReg(ADDR_CFG_P15_12, PORT_CFG_GPIO_OUT);
            writeConfigReg(ADDR_CFG_P19_16, PORT_CFG_GPIO_OUT);
            writeConfigReg(ADDR_CFG_P23_20, PORT_CFG_GPIO_OUT);
            writeConfigReg(ADDR_CFG_P27_24, PORT_CFG_GPIO_OUT);
            writeConfigReg(ADDR_CFG_P31_28, PORT_CFG_GPIO_OUT);
        } else {
            /* MAX9657 Input Modes unsupported in current version */
            return;
        }

        initialised = true;
    }
    
    // Write default configuration (shutdown, global current control, no transition detect) 
    writeConfigReg(ADDR_CONFIG, CONFIG_SHUTDOWN_OFF);
}

void Emutex_MAX9657::end() {
    writeConfigReg(ADDR_CONFIG, CONFIG_SHUTDOWN_ON);
}

void Emutex_MAX9657::testLedOutputs(bool y) {
    if (!initialised)
        return;

    if (mode != MAX9657_MODE_CCLED)
        return;

    writeConfigReg(ADDR_DISP_TEST, y ? DISPLAY_TEST_ON : DISPLAY_TEST_OFF);
}
	
void Emutex_MAX9657::setLedIntensity(byte intensity) {
    if (!initialised)
        return;

    if (mode != MAX9657_MODE_CCLED)
        return;

    if (intensity >= 0 || intensity < 16)	
        writeConfigReg(ADDR_G_CURRENT, intensity);
}

void Emutex_MAX9657::updateOutputs(void) {
    if (!initialised)
        return;

    for (byte reg = 0; reg < sizeof(port_data_reg_addr); reg++) {
        byte len = 0;
        for (byte dev = 0; dev < ndev; dev++) {
            if (do_refresh[dev][reg]) {
                spidata[len++] = port_data_reg_addr[reg];
                spidata[len++] = output_state[dev][reg];
                do_refresh[dev][reg] = false;
            } else {
                spidata[len++] = ADDR_NO_OP;
                spidata[len++] = 0;
            }
        }
        if (len)
            spiSendData(len);
    }
}

int Emutex_MAX9657::setOutput(int port, boolean state, boolean defer) {
    int ret = setOutputNextState(port, state);

    if (!initialised)
        return -1;

    if (!defer && !ret)
	    updateOutputs();
    return ret;
}

void Emutex_MAX9657::resetOutputs(void) {
    if (!initialised)
        return;

    for (byte reg = 0; reg < sizeof(port_data_reg_addr); reg++) {
        byte len = 0;
        for (byte dev = 0; dev < ndev; dev++) {
            output_state[dev][reg] = 0x0;
            spidata[len++] = port_data_reg_addr[reg];
            spidata[len++] = output_state[dev][reg];
            do_refresh[dev][reg] = false;
        }
        spiSendData(len);
    }
}
