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

#ifndef __EMUTEX_MAX9657_H__
#define __EMUTEX_MAX9657_H__

#include <Arduino.h>

/* MAX9657 Port Mode options
 *
 * Important notes regarding use of MAX9657_MODE_CCLED mode:
 *
 * NOTE 1: In this mode, the pins are open-drain current sink, for connection to LED cathodes
 * NOTE 2: If using a 5V supply, the datasheet recommends using an 82ohm resistor in series
 *         with each LED and limiting the current drive strength to 20mA. So we set default
 *         drive strength here at 19.5mA.  Use ::setIntensity(15) to change if needed.
 *         If using a 3.3V supply, 24mA drive strength can be used with no series resistors
 *
 * Please refer to the MAX9657 datasheet for more information.
 */
typedef enum {
    /* Supported modes */
    MAX9657_MODE_CCLED = 0,    /* Constant-current LED driver */
    MAX9657_MODE_OUTPUT,       /* GPIO output */

    /* Currently unsupported modes */
    MAX9657_MODE_INPUT,        /* GPIO input */
    MAX9657_MODE_INPUT_PULLUP, /* GPIO input with pull-up */
} MAX9657_Mode;

#define MAX9657_MAX_NUM_DEVICES 4
#define MAX9657_PORTS_PER_DEVICE 28 /* Assumes the 28-port variant only */

class Emutex_MAX9657 {
private:
    /* initialisation flag */
    bool initialised;
    /* SPI chip-select pin */
    byte cs;
    /* Port Mode (all ports) */
    MAX9657_Mode mode;
    /* Number of daisy-chained MAX9657 devices */
    byte ndev;
    /* SPI data buffer */
    byte spidata[MAX9657_MAX_NUM_DEVICES*2];
    /* Internal method to transmit SPI data buffer */
    void spiSendData(byte len);
    /* Internal method to write a configuration register across all devices */
    void writeConfigReg(byte addr, byte val);
    /* Set the next state of the specified LED or GPIO output port
     * All output ports are initially OFF (false) by default
     */
    int setOutputNextState(int port, boolean state);

public:
    /* Emutex_MAX9657 constructor
     * csPin - the digital pin to use for SPI chip-select signal
     * numDevices - the number of daisy-chained MAX9657 devices connected
     * portMode - default port mode: GPIO mode (default), or constant-current LED driver mode
     */
    Emutex_MAX9657(byte csPin, byte numDevices, MAX9657_Mode portMode = MAX9657_MODE_OUTPUT);

    /* Returns the number of daisy-chained MAX9657 devices connected (as supplied to constructor) */
    int getDeviceCount(void);

    /* Enable the MAX9657 devices (must be called at least one before using any other function) */
    void begin();

    /* Disable the MAX9657 devices (call begin() to re-enable) */
    void end();

    /* Turn on all LEDs with 50% drive strength.  Use only if portMode is MAX9657_MODE_CCLED */
    void testLedOutputs(bool y);

    /* Set LED drive strength [0-15], corresponding to 16-steps of 1.5mA each where 15 = 24mA (maximum).
     * Use only if portMode is MAX9657_MODE_CCLED.  See notes in .cpp file regarding drive strength.
     */
    void setLedIntensity(byte intensity);

    /* Set the state of the specified LED or GPIO output port
     * All output ports are initially OFF (false) by default
     * If defer = true, update won't be applied until updateOutputs() is called,
     * which can be useful for batching port updates to improve performance.
     */
    int setOutput(int port, boolean state, boolean defer);

    /* Update the state of the output ports (i.e. apply the next state changes, if any) */
    void updateOutputs(void);

    /* Reset the state of the output ports to OFF */
    void resetOutputs(void);
};

#endif // __MAX9657_H__
