/*
 * Example of Emutex MAX9657 shield library use in Contant-Current LED mode
 *
 * Assumes an Emutex MAX9657 GPIO Expander shield connected via SPI
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

#include <SPI.h>
#include <Emutex_MAX9657.h>

#define NUM_DEVICES 2
#define NUM_PORTS (MAX9657_PORTS_PER_DEVICE * NUM_DEVICES)
#define SPI_CS_PIN 10

Emutex_MAX9657 max9657(SPI_CS_PIN, NUM_DEVICES, MAX9657_MODE_CCLED);

void setup() {
  // put your setup code here, to run once:
  max9657.begin();
}

int state = false;
void loop() {
  // put your main code here, to run repeatedly: 
  max9657.testLedOutputs(state);
  state = !state;
  delay(1000);
}