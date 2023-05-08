/* The ADB code here is mostly derived from: https://github.com/tmk/tmk_keyboard
 * It was originally ADB host, and I've adapted it for device use.
 * Below is the license and credit for the original ADB host implementation.
 */
/*
Copyright 2011 Jun WAKO <wakojun@gmail.com>
Copyright 2013 Shay Green <gblargg@gmail.com>
This software is licensed with a Modified BSD License.
All of this is supposed to be Free Software, Open Source, DFSG-free,
GPL-compatible, and OK to use in both free and proprietary applications.
Additions and corrections to this file are welcome.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in
  the documentation and/or other materials provided with the
  distribution.
* Neither the name of the copyright holders nor the names of
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
#include <util/delay.h>

#include "TrackPoint.h"

////////////// ADB CONFIG
#define ADB_PORT        PORTD
#define ADB_PIN         PIND
#define ADB_DDR         DDRD
#define ADB_DATA_BIT    3

/// TRACKPOINT PINS

#define CLOCK   0
#define DATA    1
#define RESET   2
#define CLOCK_INT 0
#define RESET    2

TrackPoint trackpoint(CLOCK, DATA, RESET, true);

// STATE
uint8_t mouseUpdRdy = 0;
uint16_t mouseRegister0   = 0;
uint8_t mouseReqRXd     = 0;

bool buttonState = 0;
int16_t mouseAccumX = 0;
int16_t mouseAccumY = 0;

// The original data_lo code would just set the bit as an output while remaining an input
// That works for a host, since the host is doing the pullup on the ADB line,
// but for a device, it won't reliably pull the line low.  We need to actually
// set it.
#define data_lo() { (ADB_DDR |=  (1<<ADB_DATA_BIT)); (ADB_PORT &= ~(1<<ADB_DATA_BIT)); }
#define data_hi() (ADB_DDR &= ~(1<<ADB_DATA_BIT))
#define data_in() (ADB_PIN &   (1<<ADB_DATA_BIT))

static inline uint16_t wait_data_lo(uint16_t us)
{
    do {
        if ( !data_in() )
            break;
        _delay_us(1 - (6 * 1000000.0 / F_CPU));
    }
    while ( --us );
    return us;
}

static inline uint16_t wait_data_hi(uint16_t us)
{
    do {
        if ( data_in() )
            break;
        _delay_us(1 - (6 * 1000000.0 / F_CPU));
    }
    while ( --us );
    return us;
}

static inline void place_bit0(void)
{
    data_lo();
    _delay_us(65);
    data_hi();
    _delay_us(35);
}

static inline void place_bit1(void)
{
    data_lo();
    _delay_us(35);
    data_hi();
    _delay_us(65);
}
static inline void send_byte(uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        if (data&(0x80>>i))
            place_bit1();
        else
            place_bit0();
    }
}
static uint8_t inline adb_recv_cmd(uint8_t srq) 
{
  uint8_t bits;
  uint16_t data = 0;
  
  // find attention & start bit
  if(!wait_data_lo(5000)) return 0;
  uint16_t lowtime = wait_data_hi(1000);
  if(!lowtime || lowtime > 500) {
    return 0;
  }
  wait_data_lo(100);
  
  for(bits = 0; bits < 8; bits++) {
    uint8_t lo = wait_data_hi(130);
    if(!lo) {
      goto out;
    }
    uint8_t hi = wait_data_lo(lo);
    if(!hi) {
      goto out;
    }
    hi = lo - hi;
    lo = 130 - lo;
    
    data <<= 1;
    if(lo < hi) {
      data |= 1;
    }
  }
  
  if(srq) {
    data_lo();
    _delay_us(250);
    data_hi();
  } else {
    // Stop bit normal low time is 70uS + can have an SRQ time of 300uS
    wait_data_hi(400);
  }
  
  return data;
out:
  return 0;
}

void clockInterrupt(void) {
  trackpoint.getDataBit();
}

void setup() {

  trackpoint.reset();
  trackpoint.setSensitivityFactor(0xA0); // lower = less sensitive
  trackpoint.setStreamMode();

  attachInterrupt(CLOCK_INT, clockInterrupt, FALLING);
    
  Serial.begin(19200);
  
  // Set ADB line as input
  ADB_DDR &= ~(1<<ADB_DATA_BIT);
}

void loop() {
  uint8_t cmd = 0;
  
  if(trackpoint.reportAvailable()) {
  
    TrackPoint::DataReport d = trackpoint.getStreamReport();
    
    // make sure we don't lose clicks
    buttonState |= (d.state & 8);
    mouseAccumX += d.x;
    mouseAccumY += d.y;
  
    /*Serial.print("REPORT: ");
    
    if (d.state & 8) 
      Serial.print("Button pressed ");
    
    Serial.print(d.x);
    Serial.print(" ");
    Serial.println(d.y);*/
    
    // generate a fresh register value
    mouseRegister0 = 0;
    
    // mouse click
    if(!(buttonState)) // I guess it's inverted??
      mouseRegister0 |= (1 << 15);
    
    if(mouseAccumY != 0) {
      mouseRegister0 |= ((mouseAccumY > 0) & 1) << 14; // sign bit
      mouseRegister0 |= (((~(mouseAccumY))+1) & 0x3F) << 8;; // lop off any bits in excess of 6 - value is 63
    }
    
    if(mouseAccumX != 0) {
      mouseRegister0 |= ((mouseAccumX < 0) & 1) << 6; // sign bit
      mouseRegister0 |= (mouseAccumX & 0x3F);
    }
    
    mouseUpdRdy = 1;
  }

  if (mouseUpdRdy) // start informing the host we want to update
    cmd = adb_recv_cmd(mouseReqRXd);
  
  // see command is addressed to us
  if(((cmd >> 4) & 0x0F) == 3) {
    switch(cmd & 0x0F) {
      case 0xC: // talk register 0
        //Serial.println("Mouse poll");
		
        if(mouseUpdRdy) {
          _delay_us(180); // stop to start time / interframe delay
          ADB_DDR |= 1<<ADB_DATA_BIT;  // set output
          place_bit1(); // start bit
          send_byte((mouseRegister0 >> 8) & 0x00FF);
          send_byte(mouseRegister0 & 0x00FF);
          place_bit0(); // stop bit
          ADB_DDR &= ~(1<<ADB_DATA_BIT); // set input

          /*Serial.println("xmit update: ");
          Serial.println("111111");
          Serial.println("5432109876543210");
          for (unsigned int test = 0x8000; test; test >>= 1) {
            Serial.write(mouseRegister0  & test ? '1' : '0');
          }
          Serial.println();
		  */
          
		  // reset state
          mouseUpdRdy = 0;
          mouseReqRXd = 0;

          // reset accumulators
          buttonState = 0;
          mouseAccumX = 0;
          mouseAccumY = 0;
        }
        break;
      default:
        Serial.print("Unknown cmd: ");
        Serial.println(cmd, HEX);
        break;
    }
  } else {
    if(mouseUpdRdy)
      mouseReqRXd = 1;
  }
}
