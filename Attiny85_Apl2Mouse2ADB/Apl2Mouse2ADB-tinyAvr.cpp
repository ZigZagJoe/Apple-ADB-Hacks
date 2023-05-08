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
#include <avr/interrupt.h>

// util macros
#define bv(BIT) (1 << BIT)

#define bset(X,BIT) (X |= bv(BIT))
#define bclr(X,BIT) (X &= ~bv(BIT))
#define bisset(X,BIT) (X & bv(BIT))

////////////// ADB CONFIG
#define ADB_PORT        PORTB
#define ADB_PIN         PINB
#define ADB_DDR         DDRB
#define ADB_DATA_BIT    3
// ADB-green

#define SPEED_CONSTANT 2

// Quadrature Encoder Pins
// NOTE: this are bits with in the port ie. 2 is Port B2 - look at datasheet to turn this into Pin#
int y1Pin = 2; //grey 
int y2Pin = 4; //purple
int x1Pin = 0; //yellow
int x2Pin = 1; //orange
int buttonPin = 5; //blue

#define PCINT_DDR DDRB
#define PCINT_PORT PORTB
#define PCINT_PIN PINB

// STATE
volatile uint8_t mouseUpdRdy = 0;
volatile uint8_t mouseReqRXd     = 0;

volatile bool buttonState = 0;
volatile int16_t mouseAccumX = 0;
volatile int16_t mouseAccumY = 0;

volatile uint8_t lastPortState = 0;

ISR(PCINT0_vect) {
  // read the Port
  uint8_t state = PCINT_PIN;
  
  uint8_t chg = state ^ lastPortState; // xor - set bits if different

  // set the button state. OR so we don't lose clicks
  buttonState |= ! bisset(state,buttonPin);

  // interrupt was due to Y
  if (bisset(chg,y1Pin)) {    
    uint8_t y1 = bisset(state,y1Pin)?1:0;
    uint8_t y2 = bisset(state,y2Pin)?1:0;
  
    mouseAccumY += (y1 == y2) ? 1 : -1;
  }
  
  // interrupt was due to X
  if (bisset(chg,x1Pin)) {
    uint8_t x1 = bisset(state,x1Pin)?1:0;
    uint8_t x2 = bisset(state,x2Pin)?1:0;

    mouseAccumX += (x1 == x2) ? 1 : -1;
  }
  
  lastPortState = state;
  mouseUpdRdy = 1;
}

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

int main (void) {

	// directions
	bclr(PCINT_DDR,y1Pin);
	bclr(PCINT_DDR,x2Pin);
	bclr(PCINT_DDR,x1Pin);
	bclr(PCINT_DDR,x2Pin);
	bclr(PCINT_DDR, buttonPin);
	bclr(PCINT_DDR, ADB_DATA_BIT);

	// enable pullup
	bset(PCINT_PORT,buttonPin); 
	
	// set up PC Ints
	bset(GIMSK, PCIE);

	bset(PCMSK,y1Pin);
	bset(PCMSK,x1Pin);
	bset(PCMSK,buttonPin);

	// enabled interrupts
	sei();
	
	while(true) {
	  uint8_t cmd = 0;
	  
	  if (mouseUpdRdy) // start informing the host we want to update
		cmd = adb_recv_cmd(mouseReqRXd);
	  
	  // see command is addressed to us
	  if(((cmd >> 4) & 0x0F) == 3) {
		switch(cmd & 0x0F) {
		  case 0xC: // talk register 0
			//Serial.println("Mouse poll");
			
			if(mouseUpdRdy) {		
			  // generate a fresh register value
			  uint16_t mouseRegister0   = 0;
			  
			  // disable interrupts to read atomically
			  cli();
			  
			  if(!(buttonState)) // mouse click
				mouseRegister0 |= (1 << 15);

			  int16_t Ymv = mouseAccumY;
			  int16_t Xmv = mouseAccumX;
			  
			  // reset state
			  mouseUpdRdy = 0;
			  
			  // reset accumulators
			  buttonState = 0;
			  mouseAccumX = 0;
			  mouseAccumY = 0;
			  
			  sei(); // re-enable interrupts
			  
			  Ymv = Ymv * SPEED_CONSTANT;    
			  Xmv = Xmv * SPEED_CONSTANT;

			  if(Ymv != 0) {
				mouseRegister0 |= ((Ymv > 0) & 1) << 14; // sign bit
				mouseRegister0 |= (((~(Ymv))+1) & 0x3F) << 8;; // lop off any bits in excess of 6 - value is 63
			  }
			  
			  if(Xmv != 0) {
				mouseRegister0 |= ((Xmv < 0) & 1) << 6; // sign bit
				mouseRegister0 |= (Xmv & 0x3F);
			  }
			  
			  // send it
			  
			  _delay_us(180); // stop to start time / interframe delay
			  ADB_DDR |= 1<<ADB_DATA_BIT;  // set output
			  place_bit1(); // start bit
			  send_byte((mouseRegister0 >> 8) & 0x00FF);
			  send_byte(mouseRegister0 & 0x00FF);
			  place_bit0(); // stop bit
			  ADB_DDR &= ~(1<<ADB_DATA_BIT); // set input

			  // limit update rate to 100hz...
			  _delay_ms(10);
			  mouseReqRXd = 0;
			}
			break;
		  default:
			//Serial.print("Unknown cmd: ");
			//Serial.println(cmd, HEX);
			break;
		}
	  } else {
		if(mouseUpdRdy)
		  mouseReqRXd = 1;
	  }
  }
}
