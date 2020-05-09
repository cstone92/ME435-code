
// pins
#define REG_PORT_LED_BUILT_IN PORTB
#define BIT_LED_BUILD_IN PB5

// counter and compare values
#define TIMER_1_START 0
#define TIMER_1_COMPARE 31250

//   CS22  | CS21  | CS20  |  DEC |Description
//       0 |     0 |     0 |   0  |stop timer
//       0 |     0 |     1 |   1  |prescaler = 1 (no prescaller)
//       0 |     1 |     0 |   2  |prescaler = 8
//       0 |     1 |     1 |   3  |prescaler = 64
//       1 |     0 |     0 |   4  |prescaler = 256
//       1 |     0 |     1 |   5  |prescaler = 1024
uint8_t prescalerMode = 0x04;

void setup() {
  // Set LED pin to be output
  DDRB |= _BV(BIT_LED_BUILD_IN);

  // Reset Timer1 Control Reg A
  TCCR1A = 0;

  // set waveform generation mode WGM
  TCCR1B &= ~_BV(WGM13); //clears
  TCCR1B &= ~_BV(WGM12); //clears
  TCCR1B &= ~_BV(WGM11); //clears
  TCCR1B &= ~_BV(WGM10); //clears
  TCCR1B |= _BV(WGM12); //sets

  // Set Prescaller of 256
  TCCR1B &= ~0b111;         // set the three bits in TCCR2B to 0 (clears prescaler)
  TCCR1B = (TCCR1B & 0b11111000) | (prescalerMode);

  TCNT1 = TIMER_1_START;    // Reset Timer1
  OCR1A = TIMER_1_COMPARE;  // set compare values
  TIMSK1 = _BV(OCIE1A);     // Enable timer1 compare interrupt
  sei();                    // Enable global interrupts
}

void loop() {
  delay(500);
}

ISR(TIMER1_COMPA_vect) {
  REG_PORT_LED_BUILT_IN ^= _BV(BIT_LED_BUILD_IN);
}