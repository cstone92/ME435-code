
// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

#define REG_PORT_LED_YELLOW PORTD
#define BIT_LED_YELLOW 7
#define REG_PORT_LED_GREEN PORTD
#define BIT_LED_GREEN 6

#define REG_PORT_PUSHBUTTON_YELLOW PORTD
#define REG_PIN_PUSHBUTTON_YELLOW PIND
#define BIT_PUSHBUTTON_YELLOW 3
#define REG_PORT_PUSHBUTTON_GREEN PORTD
#define REG_PIN_PUSHBUTTON_GREEN PIND
#define BIT_PUSHBUTTON_GREEN 2
#define REG_PORT_PUSHBUTTON_BLUE PORTD
#define REG_PIN_PUSHBUTTON_BLUE PIND
#define BIT_PUSHBUTTON_BLUE 4

#define ON HIGH
#define OFF LOW

#define PRESSED LOW
#define NOTPRESSED HIGH

volatile uint8_t mainEventFlags = 0;
#define FLAG_GREEN_TIMER 0X01
#define FLAG_YELLOW_PUSHBUTTON 0X02
#define FLAG_GREEN_PUSHBUTTON 0X04
#define FLAG_BLUE_PUSHBUTTON 0x08

volatile uint8_t portdhistory = 0xFF;

//   CS12  | CS11  | CS10  |  DEC |Description
//       0 |     0 |     0 |   0  |stop timer
//       0 |     0 |     1 |   1  |prescaler = 1 (no prescaller)
//       0 |     1 |     0 |   2  |prescaler = 8
//       0 |     1 |     1 |   3  |prescaler = 64
//       1 |     0 |     0 |   4  |prescaler = 256
//       1 |     0 |     1 |   5  |prescaler = 1024
uint8_t Timer1Prescaler = 0x03;

//   CS22  | CS21  | CS20  |  DEC |Description
//       0 |     0 |     0 |   0  |stop timer
//       0 |     0 |     1 |   1  |prescaler = 1 (no prescaller)
//       0 |     1 |     0 |   2  |prescaler = 8
//       0 |     1 |     1 |   3  |prescaler = 32
//       1 |     0 |     0 |   4  |prescaler = 64
uint8_t Timer2Prescaler = 0x04;

// counter and compare values
#define TIMER_1_START 0
#define TIMER_1_COMPARE 24999  // 0 - 65535

// counter and compare values
#define TIMER_2_START 0
#define TIMER_2_COMPARE 249  // 0 - 256

unsigned long yellowTenthsSecond = 0;
boolean isYellowRunning = false;

unsigned long greenCounter = 1;
boolean isGreenRunning = false;

unsigned long elapsedTime = 0;
unsigned long currentTime = 0;
unsigned long priorTime = 0;

void setup() {
  Serial.begin(9600);

  //////////////// initialize display  ///////////
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  refresh();  // Print a message to the LCD.

  ///////////////// Timer1 ///////////////////////
  // Reset Timer1 Control Reg A
  TCCR1A = 0;

  // set waveform generation mode WGM
//   TCCR1B &= ~_BV(WGM13);  //clears
//   TCCR1B &= ~_BV(WGM12);  //clears
//   TCCR1B &= ~_BV(WGM11);  //clears
//   TCCR1B &= ~_BV(WGM10);  //clears
//   TCCR1B |= _BV(WGM12);   //sets

  // Set Prescaller of 256
  TCCR1B &= ~0b111;  // set the three bits in TCCR2B to 0 (clears prescaler)
  TCCR1B = (TCCR1B & 0b11111000) | (Timer1Prescaler);

  TCNT1 = TIMER_1_START;    // Reset Timer1
  OCR1A = TIMER_1_COMPARE;  // set compare values
  TIMSK1 = _BV(OCIE1A);     // Enable timer1 compare interrupt

  ///////////////// Timer2///////////////////////
  // Reset Timer1 Control Reg A
  TCCR2A = 0;

  // set waveform generation mode WGM
//   TCCR2B &= ~_BV(WGM22);  //clears
//   TCCR2B &= ~_BV(WGM21);  //clears
//   TCCR2B &= ~_BV(WGM20);  //clears
//   TCCR2B |= _BV(WGM22);   //sets

  // Set Prescaller of 256
  TCCR2B &= ~0b111;  // set the three bits in TCCR2B to 0 (clears prescaler)
  TCCR2B = (TCCR2B & 0b11111000) | (Timer2Prescaler);

  TCNT2 = TIMER_2_START;    // Reset Timer1
  OCR2A = TIMER_2_COMPARE;  // set compare values
  TIMSK2 = _BV(OCIE2A);     // Enable timer1 compare interrupt

  /////////////////// setup I/O ///////////////////
  DDRB |= _BV(BIT_LED_YELLOW);  //pinMode(PIN_LED_YELLOW, OUTPUT);
  DDRD |= _BV(BIT_LED_GREEN);   //pinMode(PIN_LED_GREEN, OUTPUT);
  // all other bits are left as input which is default

  // enable the pullup resistors
  REG_PORT_PUSHBUTTON_YELLOW |= _BV(BIT_PUSHBUTTON_YELLOW);
  REG_PORT_PUSHBUTTON_GREEN |= _BV(BIT_PUSHBUTTON_GREEN);
  REG_PORT_PUSHBUTTON_BLUE |= _BV(BIT_PUSHBUTTON_BLUE);

  /////////////////// setup interrupts //////////////////////
  //attachInterrupt(digitalPinToInterrupt(PIN_PUSHBUTTON_YELLOW), yellow_pushbutton_isr, FALLING);
  //attachInterrupt(digitalPinToInterrupt(PIN_PUSHBUTTON_RED), red_pushbutton_isr, FALLING);
  //EICRA = _BV(ISC11) | _BV(ISC01);  // set INT0 and INT1 to FALLING edge interrupts()
  //EIMSK = _BV(INT0) | _BV(INT1);    // TURNS ON BOTH INT0 AND INT1
  PCICR = _BV(PCIE2);                                   // Enable Pin Change Interrupts for PORT D pins
  PCMSK2 = _BV(PCINT19) | _BV(PCINT18) | _BV(PCINT20);  // Enables RD2 - RD4 as interrupts
  sei();                                                // Enable global interrupts                                              // Turn on interrupts globally // not required for us, since arduino does it
}

void loop() {
  checkFlags();

  refresh();
  delay(100);
}

void refresh() {
  // set the cursor to column 0, line 1
  lcd.setCursor(0, 0);
  lcd.print(yellowTenthsSecond / 10);
  lcd.print(".");
  lcd.print(yellowTenthsSecond % 10);

  if (!isGreenRunning) {
  lcd.setCursor(0, 1);
  lcd.print(greenCounter/1000);
  lcd.print(".");
  lcd.print((greenCounter/100)%10);
  }
}

void checkFlags() {
  // check for yellow isr variable
  if (mainEventFlags & FLAG_YELLOW_PUSHBUTTON) {
    delay(16);
    mainEventFlags &= ~FLAG_YELLOW_PUSHBUTTON;
    if (bit_is_clear(REG_PIN_PUSHBUTTON_YELLOW, BIT_PUSHBUTTON_YELLOW)) {
      //DO THE ACTION!
      isYellowRunning = !isYellowRunning;
      REG_PORT_LED_YELLOW ^= _BV(BIT_LED_YELLOW);
    }
  }

  //check for GREEN isr variable
  if (mainEventFlags & FLAG_GREEN_PUSHBUTTON) {
    delay(16);
    mainEventFlags &= ~FLAG_GREEN_PUSHBUTTON;
    if (bit_is_clear(REG_PIN_PUSHBUTTON_GREEN, BIT_PUSHBUTTON_GREEN)) {
      //DO THE ACTION!
      isGreenRunning = !isGreenRunning;
      REG_PORT_LED_GREEN ^= _BV(BIT_LED_GREEN);
      if (isGreenRunning) {
        lcd.clear();
      }
    }
  }

  //check for BLUE isr variable
  if (mainEventFlags & FLAG_BLUE_PUSHBUTTON) {
    delay(16);
    mainEventFlags &= ~FLAG_BLUE_PUSHBUTTON;
    if (bit_is_clear(REG_PIN_PUSHBUTTON_BLUE, BIT_PUSHBUTTON_BLUE)) {
      //DO THE ACTION!
      REG_PORT_LED_GREEN &= ~_BV(BIT_LED_GREEN);
      REG_PORT_LED_YELLOW &= ~_BV(BIT_LED_YELLOW);
      yellowTenthsSecond = 0;
      greenCounter = 0;
      isYellowRunning = false;
      isGreenRunning = false;
      lcd.clear();
    }
  }
}

ISR(PCINT2_vect) {
  //ISR called means that RD0, RD1, RD2
  uint8_t changedbits = PIND ^ portdhistory;
  portdhistory = PIND;
  if (changedbits & _BV(BIT_PUSHBUTTON_YELLOW)) {
    if (bit_is_clear(REG_PIN_PUSHBUTTON_YELLOW, BIT_PUSHBUTTON_YELLOW)) {
      mainEventFlags |= FLAG_YELLOW_PUSHBUTTON;
    }
  }
  if (changedbits & _BV(BIT_PUSHBUTTON_GREEN)) {
    if (bit_is_clear(REG_PIN_PUSHBUTTON_GREEN, BIT_PUSHBUTTON_GREEN)) {
      mainEventFlags |= FLAG_GREEN_PUSHBUTTON;
    }
  }
  if (changedbits & _BV(BIT_PUSHBUTTON_BLUE)) {
    if (bit_is_clear(REG_PIN_PUSHBUTTON_BLUE, BIT_PUSHBUTTON_BLUE)) {
      mainEventFlags |= FLAG_BLUE_PUSHBUTTON;
    }
  }
}

ISR(TIMER1_COMPA_vect) {
    TCNT1 = TIMER_1_START;
  if (isYellowRunning) {
    yellowTenthsSecond++;
  }
}

ISR(TIMER2_COMPA_vect) {
    TCNT2 = TIMER_2_START;
  if (isGreenRunning) {
    greenCounter++;
  }
}
