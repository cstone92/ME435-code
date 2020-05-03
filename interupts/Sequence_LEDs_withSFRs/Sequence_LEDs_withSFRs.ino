// #define PIN_LED_RED 10
// #define PIN_LED_YELLOW 9
// #define PIN_LED_GREEN 6
// #define PIN_LED_BLUE 5

// #define PIN_PUSHBUTTON_RED 3
// #define PIN_PUSHBUTTON_YELLOW 2
// #define PIN_PUSHBUTTON_GREEN 1
// #define PIN_PUSHBUTTON_BLUE 0

// Output PORTs and BITs
#define REG_PORT_LED_RED PORTB
#define BIT_LED_RED 2
#define REG_PORT_LED_YELLOW PORTB
#define BIT_LED_YELLOW 1
#define REG_PORT_LED_GREEN PORTD
#define BIT_LED_GREEN 6
#define REG_PORT_LED_BLUE PORTD
#define BIT_LED_BLUE 5

// Input PORTs and PIN reg and BITs
#define REG_PORT_PUSHBUTTON_RED PORTD
#define REG_PIN_PUSHBUTTON_RED PIND
#define BIT_PUSHBUTTON_RED 3
#define REG_PORT_PUSHBUTTON_YELLOW PORTD
#define REG_PIN_PUSHBUTTON_YELLOW PIND
#define BIT_PUSHBUTTON_YELLOW 2
#define REG_PORT_PUSHBUTTON_GREEN PORTD
#define REG_PIN_PUSHBUTTON_GREEN PIND
#define BIT_PUSHBUTTON_GREEN 1
#define REG_PORT_PUSHBUTTON_BLUE PORTD
#define REG_PIN_PUSHBUTTON_BLUE PIND
#define BIT_PUSHBUTTON_BLUE 0

#define ON HIGH
#define OFF LOW

#define PRESSED LOW
#define NOTPRESSED HIGH

//uint8_t blueState = HIGH;
//uint8_t lastBlueState = HIGH;
//uint8_t greenState = HIGH;
//uint8_t lastGreenState = HIGH;

uint8_t currentIndex = 0;
uint8_t savedLeds[10] = {BIT_LED_BLUE,
                         BIT_LED_BLUE,
                         BIT_LED_BLUE,
                         BIT_LED_BLUE,
                         BIT_LED_BLUE,
                         BIT_LED_BLUE,
                         BIT_LED_BLUE,
                         BIT_LED_BLUE,
                         BIT_LED_BLUE,
                         BIT_LED_BLUE};

volatile uint8_t mainEventFlags = 0;
#define FLAG_RED_PUSHBUTTON 0x01
#define FLAG_YELLOW_PUSHBUTTON 0X02
#define FLAG_GREEN_PUSHBUTTON 0X04
#define FLAG_BLUE_PUSHBUTTON 0x08

volatile uint8_t portdhistory = 0xFF;

void setup() {
  Serial.begin(9600);

  DDRB |= _BV(BIT_LED_RED);     //pinMode(PIN_LED_RED, OUTPUT);  // output = 1 input = 0
  DDRB |= _BV(BIT_LED_YELLOW);  //pinMode(PIN_LED_YELLOW, OUTPUT);
  DDRD |= _BV(BIT_LED_GREEN);   //pinMode(PIN_LED_GREEN, OUTPUT);
  DDRD |= _BV(BIT_LED_BLUE);    //pinMode(PIN_LED_BLUE, OUTPUT);
  // all other bits are left as input which is default

  // enable the pullup resistors
  REG_PORT_PUSHBUTTON_RED |= _BV(BIT_PUSHBUTTON_RED);
  REG_PORT_PUSHBUTTON_YELLOW |= _BV(BIT_PUSHBUTTON_YELLOW);
  REG_PORT_PUSHBUTTON_GREEN |= _BV(BIT_PUSHBUTTON_GREEN);
  REG_PORT_PUSHBUTTON_BLUE |= _BV(BIT_PUSHBUTTON_BLUE);
  // pinMode(PIN_PUSHBUTTON_RED, INPUT_PULLUP);
  // pinMode(PIN_PUSHBUTTON_YELLOW, INPUT_PULLUP);
  // pinMode(PIN_PUSHBUTTON_GREEN, INPUT_PULLUP);
  // pinMode(PIN_PUSHBUTTON_BLUE, INPUT_PULLUP);

  //attachInterrupt(digitalPinToInterrupt(PIN_PUSHBUTTON_YELLOW), yellow_pushbutton_isr, FALLING);
  //attachInterrupt(digitalPinToInterrupt(PIN_PUSHBUTTON_RED), red_pushbutton_isr, FALLING);
  //EICRA = _BV(ISC11) | _BV(ISC01);  // set INT0 and INT1 to FALLING edge interrupts()
  //EIMSK = _BV(INT0) | _BV(INT1);    // TURNS ON BOTH INT0 AND INT1
  PCICR = _BV(PCIE2);                                                  // Enable Pin Change Interrupts for PORT D pins
  PCMSK2 = _BV(PCINT19) | _BV(PCINT18) | _BV(PCINT17) | _BV(PCINT16);  // Enables RD0 - RD3 as interrupts
  sei();                                                               // Turn on interrupts globally // not required for us, since arduino does it
}

void feedbackLEDs() {
  if (bit_is_clear(REG_PIN_PUSHBUTTON_RED, BIT_PUSHBUTTON_RED)) {
    REG_PORT_LED_RED |= _BV(BIT_LED_RED);
  } else {
    REG_PORT_LED_RED &= ~_BV(BIT_LED_RED);
  }

  if (bit_is_clear(REG_PIN_PUSHBUTTON_YELLOW, BIT_PUSHBUTTON_YELLOW)) {
    REG_PORT_LED_YELLOW |= _BV(BIT_LED_YELLOW);
  } else {
    REG_PORT_LED_YELLOW &= ~_BV(BIT_LED_YELLOW);
  }

  if (bit_is_clear(REG_PIN_PUSHBUTTON_GREEN, BIT_PUSHBUTTON_GREEN)) {
    REG_PORT_LED_GREEN |= _BV(BIT_LED_GREEN);
  } else {
    REG_PORT_LED_GREEN &= ~_BV(BIT_LED_GREEN);
  }

  if (bit_is_clear(REG_PIN_PUSHBUTTON_BLUE, BIT_PUSHBUTTON_BLUE)) {
    REG_PORT_LED_BLUE |= _BV(BIT_LED_BLUE);
  } else {
    REG_PORT_LED_BLUE &= ~_BV(BIT_LED_BLUE);
  }
}

void oldTestLoop() {
  // Test temp loop function
  REG_PORT_LED_RED |= _BV(BIT_LED_RED);
  REG_PORT_LED_YELLOW |= _BV(BIT_LED_YELLOW);
  REG_PORT_LED_GREEN |= _BV(BIT_LED_GREEN);
  REG_PORT_LED_BLUE |= _BV(BIT_LED_BLUE);
}
uin
void loop() {
  // give feedback when a button is pressed
  // digitalWrite(PIN_LED_RED, !digitalRead(PIN_PUSHBUTTON_RED));
  // digitalWrite(PIN_LED_YELLOW, !digitalRead(PIN_PUSHBUTTON_YELLOW));
  // digitalWrite(PIN_LED_GREEN, (digitalRead(PIN_PUSHBUTTON_GREEN) == PRESSED));
  // digitalWrite(PIN_LED_BLUE, (digitalRead(PIN_PUSHBUTTON_BLUE) == PRESSED));
  feedbackLEDs();

  if (currentIndex < 10) {
    //check for yellow isr variable

    //check for RED isr variable
    if (mainEventFlags & FLAG_RED_PUSHBUTTON) {
      delay(30);
      mainEventFlags &= ~FLAG_RED_PUSHBUTTON;
      if (bit_is_clear(REG_PIN_PUSHBUTTON_RED, BIT_PUSHBUTTON_RED)) {
        //DO THE ACTION!
        //REG_PORT_LED_RED ^= _BV(BIT_LED_RED);  //togles
        //add a red led to the send array at the current location
        savedLeds[currentIndex] = BIT_LED_RED;
        currentIndex++;
      }
    }
    if (mainEventFlags & FLAG_YELLOW_PUSHBUTTON) {
      delay(30);
      mainEventFlags &= ~FLAG_YELLOW_PUSHBUTTON;
      if (bit_is_clear(REG_PIN_PUSHBUTTON_YELLOW, BIT_PUSHBUTTON_YELLOW)) {
        //DO THE ACTION!
        //REG_PORT_LED_YELLOW ^= _BV(BIT_LED_YELLOW);  //togles
        //add a yellow led to the send array at the current location
        savedLeds[currentIndex] = BIT_LED_YELLOW;
        currentIndex++;
      }
    }
    //check for GREEN isr variable
    if (mainEventFlags & FLAG_GREEN_PUSHBUTTON) {
      delay(30);
      mainEventFlags &= ~FLAG_GREEN_PUSHBUTTON;
      if (bit_is_clear(REG_PIN_PUSHBUTTON_GREEN, BIT_PUSHBUTTON_GREEN)) {
        //DO THE ACTION!
        //REG_PORT_LED_GREEN ^= _BV(BIT_LED_GREEN);  //togles
        //add a GREEN led to the send array at the current location
        savedLeds[currentIndex] = BIT_LED_GREEN;
        currentIndex++;
      }
    }
  }
  //check for BLUE isr variable
  if (mainEventFlags & FLAG_BLUE_PUSHBUTTON) {
    delay(30);
    mainEventFlags &= ~FLAG_BLUE_PUSHBUTTON;
    if (bit_is_clear(REG_PIN_PUSHBUTTON_BLUE, BIT_PUSHBUTTON_BLUE)) {
      //DO THE ACTION!
      //REG_PORT_LED_BLUE ^= _BV(BIT_LED_BLUE);  //togles

      // run sequence
      // reset the array to all blues
      // reset the array index
      runSequence();
      currentIndex = 0;
      for (int index = 0; index < 10; index++) {
        savedLeds[index] = BIT_LED_BLUE;
      }
    }
  }

  // no polling
  // greenState = bit_is_set(REG_PIN_PUSHBUTTON_GREEN, BIT_PUSHBUTTON_GREEN);
  // if (greenState != lastGreenState) {
  //   if (greenState == PRESSED) {
  //     //DO THE ACTION!
  //     REG_PORT_LED_GREEN ^= _BV(BIT_LED_GREEN);  //togles
  //     //add a green led to the send array at the current location
  //     // savedLeds[currentIndex] = PIN_LED_GREEN;
  //     // currentIndex++;
  //   }
  //   delay(50);
  // }
  // lastGreenState = greenState;
  // no polling
  // blueState = bit_is_set(REG_PIN_PUSHBUTTON_BLUE, BIT_PUSHBUTTON_BLUE);
  // if (blueState != lastBlueState) {
  //   if (blueState == PRESSED) {
  //     //DO THE ACTION!
  //     REG_PORT_LED_BLUE ^= _BV(BIT_LED_BLUE);  //togles
  //     // run sequence
  //     // reset the array to all blues
  //     // reset the array index
  //     // runSequence();
  //     // currentIndex = 0;
  //     // for (int index = 0; index < 10; index++) {
  //     //   savedLeds[index] = PIN_LED_BLUE;
  //     // }
  //   }
  //   delay(50);
  // }
  // lastBlueState = blueState;
}

void runSequence() {
  // for loop for the array
  // get the current value
  // light that value for 1000ms
  // turn it off for 100ms

  //   digitalWrite(PIN_LED_RED, OFF);
  //   digitalWrite(PIN_LED_YELLOW, OFF);
  //   digitalWrite(PIN_LED_GREEN, OFF);
  //   digitalWrite(PIN_LED_BLUE, OFF);

  for (int index = 0; index < 10; index++) {
    uint8_t activeLedBit = savedLeds[index];
    switch (activeLedBit) {
      case BIT_LED_RED:
        REG_PORT_LED_RED |= _BV(activeLedBit);
        delay(1000);
        REG_PORT_LED_RED &= ~_BV(activeLedBit);
        delay(100);
        break;
      case BIT_LED_YELLOW:
        REG_PORT_LED_YELLOW |= _BV(activeLedBit);
        delay(1000);
        REG_PORT_LED_YELLOW &= ~_BV(activeLedBit);
        delay(100);
        break;
      case BIT_LED_GREEN:
        REG_PORT_LED_GREEN |= _BV(activeLedBit);
        delay(1000);
        REG_PORT_LED_GREEN &= ~_BV(activeLedBit);
        delay(100);
        break;
      case BIT_LED_BLUE:
        REG_PORT_LED_BLUE |= _BV(activeLedBit);
        delay(1000);
        REG_PORT_LED_BLUE &= ~_BV(activeLedBit);
        delay(100);
        break;
      default:
        Serial.println("Catch-all");
        break;
    }
    //     digitalWrite(savedLeds[index], ON);
    //     delay(1000);
    //     digitalWrite(savedLeds[index], OFF);
    //     delay(100);
  }
}

//void yellow_pushbutton_isr() {
// ISR(INT0_vect) {
//   mainEventFlags |= FLAG_YELLOW_PUSHBUTTON;
// }

//void red_pushbutton_isr() {
// ISR(INT1_vect) {
//   mainEventFlags |= FLAG_RED_PUSHBUTTON;
// }

ISR(PCINT2_vect) {
  //ISR called means that RD0, RD1, RD2

  uint8_t changedbits = PIND ^ portdhistory;
  portdhistory = PIND;
  if (changedbits & _BV(BIT_PUSHBUTTON_RED)) {
    if (bit_is_clear(REG_PIN_PUSHBUTTON_RED, BIT_PUSHBUTTON_RED)) {
      mainEventFlags |= FLAG_RED_PUSHBUTTON;
    }
  }
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