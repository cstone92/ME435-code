#define PIN_LED_RED 10
#define PIN_LED_YELLOW 9
#define PIN_LED_GREEN 6
#define PIN_LED_BLUE 5

#define PIN_PUSHBUTTON_RED 3
#define PIN_PUSHBUTTON_YELLOW 2
#define PIN_PUSHBUTTON_GREEN 1
#define PIN_PUSHBUTTON_BLUE 0

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

#define ON 1
#define OFF 0

#define PRESSED 0
#define NOTPRESSED 1

uint8_t blueState = HIGH;
uint8_t lastBlueState = HIGH;
uint8_t greenState = HIGH;
uint8_t lastGreenState = HIGH;

uint8_t currentIndex = 0;
uint8_t savedLeds[10] = {PIN_LED_BLUE,
                         PIN_LED_BLUE,
                         PIN_LED_BLUE,
                         PIN_LED_BLUE,
                         PIN_LED_BLUE,
                         PIN_LED_BLUE,
                         PIN_LED_BLUE,
                         PIN_LED_BLUE,
                         PIN_LED_BLUE,
                         PIN_LED_BLUE};

volatile uint8_t mainEventFlags = 0;
#define FLAG_YELLOW_PUSHBUTTON 0X01
#define FLAG_RED_PUSHBUTTON 0x02

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

  // update this later
  attachInterrupt(digitalPinToInterrupt(PIN_PUSHBUTTON_YELLOW), yellow_pushbutton_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_PUSHBUTTON_RED), red_pushbutton_isr, FALLING);
}

void loop() {
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

void loop2() {
  // give feedback when a button is pressed
  digitalWrite(PIN_LED_RED, !digitalRead(PIN_PUSHBUTTON_RED));
  digitalWrite(PIN_LED_YELLOW, !digitalRead(PIN_PUSHBUTTON_YELLOW));
  digitalWrite(PIN_LED_GREEN, (digitalRead(PIN_PUSHBUTTON_GREEN) == PRESSED));
  digitalWrite(PIN_LED_BLUE, (digitalRead(PIN_PUSHBUTTON_BLUE) == PRESSED));

  if (currentIndex < 10) {
    //check for yellow isr variable
    if (mainEventFlags & FLAG_YELLOW_PUSHBUTTON) {
      delay(60);
      mainEventFlags &= ~FLAG_YELLOW_PUSHBUTTON;
      if (digitalRead(PIN_PUSHBUTTON_YELLOW) == PRESSED) {
        //DO THE ACTION!
        //add a yellow led to the saed array at the current location
        savedLeds[currentIndex] = PIN_LED_YELLOW;
        currentIndex++;
      }
    }

    //check for RED isr variable
    if (mainEventFlags & FLAG_RED_PUSHBUTTON) {
      delay(60);
      mainEventFlags &= ~FLAG_RED_PUSHBUTTON;
      if (digitalRead(PIN_PUSHBUTTON_RED) == PRESSED) {
        //DO THE ACTION!
        //add a red led to the saed array at the current location
        savedLeds[currentIndex] = PIN_LED_RED;
        currentIndex++;
      }
    }

    greenState = digitalRead(PIN_PUSHBUTTON_GREEN);
    if (greenState != lastGreenState) {
      if (greenState == PRESSED) {
        //DO THE ACTION!
        //add a green led to the saed array at the current location
        savedLeds[currentIndex] = PIN_LED_GREEN;
        currentIndex++;
      }
      delay(50);
    }
    lastGreenState = greenState;
  }

  blueState = digitalRead(PIN_PUSHBUTTON_BLUE);
  if (blueState != lastBlueState) {
    if (blueState == PRESSED) {
      //DO THE ACTION!
      // run sequence
      // reset the array to all blues
      // reset the array index
      runSequence();
      currentIndex = 0;
      for (int index = 0; index < 10; index++) {
        savedLeds[index] = PIN_LED_BLUE;
      }
    }
    delay(50);
  }
  lastBlueState = blueState;
}

void runSequence() {
  // for loop for the array
  // get the current value
  // light that value for 1000ms
  // turn it off for 100ms

  digitalWrite(PIN_LED_RED, OFF);
  digitalWrite(PIN_LED_YELLOW, OFF);
  digitalWrite(PIN_LED_GREEN, OFF);
  digitalWrite(PIN_LED_BLUE, OFF);

  for (int index = 0; index < 10; index++) {
    digitalWrite(savedLeds[index], ON);
    delay(1000);
    digitalWrite(savedLeds[index], OFF);
    delay(100);
  }
}

void yellow_pushbutton_isr() {
  mainEventFlags |= FLAG_YELLOW_PUSHBUTTON;
}

void red_pushbutton_isr() {
  mainEventFlags |= FLAG_RED_PUSHBUTTON;
}
