#define PIN_LED_RED 10
#define PIN_LED_YELLOW 9
#define PIN_LED_GREEN 6
#define PIN_LED_BLUE 5

#define PIN_PUSHBUTTON_RED 3
#define PIN_PUSHBUTTON_YELLOW 2
#define PIN_PUSHBUTTON_GREEN 1
#define PIN_PUSHBUTTON_BLUE 0
#define ON 1
#define OFF 0

#define PRESSED 0
#define NOTPRESSED 1

uint8_t blueState = LOW;
uint8_t lastBlueState = LOW;
uint8_t greenState = LOW;
uint8_t lastGreenState = LOW;

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

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);


  pinMode(PIN_PUSHBUTTON_RED, INPUT_PULLUP);
  pinMode(PIN_PUSHBUTTON_YELLOW, INPUT_PULLUP);
  pinMode(PIN_PUSHBUTTON_GREEN, INPUT_PULLUP);
  pinMode(PIN_PUSHBUTTON_BLUE, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_PUSHBUTTON_YELLOW), yellow_pushbutton_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_PUSHBUTTON_RED), red_pushbutton_isr, FALLING);
}

void loop() {
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
