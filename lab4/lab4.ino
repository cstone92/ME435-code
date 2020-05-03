
// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

// Output PORTs and BITs
//#define REG_PORT_LED_RED PORTB
//#define BIT_LED_RED 2
#define REG_PORT_LED_YELLOW PORTD
#define BIT_LED_YELLOW 7
#define REG_PORT_LED_GREEN PORTD
#define BIT_LED_GREEN 6
//#define REG_PORT_LED_BLUE PORTD
//#define BIT_LED_BLUE 5

// Input PORTs and PIN reg and BITs
//#define REG_PORT_PUSHBUTTON_RED PORTD
//#define REG_PIN_PUSHBUTTON_RED PIND
//#define BIT_PUSHBUTTON_RED 3
#define REG_PORT_PUSHBUTTON_YELLOW PORTD
#define REG_PIN_PUSHBUTTON_YELLOW PIND
#define BIT_PUSHBUTTON_YELLOW 3
#define REG_PORT_PUSHBUTTON_GREEN PORTD
#define REG_PIN_PUSHBUTTON_GREEN PIND
#define BIT_PUSHBUTTON_GREEN 2
#define REG_PORT_PUSHBUTTON_BLUE PORTD
#define REG_PIN_PUSHBUTTON_BLUE PIND
#define BIT_PUSHBUTTON_BLUE 4

#define POTENTIOMETER_PIN A0

#define ON HIGH
#define OFF LOW

#define PRESSED LOW
#define NOTPRESSED HIGH

volatile uint8_t mainEventFlags = 0;
//#define FLAG_RED_PUSHBUTTON 0x01
#define FLAG_YELLOW_PUSHBUTTON 0X02
#define FLAG_GREEN_PUSHBUTTON 0X04
#define FLAG_BLUE_PUSHBUTTON 0x08

volatile uint8_t portdhistory = 0xFF;

uint8_t yellowCount = 0;
uint8_t greenCount = 0;
uint16_t time = 0;
uint16_t newTime = 0;

String bottomRow;
String timeNum;
String yellowNum;
String greenNum;

boolean updateLCD = false;      // triggered whenever something changes on display to refresh and updateLCD

void setup() {
  Serial.begin(9600);

  //////////////// initialize display  ///////////
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Carson Stone");

  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // form string to put on LCD display
  yellowNum = String(yellowCount, DEC);
  greenNum = String(greenCount, DEC);
  timeNum = String(time, DEC);

  bottomRow = String("Y=" + yellowNum + " G=" + greenNum + " T=" + time);
  lcd.print(bottomRow);

  /////////////////// setup I/O ///////////////////
  //DDRB |= _BV(BIT_LED_RED);     //pinMode(PIN_LED_RED, OUTPUT);  // output = 1 input = 0
  DDRB |= _BV(BIT_LED_YELLOW);  //pinMode(PIN_LED_YELLOW, OUTPUT);
  DDRD |= _BV(BIT_LED_GREEN);   //pinMode(PIN_LED_GREEN, OUTPUT);
  //DDRD |= _BV(BIT_LED_BLUE);    //pinMode(PIN_LED_BLUE, OUTPUT);
  // all other bits are left as input which is default

  // enable the pullup resistors
  //REG_PORT_PUSHBUTTON_RED |= _BV(BIT_PUSHBUTTON_RED);
  REG_PORT_PUSHBUTTON_YELLOW |= _BV(BIT_PUSHBUTTON_YELLOW);
  REG_PORT_PUSHBUTTON_GREEN |= _BV(BIT_PUSHBUTTON_GREEN);
  REG_PORT_PUSHBUTTON_BLUE |= _BV(BIT_PUSHBUTTON_BLUE);
  // pinMode(PIN_PUSHBUTTON_RED, INPUT_PULLUP);
  // pinMode(PIN_PUSHBUTTON_YELLOW, INPUT_PULLUP);
  // pinMode(PIN_PUSHBUTTON_GREEN, INPUT_PULLUP);
  // pinMode(PIN_PUSHBUTTON_BLUE, INPUT_PULLUP);

  /////////////////// setup interrupts //////////////////////
  //attachInterrupt(digitalPinToInterrupt(PIN_PUSHBUTTON_YELLOW), yellow_pushbutton_isr, FALLING);
  //attachInterrupt(digitalPinToInterrupt(PIN_PUSHBUTTON_RED), red_pushbutton_isr, FALLING);
  //EICRA = _BV(ISC11) | _BV(ISC01);  // set INT0 and INT1 to FALLING edge interrupts()
  //EIMSK = _BV(INT0) | _BV(INT1);    // TURNS ON BOTH INT0 AND INT1
  PCICR = _BV(PCIE2);                                   // Enable Pin Change Interrupts for PORT D pins
  PCMSK2 = _BV(PCINT19) | _BV(PCINT18) | _BV(PCINT20);  // Enables RD2 - RD4 as interrupts
  sei();                                                // Turn on interrupts globally // not required for us, since arduino does it
}

void loop() {
  // check for yellow isr variable
  if (mainEventFlags & FLAG_YELLOW_PUSHBUTTON) {
    delay(16);
    mainEventFlags &= ~FLAG_YELLOW_PUSHBUTTON;
    if (bit_is_clear(REG_PIN_PUSHBUTTON_YELLOW, BIT_PUSHBUTTON_YELLOW)) {
      //DO THE ACTION!
      REG_PORT_LED_YELLOW |= _BV(BIT_LED_YELLOW);
      REG_PORT_LED_GREEN &= ~_BV(BIT_LED_GREEN);
      Serial.println("Press yellow");
      yellowCount++;
      updateLCD = true;
    }
  }

  //check for GREEN isr variable
  if (mainEventFlags & FLAG_GREEN_PUSHBUTTON) {
    delay(16);
    mainEventFlags &= ~FLAG_GREEN_PUSHBUTTON;
    if (bit_is_clear(REG_PIN_PUSHBUTTON_GREEN, BIT_PUSHBUTTON_GREEN)) {
      //DO THE ACTION!
      REG_PORT_LED_GREEN |= _BV(BIT_LED_GREEN);
      REG_PORT_LED_YELLOW &= ~_BV(BIT_LED_YELLOW);
      Serial.println("Press green");
      greenCount++;
      updateLCD = true;
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
      yellowCount = 0;
      greenCount = 0;
      updateLCD = true;
    }
  }

  // keep track of time
  newTime = millis() / 1000;  // String of the number of seconds since reset:
  if (newTime > time) {
    updateLCD = true;
    time++;
    if (!(newTime % 2)) {
      Serial.println(analogRead(POTENTIOMETER_PIN));
    }
  }

  // see if something changed and if so, update the display
  if (updateLCD) {
    refresh();
  } else {
    delay(50);
  }
}

void refresh() {
  lcd.clear();
  lcd.print("Carson Stone");
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);

  // form string to put on LCD display
  yellowNum = String(yellowCount, DEC);
  greenNum = String(greenCount, DEC);
  timeNum = String(time, DEC);

  bottomRow = String("Y=" + yellowNum + " G=" + greenNum + " T=" + time);
  lcd.print(bottomRow);

  updateLCD = false;
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
