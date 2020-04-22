// #define PIN_LED_RED			10
// #define PIN_LED_YELLOW		9
// #define PIN_LED_GREEN		6
// #define PIN_LED_BLUE		5

// #define PIN_PUSHBUTTON_BLUE 0

//#define PRESSED 0
//#define UNPRESSED 1

 //#define greenState 		0
 //#define yellowState 	1
 //#define redState 		2

byte state = 0;

void setup()
{
  //Serial.begin(9600);
  DDRB |= _BV(DDB2); //pinMode(PIN_LED_RED, OUTPUT);  // output = 1 input = 0
  DDRB |= _BV(DDB1); //pinMode(PIN_LED_YELLOW, OUTPUT);
  DDRD |= _BV(DDD6); //pinMode(PIN_LED_GREEN, OUTPUT);

  //DDRB = 0x06;         // set pins 1 and 2 as outpus, all others as inputs;
  //DDRD = 0x40;          // set pin 6 as outputs and all others as inputs

  // takes 2 lines to do this library function // pinMode(PIN_PUSHBUTTON_BLUE, INPUT_PULLUP);
  DDRD &= ~_BV(DDD0);   // make it an input  // this is automatically the default
  PORTD |= _BV(PORTD0); // make it a pull up resistor enabled
}

void loop()
{

  //Serial.println(state);
  //delay(100); // Wait for 100 milliseconds to wait for display

  if (!(PIND & 0x01))
  {
    state = 0;
    //Serial.println("Pressing button");
    delay(100); // Wait for 100 milliseconds to wait for display
  }

  switch (state)
  {
  case 0:
    PORTB &= ~_BV(PORTB2); // replaces digitalWrite(PIN_LED_RED, LOW);
    PORTB &= ~_BV(PORTB1); //digitalWrite(PIN_LED_YELLOW, LOW);
    PORTD |= _BV(PORTD6);  //digitalWrite(PIN_LED_GREEN, HIGH);
    delay(2000);           //wait two seconds
    state = 1;
    break;
  case 1:
    PORTB &= ~_BV(PORTB2); // replaces digitalWrite(PIN_LED_RED, LOW);
    PORTB |= _BV(PORTB1);  //digitalWrite(PIN_LED_YELLOW, HIGH);
    PORTD &= ~_BV(PORTD6); //digitalWrite(PIN_LED_GREEN, LOW);
    delay(1000);           //wait one seconds
    state = 2;
    break;
  case 2:
    PORTB |= _BV(PORTB2);  //digitalWrite(PIN_LED_RED, HIGH);
    PORTB &= ~_BV(PORTB1); //digitalWrite(PIN_LED_YELLOW, LOW);
    PORTD &= ~_BV(PORTD6); //digitalWrite(PIN_LED_GREEN, LOW);
    break;
  default:
    //Serial.println("Catch-all for de-bugging");
    delay(100); // Wait for 100 milliseconds to wait for display
    break;
  }
}