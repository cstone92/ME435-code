#define PIN_LED_RED       10
#define PIN_LED_YELLOW      9
#define PIN_LED_GREEN       6
#define PIN_LED_BLUE      5

#define PIN_PUSHBUTTON_RED    3   // can be used for INT1
#define PIN_PUSHBUTTON_YELLOW 2   // can be used for INT0 
#define PIN_PUSHBUTTON_GREEN  1
#define PIN_PUSHBUTTON_BLUE   0

#define ON      1
#define OFF     0

#define PRESSED   0
#define NOTPRESSED  1


void setup()
{
  Serial.begin(9600);
  
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  
  pinMode(PIN_PUSHBUTTON_RED, INPUT_PULLUP);
  pinMode(PIN_PUSHBUTTON_YELLOW, INPUT_PULLUP);
  pinMode(PIN_PUSHBUTTON_GREEN, INPUT_PULLUP);
  pinMode(PIN_PUSHBUTTON_BLUE, INPUT_PULLUP);

}

void loop()
{
  digitalWrite(PIN_LED_RED, (digitalRead(PIN_PUSHBUTTON_RED) == PRESSED));
  digitalWrite(PIN_LED_YELLOW, (digitalRead(PIN_PUSHBUTTON_YELLOW) == PRESSED));
  digitalWrite(PIN_LED_GREEN, (digitalRead(PIN_PUSHBUTTON_GREEN) == PRESSED));
  digitalWrite(PIN_LED_BLUE, (digitalRead(PIN_PUSHBUTTON_BLUE) == PRESSED));               
}
