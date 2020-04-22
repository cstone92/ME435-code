
#define PIN_LED_RED			10
#define PIN_LED_YELLOW		9
#define PIN_LED_GREEN		6
#define PIN_LED_BLUE		5

#define PIN_PUSHBUTTON_BLUE 0

#define PRESSED  		0
#define UNPRESSED 		1

#define greenState 		0
#define yellowState 	1
#define redState 		2

byte state = greenState;


void setup()
{
  Serial.begin(9600);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  
  pinMode(PIN_PUSHBUTTON_BLUE, INPUT_PULLUP);
}

void loop()
{
  
  Serial.println(state);
  delay(100); // Wait for 100 milliseconds to wait for display
  
  if (digitalRead(PIN_PUSHBUTTON_BLUE) == PRESSED) {
    state = greenState;
    Serial.println("Pressing button");
    delay(100); // Wait for 100 milliseconds to wait for display
  }
  
  switch (state) {
    case greenState:
  		digitalWrite(PIN_LED_RED, LOW);
  		digitalWrite(PIN_LED_YELLOW, LOW);
  		digitalWrite(PIN_LED_GREEN, HIGH);
    	delay(2000); //wait two seconds
    	state = yellowState;
    break;
    case yellowState:
  		digitalWrite(PIN_LED_RED, LOW);
  		digitalWrite(PIN_LED_YELLOW, HIGH);
  		digitalWrite(PIN_LED_GREEN, LOW);
    	delay(1000); //wait one seconds
    	state = redState;    
    break;
    case redState:
  		digitalWrite(PIN_LED_RED, HIGH);
  		digitalWrite(PIN_LED_YELLOW, LOW);
  		digitalWrite(PIN_LED_GREEN, LOW);    
    break;
    default:
        Serial.println("Catch-all for de-bugging");
    	delay(100); // Wait for 100 milliseconds to wait for display
    break;
  }
}