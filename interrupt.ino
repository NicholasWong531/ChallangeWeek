int buttonPin = PC_13;
int ledPin = LED_BUILTIN;
volatile int buttonState = 0;

void letterrs(){
       digitalWrite(LED_BUILTIN, HIGH);   
       delay(250);                       // wait for a second
       digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
       delay(250); 
       digitalWrite(LED_BUILTIN, HIGH);
       delay(250);
       digitalWrite(LED_BUILTIN, LOW);
       delay(250);
       digitalWrite(LED_BUILTIN, HIGH);
       delay(250);
       digitalWrite(LED_BUILTIN, LOW);
       delay(250);
        }
        
      void letterro() {
       digitalWrite(LED_BUILTIN, HIGH);   
       delay(750);                       
       digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
       delay(250); 
       digitalWrite(LED_BUILTIN, HIGH);
       delay(750);
       digitalWrite(LED_BUILTIN, LOW);
       delay(250);
       digitalWrite(LED_BUILTIN, HIGH);
       delay(750);
       digitalWrite(LED_BUILTIN, LOW);
       delay(250);
      }
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PC_13, INPUT);
  attachInterrupt(0, pin_ISR, CHANGE);
}

void loop() {
  buttonState = digitalRead(buttonPin);
       letterrs();   
       letterro();
       letterrs();
       delay(3000);
}
void pin_ISR(){
if (buttonState == HIGH) {
       for (x=0, x<10, x++)
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);                       
      digitalWrite(LED_BUILTIN, LOW);    
      delay(100);
}
     
    }}
