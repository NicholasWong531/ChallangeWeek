int buttonPin = PC_13;
int ledPin = LED_BUILTIN;
volatile int buttonState = 0;

void sLetter(){
       for(int i = 0; i < 3; i++){
              digitalWrite(LED_BUILTIN, HIGH);   
              delay(250);                       // wait for a second
              digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
              delay(250);
       }
}
        
void oLetter() {
       for(int i = 0; i < 3; i++){
              digitalWrite(LED_BUILTIN, HIGH);   
              delay(750);                       
              digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
              delay(250); 
            }
      }

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PC_13, INPUT);
  attachInterrupt(0, pin_ISR, CHANGE);
}

void loop() {
  buttonState = digitalRead(buttonPin);
       sLetter();   
       oLetter();
       sLetter();
       delay(3000);
}
void pin_ISR(){
       if (buttonState == HIGH) {
              for (int i = 0; i < 10; i++){
                     digitalWrite(LED_BUILTIN, HIGH);
                     delay(100);                       
                     digitalWrite(LED_BUILTIN, LOW);    
                     delay(100);
              }
    }
}
