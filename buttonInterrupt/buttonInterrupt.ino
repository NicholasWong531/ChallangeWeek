void setup() {
  // put your setup code here, to run once:
  // According to the board documentation, the user button is connected to I/O PC13 (pin 2) - defined as USER_BTN
  int BUTTON = USER_BTN;

  pinMode(BUTTON, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON, blinkLED, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void blinkLED(){
  
}
