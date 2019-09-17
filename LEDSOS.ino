void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
}

    void letterrs(){
       digitalWrite(LED_BUILTIN, HIGH);   
       delay(1000);                       // wait for a second
       digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
       delay(1000); 
       digitalWrite(LED_BUILTIN, HIGH);
       delay(1000);
        }
        
      void letterro() {
       digitalWrite(LED_BUILTIN, HIGH);   
       delay(3000);                       
       digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
       delay(3000); 
       digitalWrite(LED_BUILTIN, HIGH);
       delay(3000);
      }
      
void loop() {
  // put your main code here, to run repeatedly:
       letterrs;   
       letterro;
       letterrs;
}
