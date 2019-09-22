int LED = 21;

void setup() {
  int LED = 21;
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
}

    void letterrs(){
      for(int i=0; i<3; i++){
        digitalWrite(LED, HIGH);   
        delay(1000);
        digitalWrite(LED, LOW);   
        delay(1000);
      }
    }
      
      void letterro() {
       for(int i=0; i<3; i++){
        digitalWrite(LED, HIGH);   
        delay(3000);
        digitalWrite(LED, LOW);   
        delay(3000);
        }
      }
      
void loop() {
  // put your main code here, to run repeatedly:
       letterrs;   
       letterro;
       letterrs;
}
