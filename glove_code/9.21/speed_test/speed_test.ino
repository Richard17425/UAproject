void setup() {
   Serial.begin(2000000);
}

void loop() {
  for(uint8_t i=0;i<202;i++){
    Serial.print(i);
    }
Serial.println("");
}
