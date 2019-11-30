void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  Serial.println("Hello Pi, This is Elegoo UNO R3 in setup!");
}

void loop() {
  Serial.println("Hello Pi, This is Elegoo UNO R3 in loop...");
  delay(1000);
}
