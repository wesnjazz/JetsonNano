// Right Wheel
const byte encoderRpinA = 2; // interrupt 0
const byte encoderRpinB = 5;

// Left Wheel
const byte encoderLpinA = 3; // interrupt 1
const byte encoderLpinB = 6;

byte interrupt0 = 0;
byte interrupt1 = 0;
byte currentR = 0;
byte currentL = 0;

void setup() {
  Serial.begin(57600);
  EncoderInit();
}

void loop() {
//  Serial.println(digitalRead(4));
//  delay(100);
  int a = PIND;
  Serial.print(digitalRead(7));
  Serial.print(" ");
  Serial.print(digitalRead(0));
  Serial.print("   ");
  if (a<2) Serial.print(B0);
  if (a<4) Serial.print(B0);
  if (a<8) Serial.print(B0);
  if (a<16) Serial.print(B0);
  if (a<32) Serial.print(B0);
  if (a<64) Serial.print(B0);
  if (a<128) Serial.print(B0);
//  if (a<256) Serial.print(B0);
//  if (a<512) Serial.print(B0);
  Serial.print(PIND,BIN);
  Serial.print("  ");
  Serial.print(interrupt0);
  Serial.print(" ");
  Serial.print(interrupt1);
  Serial.print(" --- ");
  Serial.print(currentR);
  Serial.print(" ");
  Serial.println(currentL);
//  Serial.print("  ");
//  Serial.print(digitalRead(2));
//  Serial.println(digitalRead(5));
}

void EncoderInit() {
//  pinMode(encoderRpinB,INPUT);
//  pinMode(encoderLpinB,INPUT);
//  attachInterrupt(digitalPinToInterrupt(encoderRpinA),
//                  wheelR,CHANGE);
//  attachInterrupt(digitalPinToInterrupt(encoderLpinA),
//                  wheelL,CHANGE);
  attachInterrupt(0,wheelR,CHANGE);
  attachInterrupt(1,wheelL,CHANGE);
}

void wheelR() {
  currentR = (PIND & 0b00100100) >> 2;
  
  if (interrupt0 == 0)
    interrupt0 = 1;  
  else
    interrupt0 = 0;
}

void wheelL() {
  if (interrupt1 == 0)
    interrupt1 = 1;  
  else
    interrupt1 = 0;
}
