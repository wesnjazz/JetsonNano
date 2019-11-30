/***
 * 2019.11.29.Fri
 * 
 * Arduino digital input 0~7 : PIND 0b00000000
 * 
 * R Wheel uses pin 2 for encoder B (interrupt) and 5 for encoder A of FIT0450 'TT' motor.
 * L Wheel uses pin 3 for encoder B (interrupt) and 6 for encoder B.
 * 
 * To detect right wheel's rotation, extract value of pin 2 & 5 only.
 * PIND & 0b00100100 will mask for right wheel values.
 * Shift it by 2 bits to ignore pin 0 & 1.
 * 
 * Maskerd & Shifted value of right wheel only have:
 * 0000 (bin) = 0 (dec) = 
 * 0001 (bin) = 1 (dec) = 
 * 1000 (bin) = 8 (dec) = 
 * 1001 (bin) = 9 (dec) = 
 * 
 * Again, let's ignore middle bits (bit 2 & 3) to express encoder AB:
 * Then, 
 * 0000 becomes 00 (encoder A:0 encoder B:0)
 * 0001 becomes 01 (encoder A:0 encoder B:1)
 * 1001 becomes 11 (encoder A:1 encoder B:1)
 * 1000 becomes 10 (encoder A:1 encoder B:0)
 * 
 * STATE MACHINE:
 * 00  <--> 01
 *  ^       ^
 *  |       |
 *  >       >
 *  10 <--> 11
 *  
 * Interrupting happens only on:
 * 00 to 01 (counterclockwise) or 01 to 00 (clockwise)
 * 11 to 10 (counterclockwise) or 10 to 11 (clockwise)
 * 
 * 00: only on case; 01 -> 00; C(backwards); so count-1
 * 01: only on case; 00 -> 01; CC(forwards); so count+1
 * 10: only on case; 11 -> 10; CC(forwards); so count+1
 * 11: only on case; 10 -> 11; C(backwards); so count-1
 *  
 * We can use those four values as indicies:
 * 10bin(8dec) and 11bin(9dec) has bit 4, so remove it.
 * Bit 4 removed 10bin(8dec) and 11bin(9dec) + 10bin makes
 * those to 2dec and 3dec.
 * Ex) 11bin(9dec) is 1001 in binary
 * remove bit 4 : 0001 in binary
 * add 10(bin)  : 0011 in binary which is 3 in decimal.
 *  
 *  Table of 00, 01, 10, 11 can be of 0, 1, 2, 3 as above.
 *  Table[0]:-1 Table[1]:+1 Table[2]:+1 Table[3]:-1
 *  or change the sign for Left wheel.
 *  
 *  
 *  One rotation = 1350 ticks approx.
 *  One rotation = 21cm approx.
 *  long data type range = 2,147,483,648
 *  2147483648 / 1350 ticks = 1,590,728 rotations
 *  1,590,728 * 21cm = 33,405,301cm = 334.053km
 *  Thus, long data type is large enough to calculate distances.
 */

/*** Right Wheel ***/
const byte encoderRpinB = 2; // interrupt 0
const byte encoderRpinA = 5;
/*** Left Wheel ***/
const byte encoderLpinB = 3; // interrupt 1
const byte encoderLpinA = 6;

static int8_t lookup_table[] = {-1,1,1,-1};
long encoder_count_right = 0;
long encoder_count_left = 0;

void setup() {
  Serial.begin(9600);
  EncoderInit();
}

void loop() {
  checkResetSignal();
  printSerial();
  delay(75);
}

void EncoderInit() {
  pinMode(encoderRpinA,INPUT); // make the pin INPUT mode to read its value
  pinMode(encoderLpinA,INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderRpinB),
                  wheelR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLpinB),
                  wheelL,CHANGE);
}

void checkResetSignal() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r') {
      // Reset encoder counter
      encoder_count_right = 0;
      encoder_count_left = 0;
      // Clear all the remaining buffer
      while (Serial.available() > 0) {
        Serial.read();
        delay(1);
      }
      delay(100);
    }
  }
}

void wheelR() {
  // Read PIND value and extract pin 2 & 5 value, then remove pin0&1 values.
  byte currentR = (PIND & 0b00100100) >> 2;
  
  if (currentR > 0b0010) {
    currentR -= 0b1000;
    currentR += 0b0010;
  }

  encoder_count_right += lookup_table[currentR];
}
  
void wheelL() {
  // Read PIND value and extract pin 3 & 6 value, then remove pin0~2 values.
  byte currentL = (PIND & 0b01001000) >> 3;
  
  if (currentL > 0b0010) {
    currentL -= 0b1000;
    currentL += 0b0010;
  }

  encoder_count_left -= lookup_table[currentL];
}

void printSerial() {
  Serial.print(encoder_count_right);
  Serial.print(",");
  Serial.println(encoder_count_left);

// To minimize delay with several Serial.print,
// Sting class is used.
// But, it turns out using Serial.print is fine.
//  String ticks = String(encoder_count_right) +
//                 "," + String(encoder_count_left) + "\r\n";
//  Serial.print(ticks);
}
