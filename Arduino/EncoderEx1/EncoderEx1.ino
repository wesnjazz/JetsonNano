//The sample code for driving one way motor encoder
// Right Wheel pin 2(interrupt 0) and 5
const byte encoderRpinA = 2;//A pin -> the interrupt pin 0
const byte encoderRpinB = 5;//B pin -> the digital pin 5
// Left Wheel pin 3(interrupt 1) and 6
const byte encoderLpinA = 3;//A pin -> the interrupt pin 1
const byte encoderLpinB = 6;//B pin -> the digital pin 6

byte encoderRPinALast;
byte encoderLPinALast;
int durationR;//the number of the pulses
int durationL;//the number of the pulses
boolean Direction;//the rotation direction


void setup()
{
  Serial.begin(57600);//Initialize the serial port
  EncoderInit();//Initialize the module
}

void loop()
{
//  Serial.sprintf("Pulse R:%3d L:%3d", durationR, durationL);
  Serial.print("Pulse R:");
  Serial.print(durationR);
  Serial.print(" L:");
  Serial.println(durationL);
//  Serial.println(duration);
  durationR = 0;
  durationL = 0;
  delay(100);
}

void EncoderInit()
{
  Direction = true;//default -> Forward
  pinMode(encoderRpinB,INPUT);
  pinMode(encoderLpinB,INPUT);
  attachInterrupt(0, wheelSpeedR, CHANGE);
  attachInterrupt(1, wheelSpeedL, CHANGE);  
}

void wheelSpeedR()
{
  int Lstate = digitalRead(encoderRpinA);
  if((encoderRPinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoderRpinB);
    if(val == LOW && Direction)
    {
      Direction = false; //Reverse
    }
    else if(val == HIGH && !Direction)
    {
      Direction = true;  //Forward
    }
  }
  encoderRPinALast = Lstate;

  if(!Direction)  durationR++;
  else  durationR--;
}

void wheelSpeedL()
{
  int Lstate = digitalRead(encoderLpinA);
  if((encoderLPinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoderLpinB);
    if(val == LOW && Direction)
    {
      Direction = false; //Reverse
    }
    else if(val == HIGH && !Direction)
    {
      Direction = true;  //Forward
    }
  }
  encoderLPinALast = Lstate;

  if(Direction)  durationL++;
  else  durationL--;
}
