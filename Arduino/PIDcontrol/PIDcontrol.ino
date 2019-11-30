#include <PID_v1.h>

const byte encoderRpinB = 2; // B pin -> the interrupt pin 0
const byte encoderRpinA = 5; // A pin -> the digital pin 5
const byte encoderLpinB = 3; // B pin -> the interrupt pin 1
const byte encoderLpinA = 6; // A pin -> the digital pin 6
int E_left = 5;
int M_left = 4;
byte encoderRPinBLast;
byte encoderLPinBLast;
double duration, abs_duration; // the number of pulses
boolean Direction; // the rotation direction
boolean result;

double val_output; // Power supplied to the motor PWM value.
double Setpoint;
double Kp=0.6, Ki=5, Kd=0;
PID myPID(&abs_duration, &val_output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  Serial.begin(57600);
  pinMode(M_left, OUTPUT);
  pinMode(E_left, OUTPUT);
  Setpoint = 00; // Set the output value of the PID
  myPID.SetMode(AUTOMATIC); // PID is sert to automatic mode
  myPID.SetSampleTime(100); // Set PID sampling frequency is 100ms
  EncoderInit();
}

void loop() {

}

void EncoderInit() {
  Direction = true; // default -> forward
  pinMode(encoderRpinB, INPUT);
  attachInterrupt(0, wheelSpeed, CHANGE);
}

void wheelSpeed() {
  
}
