// Adafruit_4416_PI_Controller.ino
#include <Arduino.h>

const uint8_t ENC_A=2, ENC_B=3;
const uint8_t DIR_PIN=9, PWM_PIN=10;

const float GEAR_RATIO=20.4f;
const int CPR_X4=28; // 7 cycles/channel -> 28 counts/rev using x4 decoding

const float KP=2.0f;
const float KI=5.0f;
const uint32_t Ts_ms=20;

volatile long encCount=0;
volatile uint8_t prevState=0;

float targetRPM=100;
float rpmOut=0;
float integ=0;
long lastCount=0;
uint32_t lastT=0;

const int8_t qem[16]={
0,-1,1,0,
1,0,0,-1,
-1,0,0,1,
0,1,-1,0};

void encoderISR(){
  uint8_t s=(digitalRead(ENC_A)<<1)|digitalRead(ENC_B);
  uint8_t idx=(prevState<<2)|s;
  encCount+=qem[idx];
  prevState=s;
}

void setMotor(float u){
  bool dir=u>=0;
  int pwm=constrain((int)fabs(u),0,255);
  digitalWrite(DIR_PIN,dir);
  analogWrite(PWM_PIN,pwm);
}

void setup(){
  Serial.begin(115200);
  pinMode(ENC_A,INPUT);
  pinMode(ENC_B,INPUT);
  pinMode(DIR_PIN,OUTPUT);
  pinMode(PWM_PIN,OUTPUT);
  prevState=(digitalRead(ENC_A)<<1)|digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A),encoderISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B),encoderISR,CHANGE);
  lastT=millis();
}

void loop(){
  if(Serial.available()) targetRPM=Serial.parseFloat();
  uint32_t now=millis();
  if(now-lastT>=Ts_ms){
    float dt=(now-lastT)/1000.0f;
    lastT=now;
    noInterrupts();
    long c=encCount;
    interrupts();
    long dc=c-lastCount;
    lastCount=c;

    float motorRPM=((float)dc/CPR_X4)*(60.0f/dt);
    rpmOut=motorRPM/GEAR_RATIO;

    float err=targetRPM-rpmOut;
    float newInt=integ+err*dt;
    float u=KP*err+KI*newInt;
    if(u<255 && u>-255) integ=newInt;
    u=KP*err+KI*integ;
    u=constrain(u,-255.0f,255.0f);
    setMotor(u);

    static int div=0;
    if(++div>=5){
      div=0;
      Serial.print("Target=");
      Serial.print(targetRPM);
      Serial.print(" OutRPM=");
      Serial.print(rpmOut,1);
      Serial.print(" MotorRPM=");
      Serial.print(motorRPM,0);
      Serial.print(" Counts=");
      Serial.print(c);
      Serial.print(" dC=");
      Serial.print(dc);
      Serial.print(" PWM=");
      Serial.println(u);
    }
  }
}
