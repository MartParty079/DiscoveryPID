#include <Arduino.h>

constexpr byte EA=2, EB=3, M1=9, M2=10;
constexpr float DEG=360.0f/(56.0f*(30613.0f/1500.0f));
constexpr float MAXPWM=225.0f, SKP=1.35f, SKI=.40f, SKD=.10f;
volatile long enc=0; volatile byte prev=0;
const int8_t q[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

float kp=1.35f,ki=.4f,kd=.1f,I=0,target=0,pwm=0;
long home=0,last=0; byte mode=0,stable=0; uint32_t tick=0,at300=0;

void isr(){byte s=(digitalRead(EA)<<1)|digitalRead(EB);enc+=q[(prev<<2)|s];prev=s;}
long count(){noInterrupts();long x=enc;interrupts();return x;}
float pos(long c){return (c-home)*DEG;}
void drive(float u){
  int p=constrain((int)lroundf(fabsf(u)),0,225); pwm=u>0?p:u<0?-p:0;
  if(pwm>0){analogWrite(M1,0);analogWrite(M2,p);}
  else if(pwm<0){analogWrite(M2,0);analogWrite(M1,p);}
  else{analogWrite(M1,0);analogWrite(M2,0);}
}
void resetI(){I=0;}
float pid(float e,float v,float dt,bool setup){
  float P=setup?SKP:kp,K=setup?SKI:ki,D=setup?SKD:kd;
  I=constrain(I+e*dt,-2000.0f,2000.0f);
  return constrain(P*e+K*I-D*v,-MAXPWM,MAXPWM);
}
void zero(){home=count();target=0;resetI();mode=0;drive(0);}
void command(String s){
  s.trim();
  if(s=="ZERO"){zero();Serial.println("OK");}
  else if(s=="STOP"){mode=0;drive(0);Serial.println("OK");}
  else if(s=="TEST"){target=300;resetI();mode=1;at300=0;Serial.println("OK");}
  else if(s=="HOME"){target=0;resetI();mode=3;stable=0;Serial.println("OK");}
  else if(s.startsWith("P,")){sscanf(s.c_str()+2,"%f,%f,%f",&kp,&ki,&kd);Serial.println("OK");}
  else if(s=="ID") Serial.println("BAREPID1");
}
void setup(){
  Serial.begin(115200);Serial.setTimeout(10);
  pinMode(EA,INPUT_PULLUP);pinMode(EB,INPUT_PULLUP);pinMode(M1,OUTPUT);pinMode(M2,OUTPUT);
  prev=(digitalRead(EA)<<1)|digitalRead(EB);attachInterrupt(digitalPinToInterrupt(EA),isr,CHANGE);attachInterrupt(digitalPinToInterrupt(EB),isr,CHANGE);
  last=count();zero();Serial.println("BAREPID1");
}
void loop(){
  if(Serial.available()) command(Serial.readStringUntil('\n'));
  uint32_t now=millis(); if(now-tick<10) return; float dt=(now-tick)/1000.0f; tick=now;
  long c=count(),dc=c-last; last=c; float p=pos(c),e=target-p,v=dc*DEG/dt;
  if(mode==1){
    drive(pid(e,v,dt,true));
    if(fabsf(e)<=1.5f&&fabsf(v)<=12.0f){if(!at300)at300=now;if(now-at300>=850){target=0;resetI();mode=2;Serial.print("RETURN,");Serial.println(now);}}
    else at300=0;
  }else if(mode==2) drive(pid(e,v,dt,false));
  else if(mode==3){
    drive(pid(e,v,dt,true));
    if(labs(c-home)<=2&&dc==0){if(++stable>=5){mode=0;drive(0);Serial.println("HOME");}} else stable=0;
  }
  if(mode){Serial.print("D,");Serial.print(now);Serial.print(",");Serial.print(mode);Serial.print(",");Serial.print(p,3);Serial.print(",");Serial.print(e,3);Serial.print(",");Serial.print(c);Serial.print(",");Serial.println(pwm,0);}
}
