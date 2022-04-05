#include <IBIT.h>
#include <IBIT_Analog.h>
#include <IBIT_Motor.h>
#include <IBIT_Servo.h>
#include <mbitServo.h>
#include <IBIT.h>
int min0 = 220; int min1 = 160; int min2 = 140; int min3 = 110; int min4 = 110; int min5 = 120; //
int max0 = 1150; int max1 = 1180; int max2 = 1400; int max3 = 990; int max4 = 1150; int max5 = 1100; //
int sen1min,sen1max,sen2min,sen2max,sen3min,sen3max,sen4min,sen4max,sen5min,sen5max,sen6min,sen6max;  
float sen0 = 0; float sen1 = 0; float sen2 = 0; float sen3 = 0; float sen4 = 0; float sen5 = 0;  
float kp = 0.0725; 
float kd = 0.2;
double lasterror = 0;
int state = 0;        
int calibrated = 0;
int pos = 0;
int posservo = 90;
const int buttonA = 5;     // the number of the pushbutton pin
const int buttonB = 11;     // the number of the pushbutton pin
int servoLout = 142;
int servoLin = 111;
int servoRout = 53;
int servoRin = 88;

void setup() {
  IBIT();
  pinMode(buttonA, INPUT);  
  pinMode(buttonB, INPUT); 
  Serial.begin(115200);
  Serial.println(" Hello ");
  //outGrip();
  
  delay(200);
  
  
}


void loop() {
  //program here 
    //if(!digitalRead(buttonA)){
    //Serial.println(digitalRead(buttonA));
  //}
  Run(150);
//sensorTest();
}

void sensorTest(){
   Serial.print(analog(A0));
   Serial.print("   ");
   Serial.print(analog(A1));
   Serial.print("   ");
   Serial.print(analog(A2));
   Serial.print("   ");
   Serial.print(analog(A3));
   Serial.print("   ");
   Serial.print(analog(A4));
   Serial.print("   ");
   Serial.print(analog(A5));
   Serial.println("   ");
   delay(5000);
}
void inGrip(){

  servo(1,servoLin);
  servo(2,servoRin);
}

void outGrip(){
  servo(1,servoLout);
  servo(2,servoRout);
  
}

void servoset(){
 if (! digitalRead(buttonA)) {
  posservo++;
  }
  if (! digitalRead(buttonB)) {
    posservo--;
  }
  Serial.println(posservo);
  servo(1,posservo);
  delay(50);
}
float sen00(){
  sen0 = analog(A0);
  sen0 = map(sen0,min0,max0-300,1000,0); 
  sen0 = constrain(sen0,0,1000);
  return sen0;
}
float sen11(){
  sen1 = analog(A1);
  sen1 = map(sen1,min1,max1-300,1000,0); 
  sen1 = constrain(sen1,0,1000);
  return sen1;
}
float sen22(){
  sen2 = analog(A2);
  sen2 = map(sen2,min2,max2-300,1000,0); 
  sen2 = constrain(sen2,0,1000);
  return sen2;
}
float sen33(){
  sen3 = analog(A3);
  sen3 = map(sen3,min3,max3-300,1000,0); 
  sen3 = constrain(sen3,0,1000);
  return sen3;
}
float sen44(){
  sen4 = analog(A4);
  sen4 = map(sen4,min4,max4-300,1000,0); 
  sen4 = constrain(sen4,0,1000);
  return sen4;
}
float sen55(){
  sen5 = analog(A5);
  sen5 = map(sen5,min5,max5-300,1000,0); 
  sen5 = constrain(sen5,0,1000);
  return sen5;
}

void back(int seed){
 
  while(sen33() < 600 || sen44() < 600 || sen55() < 600){
  Run(seed);
  }
  motor(1,-255); motor(2,-120); delay(100);
  motor(1,0); motor(2,0); delay(150);
  motor(1,230); motor(2,-230); delay(100);
  while(sen55() < 600){
    motor(1,230); motor(2,-230);
  }
  motor(1,150); motor(2,-150); delay(100);
  while(sen55() < 600){
    motor(1,150); motor(2,-150);
  }
  motor(1,0); motor(2,0); delay(100);
  lasterror = 0;
  state = 0;
}
void R(int seed){
  
  while(sen33() < 600 || sen44() < 600 || sen55() < 600){
  Run(seed);
  }
  motor(1,-255); motor(2,-255); delay(50);
  motor(1,0); motor(2,0); delay(50);
  motor(1,230); motor(2,-230);delay(90);
  while(sen55() < 600){
    motor(1,230); motor(2,-230);
  }
  motor(1,0); motor(2,0); delay(50);
  lasterror = 0;
  state = 0;
}
void L(int seed){

  while(sen00() < 600 || sen11() < 600 || sen22() < 600){
  Run(seed);
  }
  motor(1,-255); motor(2,-255); delay(50);
  motor(1,0); motor(2,0); delay(50);
  motor(1,-230); motor(2,230);delay(90);
  while(sen00() < 600){
    motor(1,-230); motor(2,230);
  }
  motor(1,0); motor(2,0); delay(50);
  lasterror = 0;
  state = 0;

}

void Run(int seed){
  float pos = (0*sen00() + 1000*sen11() + 2000*sen22() + 3000*sen33() + 4000*sen44() + 5000*sen55())/(sen00()+sen11()+sen22()+sen33()+sen44()+sen55());
  /*if(sen55() > 50){
    state = 3;
  }
  if(sen00() > 50){
    state = 1;
  }
  if(sen11() > 70 || sen22() > 70 || sen33() > 70 || sen44() > 70){
    state = 2;
  }
  if(sen55() < 50  && state == 3){
    pos = 5000;
  }
  if(sen00() < 50 && state == 1){
    pos = 0;
  }
  if(state == 2 && sen00() < 70 && sen11() < 70 && sen22() < 70 && sen33() < 70 && sen44() < 70 && sen55() < 70){
    pos = 2500;
  }*/
  float error = pos - 2500;
  float P = error * kp;
  float D = (error - lasterror) * kd;
  float PID = P+D;
  float motorL = seed + PID +20;
  float motorR = seed - PID -20;
  motorL = constrain(motorL,0,200);
  motorR = constrain(motorR,0,200);
  /*if(motorL >= 170){
    motorL = 170;
  }
  if(motorR >= 170){
    motorR = 170;
  }*/
  motor(1,motorL);
  motor(2,motorR);
  lasterror = error;
}

int Timesince = 0;
void rundelay(int Timer,int seed){
  Timesince = millis();
  while((millis() - Timesince)<Timer){
    Run(seed);
  }
 motor(1,0); motor(2,0);
}

void geb(int seed){

  while(sen33() < 600 || sen44() < 600 || sen55() < 600){
  Run(seed);
  }
  motor(1,-255); motor(2,-255); delay(50);
  motor(1,0); motor(2,0); delay(150);
  inGrip();delay(200);
  motor(1,230); motor(2,-230); delay(190);
  motor(1,150); motor(2,-150); delay(100);
  while(sen55() < 600){
    motor(1,150); motor(2,-150);
  }
  motor(1,0); motor(2,0); delay(100);
  lasterror = 0;
  state = 0;
  
}

void vang(int seed){
  while(sen33() < 600 || sen44() < 600 || sen55() < 600){
  Run(seed);
  }
  motor(1,-255); motor(2,-180); delay(100);
  motor(1,0); motor(2,0); delay(150);
  outGrip();delay(200);
  
}
