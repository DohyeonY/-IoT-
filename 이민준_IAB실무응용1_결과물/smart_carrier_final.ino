#include "HUSKYLENS.h"
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(6, 7);

#define AP 3
#define A1 4
#define A2 5
#define BP 9
#define B1 10
#define B2 11
#define STBY 8

#define STOP 0;
#define LEFT 1;
#define RIGHT 2;

//huskylens variables
int x_center;
int width;
char mode;
int last_state;

//joystick variables
int xAxis=140, yAxis=140;
int motorSpeedA = 0;
int motorSpeedB = 0;
int break_state=0;

//PID constants
double kp = 0.6;
double ki = 0.005;
double kd = 10;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output;
double setPoint = 160;
double pid_i=0;
double rateError;

HUSKYLENS huskylens;
HUSKYLENSResult result;
void motor(int left, int right);

void setup() {
    Serial.begin(115200);
    BTSerial.begin(9600);
    Wire.begin();
    while (!huskylens.begin(Wire))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
    pinMode(AP, OUTPUT);
    pinMode(B1, OUTPUT);
    pinMode(B2, OUTPUT);
    pinMode(BP, OUTPUT);
    pinMode(STBY, OUTPUT);
}

void loop() {
    digitalWrite(STBY, HIGH);
    
    if(BTSerial.available()){
      mode = BTSerial.read();
    }
    
    while(mode == 33){ //!
      if(BTSerial.available()){
        mode = BTSerial.read();
      }
      autoControl();
    }
    
    motor(0, 0);
    
    if(mode == 114){
      xAxis=140;
      yAxis=140;
      while(mode == 114){
        joystickControl();
        if(break_state == 1){
          break_state = 0;
          break;
        }
      }
    }
}

void autoControl(){
  if (!huskylens.request()){
          digitalWrite(STBY, LOW);
          previousTime = millis();
          motor(0, 0);
        }
      else if(!huskylens.isLearned()){
        digitalWrite(STBY, LOW);
        previousTime = millis();
        motor(0, 0);
      }
      else if(!huskylens.available()){
        if(BTSerial.available()){
              mode = BTSerial.read();
        }
        previousTime = millis();
        if(last_state == 0){
          motor(0, 0);
        }
        else{
          motor(constrain(120 - output, 70, 255), constrain(120 + output, 70, 255));
        }
      }
      else{
        while (huskylens.available())
        {
            if(BTSerial.available()){
              mode = BTSerial.read();
              Serial.println(mode);
              if(mode == 113){
                break;
              }
            }
            result = huskylens.read();
            input = (double)result.xCenter;
            
            if(input < 120){
              last_state = LEFT;
            }
            else if(input > 200){
              last_state = RIGHT;
            }
            else{
              last_state = STOP;
            }
            
            if(result.width > 100){
              previousTime = millis();
              motor(0, 0);
              delay(500);
              last_state = STOP;
            }
            else{
              output = computePID(input);
              digitalWrite(STBY, HIGH);
              motor(constrain(120 - output, 70, 255), constrain(120 + output, 70, 255));
            }
          }    
        }
}

void joystickControl(){
  while (BTSerial.available() >= 2) {
    xAxis = BTSerial.read();
    delay(10);
    yAxis = BTSerial.read();
  }
  delay(10);
  if(xAxis < 40 or yAxis < 40){
    break_state = 1;
    mode = 33;
  }
  if (xAxis > 130 && xAxis <150 && yAxis > 130 && yAxis <150){Stop();}
  
  if (yAxis > 130 && yAxis <150){    
    if (xAxis < 130){turnRight();
      motorSpeedA = map(xAxis, 130, 60, 0, 255);
      motorSpeedB = map(xAxis, 130, 60, 0, 255);    
    }
    if (xAxis > 150) {turnLeft();
      motorSpeedA = map(xAxis, 150, 220, 0, 255);
      motorSpeedB = map(xAxis, 150, 220, 0, 255); 
    }
  }
  else{
    if (xAxis > 130 && xAxis <150){   
      if (yAxis < 130){forward();}
      if (yAxis > 150){backward();}
      if (yAxis < 130){
        motorSpeedA = map(yAxis, 130, 60, 0, 255);
        motorSpeedB = map(yAxis, 130, 60, 0, 255); 
      }
      if (yAxis > 150){
        motorSpeedA = map(yAxis, 150, 220, 0, 255);
        motorSpeedB = map(yAxis, 150, 220, 0, 255);
      }
    }
    else{
      if (yAxis < 130){forward();}
      if (yAxis > 150){backward();}
      if (xAxis < 130){
        motorSpeedA = map(xAxis, 130, 60, 255, 50);
        motorSpeedB = 255; 
      }
      if (xAxis > 150){
        motorSpeedA = 255;
        motorSpeedB = map(xAxis, 150, 220, 255, 50); 
      }
    }
  }
  analogWrite(AP, motorSpeedA);
  analogWrite(BP, motorSpeedB);
}

void forward(){
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW); 
  digitalWrite(B1, HIGH);
  digitalWrite(B2, LOW);
}

void backward(){
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH); 
  digitalWrite(B1, LOW);
  digitalWrite(B2, HIGH);
}

void turnRight(){
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW); 
  digitalWrite(B1, LOW);
  digitalWrite(B2, HIGH);
}

void turnLeft(){
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH); 
  digitalWrite(B1, HIGH);
  digitalWrite(B2, LOW);
}

void Stop(){
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW); 
  digitalWrite(B1, LOW);
  digitalWrite(B2, LOW);
}

double computePID(double inp){
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);

  error = setPoint - inp;
  if(-10 < error and error < 10){
    pid_i += ki*error;
  }
  rateError = (error - lastError)/elapsedTime;
  double out = kp*error + pid_i + kd*rateError;
  
  lastError = error;
  previousTime = currentTime;

  return out;
}

void motor(int left, int right){
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);
  analogWrite(AP, left);
  digitalWrite(B1, HIGH);
  digitalWrite(B2, LOW);
  analogWrite(BP, right);
}
