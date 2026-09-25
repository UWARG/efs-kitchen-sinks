#define GIMBAL_Y_PIN 5
#define GIMBAL_X_PIN 6
#include <Servo.h>


Servo gimbalY;
Servo gimbalX;

void setup() {
  // put your setup code here, to run once:
  gimbalY.attach(GIMBAL_Y_PIN);
  gimbalX.attach(GIMBAL_X_PIN);
  Serial.begin(9600); //9600 Baud Rate
  Serial.setTimeout(15);

}

const int defaultInterval = 50, interval = 4;
int rollingX = defaultInterval, rollingY = defaultInterval;

void loop() {
  // put your main code here, to run repeatedly:
  const int W = 1;
  const int S = 2;
  const int A = 3;
  const int D = 4;

  if(true)
  {
    int code = Serial.parseInt();

    switch(code)
    {
      case W:
        rollingY += interval;
        break;
      case S:
        rollingY -= interval;
        break;
      case A:
        rollingX += interval;
        break;
      case D:
        rollingX -= interval;
        break;
      default:
        break;       
    }

    //Serial.println(code);
    //Serial.print(" ");
    
  }
  setGimbalY(rollingY);
  setGimbalX(rollingX);
  //Serial.flush();  

}

const int SERVO_MIN_MS = 400;
const int SERVO_MAX_MS = 2500;

void setGimbalY(int &percent)
{
  if(percent < 0) {percent = 0;}
  if(percent > 100) {percent = 100;}
  int microseconds = ((SERVO_MAX_MS-SERVO_MIN_MS)*(percent/100.0) + SERVO_MIN_MS);
  
  Serial.print(" Gimbal Y ");
  Serial.print(percent);
  Serial.print(" ");
  Serial.println(microseconds);
  

  gimbalY.writeMicroseconds(microseconds);
}

void setGimbalX(int &percent)
{
  if(percent < 0) {percent = 0;}
  if(percent > 100) {percent = 100;}
  int microseconds = ((SERVO_MAX_MS-SERVO_MIN_MS)*(percent/100.0) + SERVO_MIN_MS);
  
  Serial.print(" Gimbal X ");
  Serial.print(percent);
  Serial.print(" ");
  Serial.println(microseconds);
  
  
  gimbalX.writeMicroseconds(microseconds);
}
