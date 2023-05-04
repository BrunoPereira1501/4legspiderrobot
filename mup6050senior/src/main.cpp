#include <Arduino.h>
#include "RP2040_PWM.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>


Adafruit_MPU6050 mpu;
TwoWire CustomI2C0(26, 27); //important

//creates pwm instance
RP2040_PWM* PWM_Instance0;
RP2040_PWM* PWM_Instance1;
RP2040_PWM* PWM_Instance2;
RP2040_PWM* PWM_Instance3;
RP2040_PWM* PWM_Instance4;
RP2040_PWM* PWM_Instance5;
RP2040_PWM* PWM_Instance6;
RP2040_PWM* PWM_Instance7;

#define pinToUse0      0
#define pinToUse1      1
#define pinToUse2      2
#define pinToUse3      3
#define pinToUse4      4
#define pinToUse5      5
#define pinToUse6      6
#define pinToUse7      7

#define DT 0.01 //sampling time
#define ALPHA 0.9 //complementary filter coefficient

float prevAngle = 0;
float anglex = 0, angley = 0, x = 0, y = 0;
float now_mpu, duration, dtime, previous;
float offset4, offset5, offset6, offset7;

float frequency = 50;
float currPWM0, 
      currPWM1,
      currPWM2,
      currPWM3,
      currPWM4,     
      currPWM5,
      currPWM6,
      currPWM7;


float setPWM(int servoNum, float angle){
  float pwmRet = 0;
  float cal[8][2] = {{7.9, 3.2},
                  {7.8, 12.5},
                  {7.25, 2.5},
                  {7.85, 12.5},
                  {12.1, 2.5},
                  {11.6, 2.5},
                  {2.1, 12},
                  {2.6, 12.6}};
  if(servoNum <= 3){
    if(servoNum == 0 || servoNum == 2)  
      pwmRet =  cal[servoNum][0] - abs((angle/90) * (cal[servoNum][0] - cal[servoNum][1]));
    else if(servoNum == 1 || servoNum == 3)  
      pwmRet =  cal[servoNum][0] + abs((angle/90) * (cal[servoNum][0] - cal[servoNum][1]));
  }
  else if(servoNum >= 4){
    if(servoNum == 4 || servoNum == 5)  
      pwmRet =  cal[servoNum][0] - abs((angle/180) * (cal[servoNum][0] - cal[servoNum][1]));
    else if(servoNum == 6 || servoNum == 7)  
      pwmRet =  cal[servoNum][0] + abs((angle/180) * (cal[servoNum][0] - cal[servoNum][1]));
  }
  return pwmRet;
}


void stand(){
  currPWM4 = setPWM(4, (0 + offset4));
  PWM_Instance4->setPWM(pinToUse4, frequency, currPWM4);
  
  currPWM5 = setPWM(5, (0 + offset5));
  PWM_Instance5->setPWM(pinToUse5, frequency, currPWM5);

  currPWM6 = setPWM(6, (0 + offset6));
  PWM_Instance6->setPWM(pinToUse6, frequency, currPWM6);

  currPWM7 = setPWM(7, (0 + offset7));
  PWM_Instance7->setPWM(pinToUse7, frequency, currPWM7);

  currPWM0 = setPWM(0, 45);
  PWM_Instance0->setPWM(pinToUse0, frequency, currPWM0);

  currPWM1 = setPWM(1, 45);
  PWM_Instance1->setPWM(pinToUse1, frequency, currPWM1);

  currPWM2 = setPWM(2, 45);
  PWM_Instance2->setPWM(pinToUse2, frequency, currPWM2);

  currPWM3 = setPWM(3, 45);
  PWM_Instance3->setPWM(pinToUse3, frequency, currPWM3);

}

unsigned long interval, last_cycle;
unsigned long loop_micros;

void setup() {

  //assigns pin 1-7 , with frequency of 50 Hz and a duty cycle of 0%
  PWM_Instance0 = new RP2040_PWM(pinToUse0, 50, 0);
  PWM_Instance1 = new RP2040_PWM(pinToUse1, 50, 0);
  PWM_Instance2 = new RP2040_PWM(pinToUse2, 50, 0);
  PWM_Instance3 = new RP2040_PWM(pinToUse3, 50, 0);
  PWM_Instance4 = new RP2040_PWM(pinToUse4, 50, 0);
  PWM_Instance5 = new RP2040_PWM(pinToUse5, 50, 0);
  PWM_Instance6 = new RP2040_PWM(pinToUse6, 50, 0);
  PWM_Instance7 = new RP2040_PWM(pinToUse7, 50, 0);

  // begin mpu in pins 26 and 27
  mpu.begin(0x68, &CustomI2C0 ,0);

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  interval = 10;

}

void loop() {
  
  unsigned long now = millis();
  if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;

      // Get the duration of each loop
      now_mpu = millis();
      duration = now - previous;
      previous = now;
      dtime = duration / 1000.0;

      /* Get new sensor events with the readings */
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      x = atan(a.acceleration.x/sqrt(pow(a.acceleration.y, 2) + pow (a.acceleration.z, 2))) * RAD_TO_DEG + 3.2; 
      y = atan(a.acceleration.y/sqrt(pow(a.acceleration.x, 2) + pow (a.acceleration.z, 2))) * RAD_TO_DEG - 1.2;

      anglex = ALPHA * (anglex + a.gyro.x * dtime *RAD_TO_DEG) + (1 - ALPHA) * x;
      angley = ALPHA * (angley + a.gyro.y * dtime *RAD_TO_DEG) + (1 - ALPHA) * y;

      if(anglex > 7.5){
        offset4 = anglex;
        offset7 = anglex;

      }
      else if(anglex < -7.5){
        offset5 = -anglex;
        offset6 = -anglex;
      }
      else if(angley > 7.5){
        offset4 = angley;
        offset5 = angley;
      }
      else if(angley < -7.5){
        offset6 = -angley;
        offset7 = -angley;
      }
      else{
        offset4 = 0;
        offset5 = 0;
        offset6 = 0;
        offset7 = 0;
      }

      stand();

      /* Print out the values */
      /*
      Serial.print("Acceleration X: ");
      Serial.print(a.acceleration.x);
      Serial.print(" ");
      Serial.print(", Y: ");
      Serial.print(a.acceleration.y);
      Serial.print(" ");
      Serial.print(", Z: ");
      Serial.println(a.acceleration.z);
      Serial.print(" ");
      Serial.println(" m/s^2");

      Serial.print("Rotation X: ");
      Serial.print(g.gyro.x);
      Serial.print(", Y: ");
      Serial.print(g.gyro.y);
      Serial.print(", Z: ");
      Serial.print(g.gyro.z);
      Serial.println(" rad/s");

      Serial.print("Temperature: ");
      Serial.print(temp.temperature);
      Serial.println(" degC");

      Serial.print("anglex:");
      Serial.println(anglex);

      Serial.print("angley:");
      Serial.println(angley);
      */
      Serial.print("x:");
      Serial.print(x);

      Serial.print(" y:");
      Serial.print(y);

      Serial.print(" anglex:");
      Serial.print(anglex);

      Serial.print(" angley:");
      Serial.print(angley);

      Serial.print(" o4:");
      Serial.print(offset4);

      Serial.print(" o5:");
      Serial.print(offset5);

      Serial.print(" o6:");
      Serial.print(offset6);

      Serial.print(" o7:");
      Serial.println(offset7);
  }
}