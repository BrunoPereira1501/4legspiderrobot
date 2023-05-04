#include <Arduino.h>
#include "RP2040_PWM.h"
#include <Wire.h>
#include <HCSR04.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// Creates pwm instances
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


// Pins for sonar
#define trigPin 16
#define echoPin 17
UltraSonicDistanceSensor distanceSensor(trigPin, echoPin);  // Initialize sensor that uses digital pins 13 and 12.

// Mpu initializations
Adafruit_MPU6050 mpu;
TwoWire CustomI2C0(26, 27); //customized I2C pins

#define ALPHA 0.9 //complementary filter coefficient for mpu

float prevAngle = 0;
float anglex = 0, angley = 0, x = 0, y = 0;
float now_mpu, duration, dtime, previous;
float offset4, offset5, offset6, offset7;


#define time 200 // time between walking states
#define time_tr 200 // time between turning states
#define countc 8 // amount of times it performs a turning cycle
#define countd 14 // amount of cycles it performs after object detection 
#define dist_stop 18 // min distance before object detection

bool stop; // variable to stop
int count1, count2, set1, set2, direita, esquerda, flagCount1, flagCount2, countEsq, dec, flagdec; // variables necessary for M3 (turning)
//long duration;
float distanceCm, distance = 0;
float frequency = 50;
int turn_count = 0;
float currPWM0, 
      currPWM1,
      currPWM2,
      currPWM3,
      currPWM4,     
      currPWM5,
      currPWM6,
      currPWM7;

// initialize funtions
float setPWM(int servoNum, float angle);
void stand();
void lay_down();
void first_position();
void move1();
void move2();

typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

// Our finite state machines
fsm_t fsm1, fsm2, fsm3, fsm4;

unsigned long interval, last_cycle;
unsigned long loop_micros;

// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}


void setup() 
{
 
  //assigns pin 1-7 , with frequency of 50 Hz and a duty cycle of 0%
  PWM_Instance0 = new RP2040_PWM(pinToUse0, 50, 0);
  PWM_Instance1 = new RP2040_PWM(pinToUse1, 50, 0);
  PWM_Instance2 = new RP2040_PWM(pinToUse2, 50, 0);
  PWM_Instance3 = new RP2040_PWM(pinToUse3, 50, 0);
  PWM_Instance4 = new RP2040_PWM(pinToUse4, 50, 0);
  PWM_Instance5 = new RP2040_PWM(pinToUse5, 50, 0);
  PWM_Instance6 = new RP2040_PWM(pinToUse6, 50, 0);
  PWM_Instance7 = new RP2040_PWM(pinToUse7, 50, 0);

  
  Serial.begin(115200); // Starts the serial communication
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
 

  interval = 10;
  set_state(fsm1, 0);
  set_state(fsm2, 0);
  set_state(fsm3, 0); 
  set_state(fsm4, 0); 

  // begin mpu in pins 26 and 27
  mpu.begin(0x68, &CustomI2C0 ,0);

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);  

  count1 = countc ;
  count2 = 5;
  set1 = 0;
  set2 = 0;
  direita = 0;
  esquerda = 0;
  stop = false;
  flagCount1 = 0;
  flagCount2 = 0;
  countEsq = 0;
  dec = countd;
  flagdec = 0;
}

void loop() 
{
    // To measure the time between loop() calls
    //unsigned long last_loop_micros = loop_micros; 
    
    // Do this only every "interval" miliseconds 
    // It helps to clear the switches bounce effect
    unsigned long now = millis();
    distance = distanceSensor.measureDistanceCm();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;

      // Get the duration of each loop which should be equivalent to interval (10 ms)
      now_mpu = millis();
      duration = now_mpu - previous;
      previous = now_mpu;
      dtime = duration / 1000.0;

      // Get new sensor events with the readings 
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      // Get coordinates through formula
      x = atan(a.acceleration.x/sqrt(pow(a.acceleration.x, 2) + pow (a.acceleration.z, 2))) * RAD_TO_DEG + 3.2; 
      y = atan(a.acceleration.y/sqrt(pow(a.acceleration.y, 2) + pow (a.acceleration.z, 2))) * RAD_TO_DEG - 1.2;

      // Filter the angles
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

      // State Machines

      // Update tis for all state machines
      unsigned long cur_time = millis();   // Just one call to millis()
      fsm1.tis = cur_time - fsm1.tes;
      fsm2.tis = cur_time - fsm2.tes;
      fsm3.tis = cur_time - fsm3.tes; 
      fsm4.tis = cur_time - fsm4.tes; 

      // Walking
      if (fsm1.state == 0 && fsm1.tis > time){
        fsm1.new_state = 1;
      } else if (fsm1.state == 1 && stop){
        fsm1.new_state = 8;
      } else if (fsm1.state == 1 && fsm1.tis > time){
        fsm1.new_state = 2;
      } else if (fsm1.state == 2 && stop){
        fsm1.new_state = 8;
      } else if (fsm1.state == 2 && fsm1.tis > time){
        fsm1.new_state = 21;
      } else if (fsm1.state == 21 && stop){
        fsm1.new_state = 8;
      } else if (fsm1.state == 21 && fsm1.tis > time){
        fsm1.new_state = 3;
      } else if (fsm1.state == 3 && stop){
        fsm1.new_state = 8;
      } else if (fsm1.state == 3 && fsm1.tis > time){
        fsm1.new_state = 4;
      } else if (fsm1.state == 4 && stop){
        fsm1.new_state = 8;
      } else if (fsm1.state == 4 && fsm1.tis > time){
        fsm1.new_state = 41;
      } else if (fsm1.state == 41 && stop){
        fsm1.new_state = 8;
      } else if (fsm1.state == 41 && fsm1.tis > time){
        fsm1.new_state = 5;
      } else if (fsm1.state == 5 && stop){
        fsm1.new_state = 8;
      } else if (fsm1.state == 5 && fsm1.tis > time){
        fsm1.new_state = 51;
      } else if (fsm1.state == 51 && stop){
        fsm1.new_state = 8;
      } else if (fsm1.state == 51 && fsm1.tis > time){
        fsm1.new_state = 6;
      } else if (fsm1.state == 6 && stop){
        fsm1.new_state = 8;
      } else if (fsm1.state == 6 && fsm1.tis > time){
        fsm1.new_state = 7;
      } else if (fsm1.state == 7 && stop){
        fsm1.new_state = 8;
      } else if (fsm1.state == 7 && fsm1.tis > time){
        fsm1.new_state = 71;
      } else if (fsm1.state == 71 && stop){
        fsm1.new_state = 8;
      } else if (fsm1.state == 71 && fsm1.tis > time){
        fsm1.new_state = 2;
      } else if (fsm1.state == 8 && !stop){
        fsm1.new_state = 1;
      }

      // Sonar
       if (fsm2.state == 0 && distance < dist_stop && distance > 0){
        fsm2.new_state = 1;
      } else if (fsm2.state == 1 && set1 == 1){
        fsm2.new_state = 0;
      } else if (fsm2.state == 0 && dec == 0){
        fsm2.new_state = 2;
      } else if(fsm2.state == 2 && set2 == 1){
        fsm2.new_state = 0;
      }
      
      // Turn right
        if (fsm3.state == 0 && direita == 1){
        fsm3.new_state = 10;
      } else if (fsm3.state == 10 && fsm3.tis > time_tr){
        fsm3.new_state = 1;
      } else if (fsm3.state == 1 && fsm3.tis > time_tr){
        fsm3.new_state = 2;
      } else if (fsm3.state == 2 && fsm3.tis > time_tr){
        fsm3.new_state = 3;
      } else if (fsm3.state == 3 && fsm3.tis > time_tr){
        fsm3.new_state = 4;
      } else if (fsm3.state == 4 && fsm3.tis > time_tr){
        fsm3.new_state = 5;
      } else if (fsm3.state == 5 && fsm3.tis > time_tr){
        fsm3.new_state = 6;
      } else if (fsm3.state == 6 && fsm3.tis > time_tr){
        fsm3.new_state = 7;
      } else if (fsm3.state == 7 && fsm3.tis > time_tr){
        fsm3.new_state = 8;
      } else if (fsm3.state == 8 && fsm3.tis > time_tr){
        fsm3.new_state = 9;
      } else if (fsm3.state == 9 && count1> 0){
        fsm3.new_state = 10;
      } else if (fsm3.state == 9 && count1 == 0){
        fsm3.new_state = 0;
      }

      // Turn left 
        if (fsm4.state == 0 && esquerda == 1){
        fsm4.new_state = 1;
      } else if (fsm4.state == 1 && fsm4.tis > time_tr){
        fsm4.new_state = 2;
      } else if (fsm4.state == 2 && fsm4.tis > time_tr){
        fsm4.new_state = 3;
      } else if (fsm4.state == 3 && fsm4.tis > time_tr){
        fsm4.new_state = 4;
      } else if (fsm4.state == 4 && fsm4.tis > time_tr){
        fsm4.new_state = 5;
      } else if (fsm4.state == 5 && fsm4.tis > time_tr){
        fsm4.new_state = 6;
      } else if (fsm4.state == 6 && fsm4.tis > time_tr){
        fsm4.new_state = 7;
      } else if (fsm4.state == 7 && fsm4.tis > time_tr){
        fsm4.new_state = 8;
      } else if (fsm4.state == 8 && fsm4.tis > time_tr){
        fsm4.new_state = 9;
      } else if (fsm4.state == 9 && fsm4.tis > time_tr){
        fsm4.new_state = 10;
      } else if (fsm4.state == 10 && count2 > 0){
        fsm4.new_state = 1;
      } else if (fsm4.state == 10 && count2 == 0){
        fsm4.new_state = 0;
      }

      // Update the states
      set_state(fsm1, fsm1.new_state);
      set_state(fsm2, fsm2.new_state);
      set_state(fsm3, fsm3.new_state);
      set_state(fsm4, fsm4.new_state);

      // Actions set by walking
      if (fsm1.state == 0){
        stand();
      } else if (fsm1.state == 1){
        first_position();

      } else if (fsm1.state == 2){
        flagdec = 0;
        if(fsm1.tis < time/2){
          currPWM5 = setPWM(5, 45);
          PWM_Instance5->setPWM(pinToUse5, frequency, currPWM5);
        }
        else{
        currPWM1 = setPWM(1, 40);
        PWM_Instance1->setPWM(pinToUse1, frequency, currPWM1);
        }
      
      } else if(fsm1.state == 21){
        currPWM5 = setPWM(5, 0);
        PWM_Instance5->setPWM(pinToUse5, frequency, currPWM5);

      } else if (fsm1.state == 3){
        move1();
      
      } else if (fsm1.state == 4){
        if(fsm1.tis < time/2){
        currPWM7 = setPWM(7, 70);
        PWM_Instance7->setPWM(pinToUse7, frequency, currPWM7);
        }
        else {
        currPWM3 = setPWM(3, 0);
        PWM_Instance3->setPWM(pinToUse3, frequency, currPWM3);
        }
      } else if (fsm1.state == 41){
        currPWM7 = setPWM(7, 0);
        PWM_Instance7->setPWM(pinToUse7, frequency, currPWM7);
     
      } else if (fsm1.state == 5){
        if(fsm1.tis < time/2){
        currPWM6 = setPWM(6, 30);
        PWM_Instance5->setPWM(pinToUse6, frequency, currPWM6);
        }
        else{
        currPWM2 = setPWM(2, 60);
        PWM_Instance2->setPWM(pinToUse2, frequency, currPWM2);
        }
      } else if (fsm1.state == 51){
        currPWM6 = setPWM(6, 0);
        PWM_Instance5->setPWM(pinToUse6, frequency, currPWM6);
      
      }else if (fsm1.state == 6){
        move2();
      
      } else if (fsm1.state == 7){
        if(fsm1.tis < time/2){
        currPWM4 = setPWM(4, 70);
        PWM_Instance4->setPWM(pinToUse4, frequency, currPWM4);
        }
        else{
        currPWM0 = setPWM(0, 0);
        PWM_Instance0->setPWM(pinToUse0, frequency, currPWM0);
        }
        if(countEsq == 1 && flagdec == 0){
          dec--;
          flagdec = 1;
        }
      
      } else if (fsm1.state == 71){
        currPWM4 = setPWM(4, 0);
        PWM_Instance4->setPWM(pinToUse4, frequency, currPWM4);
      }

      //Actions set by Sonar
      if (fsm2.state == 0){
        stop = false;
        direita = 0;
        esquerda = 0;
        count1 = countc;
        count2 = 5;
      } else if (fsm2.state == 1){
        stop = true;
        direita = 1;
        countEsq = 1;
      } else if (fsm2.state == 2){
        stop = true;
        esquerda = 1;
        countEsq = 0;
        dec = countd;
      }

      // Action set by Turning Right
      if (fsm3.state == 0){
        set1 = 0;
      } else if (fsm3.state == 10){
        stand();
        flagCount1 = 0;
      } else if (fsm3.state == 1){
        currPWM5 = setPWM(5, 45);
        PWM_Instance5->setPWM(pinToUse5, frequency, currPWM5);

        currPWM1 = setPWM(1, 10);
        PWM_Instance1->setPWM(pinToUse1, frequency, currPWM1);
      } else if (fsm3.state == 2){
        currPWM5 = setPWM(5, 0);
        PWM_Instance5->setPWM(pinToUse5, frequency, currPWM5);
      } else if (fsm3.state == 3){
          currPWM7 = setPWM(7, 45);
          PWM_Instance7->setPWM(pinToUse7, frequency, currPWM7);
      
          currPWM3 = setPWM(3, 0);
          PWM_Instance3->setPWM(pinToUse3, frequency, currPWM3);
      } else if (fsm3.state == 4){
          currPWM7 = setPWM(7, 0);
          PWM_Instance7->setPWM(pinToUse7, frequency, currPWM7);
      } else if (fsm3.state == 5){
        currPWM6 = setPWM(6, 45);
        PWM_Instance5->setPWM(pinToUse6, frequency, currPWM6);
    
        currPWM2 = setPWM(2, 70);
        PWM_Instance2->setPWM(pinToUse2, frequency, currPWM2);
      } else if (fsm3.state == 6){
        currPWM6 = setPWM(6, 0);
        PWM_Instance5->setPWM(pinToUse6, frequency, currPWM6);
      } else if (fsm3.state == 7){
        currPWM4 = setPWM(4, 45);
        PWM_Instance4->setPWM(pinToUse4, frequency, currPWM4);
        
        currPWM0 = setPWM(0, 90);
        PWM_Instance0->setPWM(pinToUse0, frequency, currPWM0);
      } else if(fsm3.state == 8){
        currPWM4 = setPWM(4, 0);
        PWM_Instance4->setPWM(pinToUse4, frequency, currPWM4);
      } else if (fsm3.state == 9){
        
        stand();
        
        if(flagCount1 == 0){
          count1 --;
          flagCount1 = 1;
        }
        if(count1 == 0)
          set1 = 1;
      }

      // Actions set by turning left 
      if (fsm4.state == 0){
        set2 = 0;
      } else if (fsm4.state == 1){
        stand();
        flagCount2 = 0;
      } else if (fsm4.state == 2){
        currPWM5 = setPWM(5, 45);
        PWM_Instance5->setPWM(pinToUse5, frequency, currPWM5);

        currPWM1 = setPWM(1, 70);
        PWM_Instance1->setPWM(pinToUse1, frequency, currPWM1);
      } else if (fsm4.state == 3){
        currPWM5 = setPWM(5, 0);
        PWM_Instance5->setPWM(pinToUse5, frequency, currPWM5);
      } else if (fsm4.state == 4){
          currPWM7 = setPWM(7, 45);
          PWM_Instance7->setPWM(pinToUse7, frequency, currPWM7);
      
          currPWM3 = setPWM(3, 90);
          PWM_Instance3->setPWM(pinToUse3, frequency, currPWM3);
      } else if (fsm4.state == 5){
          currPWM7 = setPWM(7, 0);
          PWM_Instance7->setPWM(pinToUse7, frequency, currPWM7);
      } else if (fsm4.state == 6){
        currPWM6 = setPWM(6, 45);
        PWM_Instance5->setPWM(pinToUse6, frequency, currPWM6);
    
        currPWM2 = setPWM(2, 0);
        PWM_Instance2->setPWM(pinToUse2, frequency, currPWM2);
      } else if (fsm4.state == 7){
        currPWM6 = setPWM(6, 0);
        PWM_Instance5->setPWM(pinToUse6, frequency, currPWM6);
      } else if (fsm4.state == 8){
        currPWM4 = setPWM(4, 45);
        PWM_Instance4->setPWM(pinToUse4, frequency, currPWM4);
        
        currPWM0 = setPWM(0, 0);
        PWM_Instance0->setPWM(pinToUse0, frequency, currPWM0);
      } else if(fsm4.state == 9){
        currPWM4 = setPWM(4, 0);
        PWM_Instance4->setPWM(pinToUse4, frequency, currPWM4);
      } else if (fsm4.state == 10){
        stand();
        
        if(flagCount2 == 0){
          count2 --;
          flagCount2 = 1;
        }

        if(count2 == 0)
          set2 = 1;
      }

      
      // Debug using the serial port

      Serial.print(" fsm1.state: ");
      Serial.print(fsm1.state);

      Serial.print(" fsm2.state: ");
      Serial.print(fsm2.state);

      Serial.print(" fsm3.state: ");
      Serial.print(fsm3.state);

      Serial.print(" fsm4.state: ");
      Serial.print(fsm4.state);

      Serial.print(" count1: ");
      Serial.print(count1);

      Serial.print(" count2: ");
      Serial.print(count2);

      Serial.print(" dec: ");
      Serial.print(dec);  

      Serial.print(" esquerda: ");
      Serial.print(esquerda);      

      Serial.print(" countEsq: ");
      Serial.print(countEsq); 

      Serial.print(" loop: ");
      Serial.println(micros() - loop_micros);

      Serial.print(" distance: ");
      Serial.print(distance);
    }

    
}
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
      pwmRet =  cal[servoNum][0] - abs((angle/180) * (cal[servoNum][0]-cal[servoNum][1]));
    else if(servoNum == 6 || servoNum == 7)  
      pwmRet =  cal[servoNum][0] + abs((angle/180) * (cal[servoNum][0]-cal[servoNum][1]));
  }
  return pwmRet;
}


void stand(){
  currPWM4 = setPWM(4, 0);
  PWM_Instance4->setPWM(pinToUse4, frequency, currPWM4);
  
  currPWM5 = setPWM(5, 0);
  PWM_Instance5->setPWM(pinToUse5, frequency, currPWM5);

  currPWM6 = setPWM(6, 0);
  PWM_Instance6->setPWM(pinToUse6, frequency, currPWM6);

  currPWM7 = setPWM(7, 0);
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

void lay_down()
{
  currPWM0 = setPWM(0, 45);
  PWM_Instance0->setPWM(pinToUse0, frequency, currPWM0);

  currPWM1 = setPWM(1, 45);
  PWM_Instance1->setPWM(pinToUse1, frequency, currPWM1);

  currPWM2 = setPWM(2, 45);
  PWM_Instance2->setPWM(pinToUse2, frequency, currPWM2);

  currPWM3 = setPWM(3, 45);
  PWM_Instance3->setPWM(pinToUse3, frequency, currPWM3);

  currPWM4 = setPWM(4, 90);
  PWM_Instance4->setPWM(pinToUse4, frequency, currPWM4);

  currPWM5 = setPWM(5, 90);
  PWM_Instance5->setPWM(pinToUse5, frequency, currPWM5);

  currPWM6 = setPWM(6, 90);
  PWM_Instance6->setPWM(pinToUse6, frequency, currPWM6);

  currPWM7 = setPWM(7, 90);
  PWM_Instance7->setPWM(pinToUse7, frequency, currPWM7);
}

void first_position()
{
  currPWM0 = setPWM(0, 0);
  PWM_Instance0->setPWM(pinToUse0, frequency, currPWM0);
  
  currPWM1 = setPWM(1, 0);
  PWM_Instance1->setPWM(pinToUse1, frequency, currPWM1);

  currPWM2 = setPWM(2, 45);
  PWM_Instance2->setPWM(pinToUse2, frequency, currPWM2);

  currPWM3 = setPWM(3, 45);
  PWM_Instance3->setPWM(pinToUse3, frequency, currPWM3);

  currPWM4 = setPWM(4, 0);
  PWM_Instance4->setPWM(pinToUse4, frequency, currPWM4);
  
  currPWM5 = setPWM(5, 0);
  PWM_Instance5->setPWM(pinToUse5, frequency, currPWM5);

  currPWM6 = setPWM(6, 0);
  PWM_Instance6->setPWM(pinToUse6, frequency, currPWM6);

  currPWM7 = setPWM(7, 0);
  PWM_Instance7->setPWM(pinToUse7, frequency, currPWM7);
}



void move1()
{
  currPWM1 = setPWM(1, 30);
  PWM_Instance1->setPWM(pinToUse1, frequency, currPWM1);
  currPWM2 = setPWM(2, 0);
  PWM_Instance2->setPWM(pinToUse2, frequency, currPWM2);
  currPWM0 = setPWM(0, 45);
  PWM_Instance0->setPWM(pinToUse0, frequency, currPWM0);
  currPWM3 = setPWM(3, 60);
  PWM_Instance3->setPWM(pinToUse3, frequency, currPWM3);
  
  currPWM4 = setPWM(4, 0);
  PWM_Instance4->setPWM(pinToUse4, frequency, currPWM4);
  
  currPWM5 = setPWM(5, 0);
  PWM_Instance5->setPWM(pinToUse5, frequency, currPWM5);

  currPWM6 = setPWM(6, 0);
  PWM_Instance6->setPWM(pinToUse6, frequency, currPWM6);

  currPWM7 = setPWM(7, 0);
  PWM_Instance7->setPWM(pinToUse7, frequency, currPWM7);

}



void move2()
{
  currPWM2 = setPWM(2, 30);
  PWM_Instance2->setPWM(pinToUse2, frequency, currPWM2);
  currPWM0 = setPWM(0, 60);
  PWM_Instance0->setPWM(pinToUse0, frequency, currPWM0);
  currPWM3 = setPWM(3, 45);
  PWM_Instance3->setPWM(pinToUse3, frequency, currPWM3);
  currPWM1 = setPWM(1, 0);
  PWM_Instance1->setPWM(pinToUse1, frequency, currPWM1);

  currPWM4 = setPWM(4, 0);
  PWM_Instance4->setPWM(pinToUse4, frequency, currPWM4);
  
  currPWM5 = setPWM(5, 0);
  PWM_Instance5->setPWM(pinToUse5, frequency, currPWM5);

  currPWM6 = setPWM(6, 0);
  PWM_Instance6->setPWM(pinToUse6, frequency, currPWM6);

  currPWM7 = setPWM(7, 0);
  PWM_Instance7->setPWM(pinToUse7, frequency, currPWM7);
 
}