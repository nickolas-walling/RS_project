/* NOTE:
   Hold paper with hands while robot is driving, otherwise the paper may slip
*/

#define SAMPLE0 5
#define READINGS 16 //set this number to twice the number of lines you want to read (including gaps)
#define buttonAPin 14


#include "encoders.h"
#include "linesensor.h"
#include "motors.h"
#include "pid.h"
#include "kinematics.h"
#include <math.h>

float GO = 6; //this sets the wheel speed in radians per second

float avg_spd_L;
float avg_spd_R;
float x_prev = 0;
float LINE_WIDTH = 8;

float CHECK = (LINE_WIDTH / 2)*0.9; //we are pinging the line based on distance. Because it's highly unlikely to go exactly WIDTH/2 every time, we want a slightly faster sampling rate. If we're "close enough" go ahead and ping. 

PID_c left;
PID_c right;
LineSensor_c sensors;
Motors_c motors;
Kinematics_c kine;

unsigned long start_ts0;
float avg_elapsed = 0;
float K_p_left = 1;
float K_i_left = 0.05;
float K_d_left = 100;
int j = 0;

bool data[READINGS];
float x_diff_store[READINGS] = {0};

bool flag = false;
bool flag2 = false;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  //while (!Serial) {}
  Serial.println("***RESET***");
  pinMode(buttonAPin, INPUT);
  int buttonStateA = 0;

  sensors.initialize();
  /* CALIBRATION

     Place on white surface until yellow LED turns off
     Once the LED turns on again, place on black surface
     Once LED turns off again calibration is finished
     Set robot in start postition
  */

  //once calibration is done, press A button and robot will go after 1.5 seconds
  while (!flag) {
    buttonStateA = digitalRead(buttonAPin);
    if (buttonStateA == LOW) {
      // turn LED on:
      flag = true;
      delay(1500);
    }
  }
  motors.initialize();

  avg_spd_L = 0.0;
  avg_spd_R = 0.0;

  setupEncoder0();
  setupEncoder1();

  left.initialize(K_p_left, K_i_left, K_d_left);
  right.initialize(K_p_left, K_i_left, K_d_left);

  start_ts0 = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long current_ts0 = millis();
  unsigned long elapsed0;
  float x_diff;

  float feedback_L;
  float feedback_R;

  elapsed0 = current_ts0 - start_ts0;

  if (elapsed0 > SAMPLE0) {
    kine.update();
    kine.velocity(elapsed0);

    avg_spd_L = (avg_spd_L * 0.7) + (kine.velocity_L_rad * 0.3);
    avg_spd_R = (avg_spd_R * 0.7) + (kine.velocity_R_rad * 0.3);

    //unless we've found a line, we're always searching for a line
    if (!flag2) {
      sensors.read_linesensors();
    }

    //keeps track of distance since last measurement
    x_diff = kine.X - x_prev;

    //if we've gone the requisite distance AND we've already found a line, read the line
    if (x_diff >= CHECK && flag2) {
      sensors.read_linesensors();
      data[j] = sensors.on_line();
      x_diff_store[j] = x_diff;
      j++;
      x_prev = kine.X;
    }

    //if this is the first time we're finding a line, record the distance and switch flag2 to true
    if (sensors.on_line() && !flag2) {
      data[j] = true;
      flag2 = true;
      j++;
      x_prev = kine.X;
    }

    feedback_L = left.update(GO, avg_spd_L);
    feedback_R = right.update(GO, avg_spd_R);

    motors.left(feedback_L);
    motors.right(feedback_R);

    avg_elapsed = avg_elapsed * 0.5 + (float)elapsed0 * 0.5;

    start_ts0 = millis();
  }

  if (j == READINGS) {
    while (1) {
      motors.halt();
      Serial.println("Data:");
      for (int k = 0; k < READINGS; k++) {
        Serial.println(data[k]);
      }
      Serial.println("*");
      Serial.println("Distance since last reading:");
      for (int k = 0; k < READINGS; k++) {
        Serial.println(x_diff_store[k]);
      }
      Serial.println("**");
      Serial.print("Average velocity left/right: ");
      Serial.print(avg_spd_L);
      Serial.print(" , ");
      Serial.println(avg_spd_R);
      Serial.println("***");
      Serial.print("Average sample time (ms): ");
      Serial.println(avg_elapsed);
      
      Serial.println("******");
    }
  }
}


/* THOUGHTS:
    use left and right sensors separately for measurements
    can determine directionality that way
    give more data for averages
    since we have kinematics, we could re-phrase everything for distance instead of time?
    Need to re-set line time values--> play with sensitivity value
*/
