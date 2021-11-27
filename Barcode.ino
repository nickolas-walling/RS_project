/* NOTE:
 * Hold paper with hands while robot is driving, otherwise the paper may slip
 */

#define SAMPLE0 5
#define READINGS 16 //set this number to twice the number of lines you want to read
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

PID_c heading;
PID_c follow;
PID_c left;
PID_c right;
LineSensor_c sensors;
Motors_c motors;
Kinematics_c kine;

unsigned long start_ts0;
unsigned long avg_elapsed = 0;
float K_p_left = 1;
float K_i_left = 0.05;
float K_d_left = 100;

unsigned long start = 0;

int state = 0;

int i = 0;

unsigned long val[READINGS] = {0};
float pos[READINGS] = {0};

bool flag = false;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  //while (!Serial) {}
  Serial.println("***RESET***");
  pinMode(buttonAPin, INPUT);
  int buttonStateA = 0;

  sensors.initialize();
  /* CALIBRATION
   *  
   * Place on white surface until yellow LED turns off
   * Once the LED turns on again, place on black surface
   * Once LED turns off again calibration is finished
   * Set robot in start postition
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

  float feedback_L;
  float feedback_R;

  elapsed0 = current_ts0 - start_ts0;

  if (elapsed0 > SAMPLE0) {
    kine.update();
    kine.velocity(elapsed0);
    avg_spd_L = (avg_spd_L * 0.7) + (kine.velocity_L_rad * 0.3);
    avg_spd_R = (avg_spd_R * 0.7) + (kine.velocity_R_rad * 0.3);
    sensors.read_linesensors();

    feedback_L = left.update(GO, avg_spd_L);
    feedback_R = right.update(GO, avg_spd_R);

    motors.left(feedback_L);
    motors.right(feedback_R);

    line_check();
    avg_elapsed = avg_elapsed * 0.5 + elapsed0 * 0.5;
    start_ts0 = millis();
  }
}

void line_check() {

  //This is where the line measurements start
  //If you want it to just stop the first time it reads a line, add "motors.halt(); while (1){} to this section
  if (sensors.on_line() && state == 0) {
    start = micros();
    val[i] = 0;
    pos[i] = kine.X;
    state = 1;
    i++;
  }


  //reads when it goes off of a line
  else if (!sensors.on_line() && state == 1) {
    val[i] = micros() - start;
    pos[i] = kine.X;//-offset;
    state = 2;
    i++;
  }

  //reads when it goes onto a line (only after the initial line read)
  else if (sensors.on_line() && state == 2) {
    val[i] = micros() - start;
    pos[i] = kine.X;//-offset;
    state = 1;
    i++;
  }


  //stops the robot if it has read however many readings we set it to OR if it has traveled 240 mm
  if (kine.X >= 240 || i == READINGS) {

    motors.halt(); //stops motors

    //traps in an infinite loop, plug in cable to get data
    while (1) {

      //prints out data points in (time, position) format
      for (int j = 0; j < READINGS; j++) {
        Serial.print(val[j]);
        Serial.print(",");
        Serial.print(pos[j]);
        Serial.print("\n");
      }
      //prints out the average speed
      Serial.println("**");
      Serial.println("average speed");
      Serial.println(avg_spd_L);
      Serial.println(avg_spd_R);
      Serial.println("Number of readings:");
      Serial.println(i); //prints out how many readings
      Serial.println("Average sampling time");
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
