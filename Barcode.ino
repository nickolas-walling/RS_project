#define GO 4
#define SAMPLE 5

#include "encoders.h"
#include "linesensor.h"
#include "motors.h"
#include "pid.h"
#include "kinematics.h"
#include <math.h>

double STR_TIME = 138222;
double inc_angle;
float vels[3];

float avg_spd_L;
float avg_spd_R;

PID_c heading;
PID_c follow;
PID_c left;
PID_c right;
LineSensor_c sensors;
Motors_c motors;
Kinematics_c kine;

unsigned long start_ts;

float K_p_left = 5; //1
float K_i_left = 0.1; //0.1
float K_d_left = 0; //0.01

unsigned long start = 0;

int state = 0;

int i = 1;

unsigned long val[6];


void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  //while (!Serial) {}
  Serial.println("***RESET***");

  sensors.initialize();
  delay(3000);
  motors.initialize();

  avg_spd_L = 0.0;
  avg_spd_R = 0.0;


  setupEncoder0();
  setupEncoder1();

  left.initialize(K_p_left, K_i_left, K_d_left);
  right.initialize(K_p_left, K_i_left, K_d_left);

  start_ts = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long current_ts = millis();
  unsigned long elapsed;

  float feedback_L;
  float feedback_R;

  elapsed = current_ts - start_ts;

  if (elapsed > SAMPLE) {
    kine.update();
    kine.velocity(elapsed);
    avg_spd_L = (avg_spd_L * 0.7) + (kine.velocity_L_rad * 0.3);
    avg_spd_R = (avg_spd_R * 0.7) + (kine.velocity_R_rad * 0.3);
    sensors.read_linesensors();

    feedback_L = left.update(GO, avg_spd_L);
    feedback_R = right.update(GO, avg_spd_R);

    motors.left(feedback_L);
    motors.right(feedback_R);

    //motors.left(25);
    //motors.right(25);

    if (sensors.on_line() && state == 0) {
      start = micros();
      state = 1;
    }
    else if (!sensors.on_line() && state == 1) {
      val[i] = micros() - start;
      state = 2;
      i++;
    }
    else if (sensors.on_line() && state == 2) {
      val[i] = micros() - start;
      state = 1;
      i++;
    }

    if (i == 6) {
      motors.halt();
      while (1) {
        for (int j = 0; j < 6; j++) {
          Serial.println(val[j]);
        }
        Serial.println("******");
      }
    }
    start_ts = millis();

  }

}

double average(unsigned long num[3]) {
  double temp = 0;
  for (int i = 0; i < 3; i++) {
    temp += (double)num[i];
  }
  temp = temp / 3;

  return temp;
}
