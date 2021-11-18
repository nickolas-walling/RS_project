#define GO 6
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

PID_c heading;
PID_c follow;
PID_c left;
PID_c right;
LineSensor_c sensors;
Motors_c motors;
Kinematics_c kine;

unsigned long start_ts;

float K_p_left = 5; //1
float K_i_left = 0; //0.1
float K_d_left = 0; //0.01

unsigned long temp1 = 0;
unsigned long temp2 = 0;

int i = 0;

unsigned long val[3] = {0, 0, 0};
unsigned long blank[3] = {0, 0, 0};

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  //while (!Serial) {}
  Serial.println("***RESET***");

  sensors.initialize();
  delay(3000);
  motors.initialize();

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
    sensors.read_linesensors();

    feedback_L = left.update(GO, kine.velocity_L_rad);
    feedback_R = right.update(GO, kine.velocity_R_rad);

    motors.left(feedback_L);
    motors.right(feedback_R);

    //motors.left(25);
    //motors.right(25);

    if (sensors.on_line() && temp1 == 0) {
      temp1 = micros();
      blank[i] = temp1 - temp2;
      temp2 = 0;
    }
    else if (!sensors.on_line() && temp1 > 0) {
      temp2 = micros();
      val[i] = temp2 - temp1;
      temp1 = 0;
      vels[i] = kine.velocity_L_rad;
      i++;
    }

    if (i==3) {
      motors.halt();
      Serial.begin(9600);
      while (!Serial) {}
        Serial.println(val[0]);
        Serial.println(val[1]);
        Serial.println(val[2]);
        Serial.println("*");
        Serial.println(vels[0]);
        Serial.println(vels[1]);
        Serial.println(vels[2]);
        Serial.println("**");
        Serial.println(blank[0]);
        Serial.println(blank[1]);
        Serial.println(blank[2]);
        Serial.println("***");
        Serial.println(average(val));
        Serial.println("****");
        inc_angle = acos(STR_TIME/average(val));
        Serial.println(inc_angle);
        Serial.println("******");
        
      //}
    }
    start_ts = millis();

  }

}

double average(unsigned long num[3]){
  double temp = 0;
  for (int i = 0; i < 3; i++){
    temp += (double)num[i];
  }
  temp = temp/3;

  return temp;
}
