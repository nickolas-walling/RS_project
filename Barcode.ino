/* THOUGHTS:
    use left and right sensors separately for measurements
    can determine directionality that way
    give more data for averages
    since we have kinematics, we could re-phrase everything for distance instead of time
*/

//#define GO 6
#define SAMPLE0 5
#define READINGS 18

#include "encoders.h"
#include "linesensor.h"
#include "motors.h"
#include "pid.h"
#include "kinematics.h"
#include <math.h>

double STR_TIME = 150000; //this is the time to get across 10 mm
//double STR_TIME = 15000; //this would be the time to get across 1 mm
double inc_angle;
double c;
int code[READINGS];
int count = 0;
bool start_tag = false;

float GO = 6;

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
unsigned long start_ts1;

float K_p_left = 1; //1
float K_i_left = 0.05; //0.1
float K_d_left = 100; //0.01

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

  start_ts0 = millis();
  start_ts1 = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long current_ts0 = millis();
  unsigned long current_ts1 = micros();
  unsigned long elapsed0;
  unsigned long elapsed1;

  float feedback_L;
  float feedback_R;

  elapsed0 = current_ts0 - start_ts0;
  elapsed1 = current_ts1 - start_ts1;

  if (elapsed0 > SAMPLE0) {
    //kine.update();
    kine.velocity(elapsed0);
    avg_spd_L = (avg_spd_L * 0.7) + (kine.velocity_L_rad * 0.3);
    avg_spd_R = (avg_spd_R * 0.7) + (kine.velocity_R_rad * 0.3);
    sensors.read_linesensors();

    feedback_L = left.update(GO, avg_spd_L);
    feedback_R = right.update(GO, avg_spd_R);

    motors.left(feedback_L);
    motors.right(feedback_R);

    //motors.left(25);
    //motors.right(25);

    angle_check();
    start_ts0 = millis();

    /*
        if (sensors.on_line()){
          start_tag = true;
        }

        start_ts0 = millis();
      }


      if ((double)elapsed1 > STR_TIME/2 && start_tag == true){
        sensors.read_linesensors();
        if (sensors.on_line()){
          code[count] = 1;
        }
        else{
          code[count] = 0;
        }
        count++;
        start_ts1 = micros();
      }

      if (count > READINGS){
        motors.halt();
        while(1){
          for (int i = 0; i < READINGS; i++){
            Serial.println(code[i]);
          }
          Serial.println("******");
        }*/
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

double angle_check() {
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
    c = (((double)val[3] + (double)val[5]) - ((double)val[2] + (double)val[4])) / 2;
    inc_angle = acos(STR_TIME / c);
    while (1) {
      for (int j = 0; j < 6; j++) {
        Serial.println(val[j]);
      }
      Serial.println("**");
      Serial.println(inc_angle * 100);
      Serial.println("average speed");
      Serial.println(avg_spd_L * 100);
      Serial.println("******");
    }
  }
  return inc_angle;
}

void code_read(int count) {
  float start_read_ts = micros();
  code[count] = sensors.on_line();
}
