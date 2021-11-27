// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#define LS_LEFT_IN_PIN 18
#define LS_CNTR_IN_PIN 20
#define LS_RIGHT_IN_PIN 21
#define EMIT 11
#define NB_LS_PINS 3
#define timeout 5000
#define n 24



// Class to operate the linesensor(s).
class LineSensor_c {
  public:

    // Constructor, must exist.
    LineSensor_c() {
    }


    int ls_pin[NB_LS_PINS] = {LS_LEFT_IN_PIN, LS_CNTR_IN_PIN, LS_RIGHT_IN_PIN };
    int which;
    int data[NB_LS_PINS];

    int L_offset = 0;
    int C_offset = 0;
    int R_offset = 0;

    float S_L = 0;
    float S_C = 0;
    float S_R = 0;

    float cond_L;
    float cond_C;
    float cond_R;

    void initialize() {
      pinMode(LS_LEFT_IN_PIN, INPUT);
      pinMode(LS_CNTR_IN_PIN, INPUT);
      pinMode(LS_RIGHT_IN_PIN, INPUT);
      pinMode(LED_BUILTIN, OUTPUT);
      pinMode(EMIT, OUTPUT);

      digitalWrite(EMIT, HIGH);

      calibrate();
    }

    void read_linesensors() {

      unsigned long start_time; // t_1
      unsigned long t_test;
      unsigned long elapsed_time;
      int i;
      bool done = false;
      unsigned long time_duration[NB_LS_PINS] = {0, 0, 0};

      // Charge capacitor by setting input pin
      // temporarily to output and HIGH

      for (i = 0; i < NB_LS_PINS; i++) {
        pinMode( ls_pin[i], OUTPUT );
        digitalWrite( ls_pin[i], HIGH );
      }

      delayMicroseconds(10);

      for (i = 0; i < NB_LS_PINS; i++) {
        pinMode(ls_pin[i], INPUT);
      }

      // Store current microsecond count
      start_time = micros();

      //Get readings from sensors
      while (done == false) {
        for (which  = 0; which < NB_LS_PINS; which++) {
          if (digitalRead(ls_pin[which]) == LOW && time_duration[which] == 0) {
            t_test = micros();
            time_duration[which] = t_test - start_time;
          }
        }
        //Only done if all sensors have read "LOW"
        if (time_duration[0] != 0 && time_duration[1] != 0 && time_duration[2] != 0) {
          done = true;
        }
        t_test = micros();

        //timeout if it takes too long
        if (t_test - start_time > timeout) {
          Serial.println("TIMEOUT");

        }

      }

      /*

        Serial.println("Line Sensors, left to right: " );
        Serial.println( time_duration[0]);
        Serial.println(time_duration[1]);
        Serial.println(time_duration[2]);
        //return time_duration;

      */

      data[0] = (int)time_duration[0];
      data[1] = (int)time_duration[1];
      data[2] = (int)time_duration[2];

      cond_L = (data[0] - L_offset) * S_L;
      cond_C = (data[1] - C_offset) * S_C;
      cond_R = (data[2] - R_offset) * S_R;


    }

    float line_error() {
      read_linesensors();

      int i = 0;
      float store = 0;

      float w_left;
      float w_right;
      float e_line;

      while (i < 3) {
        store += data[i];
        i += 1;
      }

      w_left = (cond_L + 0.5 * cond_C) / store;
      w_right = (cond_R  + 0.5 * cond_C) / store;
      e_line = w_left - w_right;
      return e_line;
    }


    void calibrate() {

      digitalWrite(LED_BUILTIN, HIGH);

      int store_white_L[n];
      int store_white_C[n];
      int store_white_R[n];

      int store_black_L[n];
      int store_black_C[n];
      int store_black_R[n];


      int jello = n;


      int i = 0;
      Serial.println("Place on white surface");
      delay(2000);

      while (i < n) {
        read_linesensors();
        store_white_L[i] = data[0];
        store_white_C[i] = data[1];
        store_white_R[i] = data[2];

        i ++;
      }
      digitalWrite(LED_BUILTIN, LOW);


      delay(3000);
      Serial.println("place on Black surface");
      digitalWrite(LED_BUILTIN, HIGH);
      delay(2000);

      i = 0;
      while ( i < n) {

        read_linesensors();
        store_black_L[i] = data[0];
        store_black_C[i] = data[1];
        store_black_R[i] = data[2];
        i ++;
      }
      digitalWrite(LED_BUILTIN, LOW);


      L_offset = getMean(store_white_L, jello);
      C_offset = getMean(store_white_C, jello);
      R_offset = getMean(store_white_R, jello);

      S_L = 1 / (getMax(store_black_L, jello) - getMin(store_white_L, jello));
      S_C = 1 / (getMax(store_black_C, jello) - getMin(store_white_C, jello));
      S_R = 1 / (getMax(store_black_R, jello) - getMin(store_white_R, jello));

      /*
        Serial.println("*******");
        Serial.println("Left scale*1000, offset");
        Serial.println(S_L * 1000);
        Serial.println(L_offset);
      */

    }

    float getMin(int list[], int j) {
      int k = 0;
      int val = list[0];
      while (k < j) {
        if (list[k] < val) {
          val = list[k];
        }
        k ++;
      }
      return (float)val;
    }

    float getMax(int list[n], int j) {
      int k = 0;
      int val = list[0];
      while (k < j) {
        if (list[k] > val) {
          val = list[k];
        }
        k ++;
      }
      return (float)val;
    }

    float getMean(int list[], int j) {
      int k = 0;
      int val = 0;
      while (k < j) {
        val += list[k];
        k++;
      }
      val = val / j;
      return (float)val;
    }

    bool on_line() {
      if (cond_C > 0.16) { //0.20 was good enough
        return true;
      }
      else {
        return false;
      }
    }
};



#endif
