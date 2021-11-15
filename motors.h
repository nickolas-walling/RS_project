// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H

# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15

// Class to operate the motor(s).
class Motors_c {
  public:
#define P5 5
    // Constructor, must exist.
    Motors_c() {

    }

    // Use this function to
    // initialise the pins and
    // state of your motor(s).
    void initialize() {
      pinMode(LED_BUILTIN, OUTPUT);
      pinMode(L_PWM_PIN, OUTPUT);
      pinMode(L_DIR_PIN, OUTPUT);
      pinMode(R_PWM_PIN, OUTPUT);
      pinMode(R_DIR_PIN, OUTPUT);

      analogWrite(L_PWM_PIN, 0);
      analogWrite(R_PWM_PIN, 0);
    }

    void halt() {
      analogWrite(L_PWM_PIN, 0);
      analogWrite(R_PWM_PIN, 0);

    }
    // Write a function to operate
    // your motor(s)
    // ...

    void left(float pwm) {

      //check if value is within range

      /*if (pwm < -100 || pwm > 100) {
        Serial.println("Invalid PWM value");
        halt();
        while (1) {
          digitalWrite(LED_BUILTIN, HIGH);
          delay(250);
          digitalWrite(LED_BUILTIN, LOW);
          delay(250);
        }
        }*/

      if (pwm < -100) {
        pwm = -100;
      }
      else if (pwm > 100) {
        pwm = 100;
      }
      else {

        //set motor direction
        if (pwm < 0) {
          digitalWrite(L_DIR_PIN, HIGH);
        }
        else {
          digitalWrite(L_DIR_PIN, LOW);
        }

        //set motor power
        analogWrite(L_PWM_PIN, abs(pwm));
      }
    }

    void right(float pwm) {

      //check if value is within range
      /*
            if (pwm < -100 || pwm > 100) {
              halt();
              Serial.println("Invalid PWM value");
              while (1) {
                digitalWrite(LED_BUILTIN, HIGH);
                delay(250);
                digitalWrite(LED_BUILTIN, LOW);
                delay(250);
              }
            }*/
      if (pwm < -100) {
        pwm = -100;
      }
      else if (pwm > 100) {
        pwm = 100;
      }
      else {

        //set motor direction
        if (pwm < 0) {
          digitalWrite(R_DIR_PIN, HIGH);
        }
        else {
          digitalWrite(R_DIR_PIN, LOW);
        }

        //set motor power
        analogWrite(R_PWM_PIN, abs(pwm));
      }

    }


};



#endif
