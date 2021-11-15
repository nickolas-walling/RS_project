// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:

    // Constructor, must exist.
    PID_c() {

    }

    float K_p;
    float K_i;
    float K_d;

    float feedback_p;
    float feedback_i;
    float feedback_d;
    float feedback;

    unsigned long dt;
    unsigned long start_ts;

    void initialize(float p, float i, float d) {
      feedback = 0;
      feedback_i = 0;
      K_p = p;
      K_i = i;
      K_d = d;
      start_ts = millis();
    }

    float update(float demand, float measurement) {
      float err = 0;
      unsigned long current_ts;

      current_ts = millis();

      dt = current_ts - start_ts;

      feedback_d = K_d * (demand - measurement - err) / (float)dt;

      err = demand - measurement;

      feedback_p = K_p * err;

      feedback_i = K_i * err * (float)dt + feedback_i;

      feedback = feedback_p + feedback_i + feedback_d;

      start_ts = millis();

      return feedback;
    }


    void reset() {

      start_ts = millis();

      feedback_i = 0;

    }
};



#endif
