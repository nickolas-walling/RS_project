// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

//gear ratio: 29.86 * 12 = 358.3. i.e. 358.3 counts per revolution

// Class to track robot position.
class Kinematics_c {
  public:

    // Constructor, must exist.
    Kinematics_c() {

    }

    float count2move = 0.28058;//mm per count
    float L = (85) / 2; //radius from center of robot to center of one wheel, mm
    float X = 0;
    float Y = 0;
    float T = 0;
    int wheel_rad = 16;
    int last_count_left = 0;
    int last_count_right = 0;
    float cnts_per_rev = 358.3;

    int counts_L;
    int counts_R;

    int prev_L = 0;
    int prev_R = 0;

    float velocity_L_rad;
    float velocity_R_rad;

    float velocity_L_cnts;
    float velocity_R_cnts;

    float velocity_L_mm;
    float velocity_R_mm;


    // Use this function to update
    // your kinematics
    void update() {
      int temp_count_left = 0;
      int temp_count_right = 0;

      temp_count_left = count_LEFT - last_count_left;
      temp_count_right = count_RIGHT - last_count_right;

      last_count_left = count_LEFT;
      last_count_right = count_RIGHT;

      X = X + cos(T) * (cnt2dist(temp_count_left) + cnt2dist(temp_count_right)) / 2;
      Y = Y + sin(T) * (cnt2dist(temp_count_left) + cnt2dist(temp_count_right)) / 2;
      T = T + (cnt2dist(temp_count_left) - cnt2dist(temp_count_right)) / (2 * L);

      if (T >= 3.14159 * 2) {
        T = T - 3.14159 * 2;
      }
      if (T <= -3.14159 * 2) {
        T = T + 3.14159 * 2;
      }
    }

    void velocity(unsigned long elapsed) {

      counts_L = (int)count_LEFT - prev_L;
      prev_L = (int)count_LEFT;
      velocity_L_cnts = (float)counts_L / ((float)elapsed * 1e-3);


      counts_R = (int)count_RIGHT - prev_R;
      prev_R = (int)count_RIGHT;
      velocity_R_cnts = (float)counts_R / ((float)elapsed * 1e-3);


      velocity_L_rad = cnt2rad(velocity_L_cnts);
      velocity_R_rad = cnt2rad(velocity_R_cnts);

      velocity_L_mm = cnt2dist(velocity_L_cnts);
      velocity_R_mm = cnt2dist(velocity_R_cnts);
    }

    float cnt2dist(int num) {
      return (float)num * count2move;
    }

    float cnt2rad(int num) {
      return (float)num * 2 * 3.14159 / cnts_per_rev;
    }

};



#endif
