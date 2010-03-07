#include <avr/pgmspace.h>
#include "3pi_kinematics.h"

int cM2S_Num = 238;
int cM2S_Denom = 5;
int cM2S_Intercept = -330;


long motor2speed(int v) {
  int r = ( (v>0)? v : -v ) * cM2S_Num/cM2S_Denom + cM2S_Intercept;
  r = (r>0) ? r : 0;
  if (v>=0) {
    return (long)(r);
  } else {
    return (long)(-r);
  }
}

long motor2angle(int left_motor, int right_motor) {
  long left_vel = motor2speed(left_motor);
  long right_vel = motor2speed(right_motor);
  return (left_vel-right_vel)*c360/c5152;
}

void update_calibration(int first_speed_time, int second_speed_time){
  
  
  return;
}
