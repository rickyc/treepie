#include <avr/pgmspace.h>
#include "3pi_kinematics.h"

//Constants used in m2s calculations
long calibration_distance = 2000; //Unit: .1mm
long cM2S_Num = 238;
long cM2S_Denom = 5;
long cM2S_Intercept = -330;

long motor2speed(int v) {
  long r = ( (v>0)? v : -v ) * cM2S_Num/cM2S_Denom + cM2S_Intercept;
  r = (r>0) ? r : 0;
  if (v>=0) {
    return (r);
  } else {
    return (-r);
  }
}

long motor2angle(int left_motor, int right_motor) {
  long left_vel = motor2speed(left_motor);
  long right_vel = motor2speed(right_motor);
  return (left_vel-right_vel)*c360/c5152;
}

//Units are 3pi_speed x2, milliseconds x2
void update_calibration(int speed_one, int speed_two, int time_one, int time_two){
  long slope_numerator = calibration_distance*1000
  long slope_denom = (speed_two - speed_one)*(time_two - time_one);
  long intercept = (calibration_distance/time_one)-(speed_one*slope_numerator/slope_denom);

  cM2S_Num = slope_numerator; //Unit: .1mm*1k
  cM2S_Denom = slope_denom;   //Unit: 3pi_spd*sec/1k
  cM2S_Intercept = intercept; //Unit: .1mm/sec
  return;
}
