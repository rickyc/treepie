#include <avr/pgmspace.h>

////////////////////////////////////////////////////////////////
// Some useful long constants, that are stored in variables
// because of bug in avr-gcc with long constants
long c2 = 2;
long c9 = 9;
long c10 = 10;
long c90 = 90;
long c100 = 100;
long c180 = 180;
long c360 = 360;
long c1000 = 1000;
long c5152 = 5152;
long cmillion = 1000000;

// width of the robot in 1/10 of a millimiter.
// (distance between the two weels)
long robot_width = 820;

////////////////////////////////////////////////////////////////
// trig tables.
// sin and cos tables are in flash (program memory),
// because they don't fit in 1K of RAM
// lush: (for (i 0 359) (printf "%d, " (+ 1000 (* 1000 (sin (* i (/ 3.1415927 180)))))))
const int sin_table[] PROGMEM= 
  {1000, 1017, 1034, 1052, 1069, 1087, 1104, 1121, 1139, 1156, 1173,
   1190, 1207, 1224, 1241, 1258, 1275, 1292, 1309, 1325, 1342, 1358,
   1374, 1390, 1406, 1422, 1438, 1453, 1469, 1484, 1500, 1515, 1529,
   1544, 1559, 1573, 1587, 1601, 1615, 1629, 1642, 1656, 1669, 1681,
   1694, 1707, 1719, 1731, 1743, 1754, 1766, 1777, 1788, 1798, 1809,
   1819, 1829, 1838, 1848, 1857, 1866, 1874, 1882, 1891, 1898, 1906,
   1913, 1920, 1927, 1933, 1939, 1945, 1951, 1956, 1961, 1965, 1970,
   1974, 1978, 1981, 1984, 1987, 1990, 1992, 1994, 1996, 1997, 1998,
   1999, 1999, 1999, 1999, 1999, 1998, 1997, 1996, 1994, 1992, 1990,
   1987, 1984, 1981, 1978, 1974, 1970, 1965, 1961, 1956, 1951, 1945,
   1939, 1933, 1927, 1920, 1913, 1906, 1898, 1891, 1882, 1874, 1866,
   1857, 1848, 1838, 1829, 1819, 1809, 1798, 1788, 1777, 1766, 1754,
   1743, 1731, 1719, 1707, 1694, 1681, 1669, 1656, 1642, 1629, 1615,
   1601, 1587, 1573, 1559, 1544, 1529, 1515, 1499, 1484, 1469, 1453,
   1438, 1422, 1406, 1390, 1374, 1358, 1342, 1325, 1309, 1292, 1275,
   1258, 1241, 1224, 1207, 1190, 1173, 1156, 1139, 1121, 1104, 1087,
   1069, 1052, 1034, 1017, 999, 982, 965, 947, 930, 912, 895, 878, 860,
   843, 826, 809, 792, 775, 758, 741, 724, 707, 690, 674, 657, 641, 625,
   609, 593, 577, 561, 546, 530, 515, 499, 484, 470, 455, 440, 426, 412,
   398, 384, 370, 357, 343, 330, 318, 305, 292, 280, 268, 256, 245, 233,
   222, 211, 201, 190, 180, 170, 161, 151, 142, 133, 125, 117, 108, 101,
   93, 86, 79, 72, 66, 60, 54, 48, 43, 38, 34, 29, 25, 21, 18, 15, 12, 9,
   7, 5, 3, 2, 1, 0, 0, 0, 0, 0, 1, 2, 3, 5, 7, 9, 12, 15, 18, 21, 25,
   29, 34, 38, 43, 48, 54, 60, 66, 72, 79, 86, 93, 101, 108, 117, 125,
   133, 142, 151, 161, 170, 180, 190, 201, 211, 222, 233, 245, 256, 268,
   280, 292, 305, 318, 330, 343, 357, 370, 384, 398, 412, 426, 440, 455,
   470, 484, 500, 515, 530, 546, 561, 577, 593, 609, 625, 641, 657, 674,
   690, 707, 724, 741, 758, 775, 792, 809, 826, 843, 860, 878, 895, 912,
   930, 947, 965, 982
  };

// (for (i 0 359) (printf "%d, " (+ 1000 (* 1000 (cos (* i (/ 3.1415927 180)))))))
const int cos_table[] PROGMEM= 
  {2000, 1999, 1999, 1998, 1997, 1996, 1994, 1992, 1990, 1987, 1984,
   1981, 1978, 1974, 1970, 1965, 1961, 1956, 1951, 1945, 1939, 1933,
   1927, 1920, 1913, 1906, 1898, 1891, 1882, 1874, 1866, 1857, 1848,
   1838, 1829, 1819, 1809, 1798, 1788, 1777, 1766, 1754, 1743, 1731,
   1719, 1707, 1694, 1681, 1669, 1656, 1642, 1629, 1615, 1601, 1587,
   1573, 1559, 1544, 1529, 1515, 1499, 1484, 1469, 1453, 1438, 1422,
   1406, 1390, 1374, 1358, 1342, 1325, 1309, 1292, 1275, 1258, 1241,
   1224, 1207, 1190, 1173, 1156, 1139, 1121, 1104, 1087, 1069, 1052,
   1034, 1017, 999, 982, 965, 947, 930, 912, 895, 878, 860, 843, 826,
   809, 792, 775, 758, 741, 724, 707, 690, 674, 657, 641, 625, 609, 593,
   577, 561, 546, 530, 515, 499, 484, 470, 455, 440, 426, 412, 398, 384,
   370, 357, 343, 330, 318, 305, 292, 280, 268, 256, 245, 233, 222, 211,
   201, 190, 180, 170, 161, 151, 142, 133, 125, 117, 108, 101, 93, 86,
   79, 72, 66, 60, 54, 48, 43, 38, 34, 29, 25, 21, 18, 15, 12, 9, 7, 5,
   3, 2, 1, 0, 0, 0, 0, 0, 1, 2, 3, 5, 7, 9, 12, 15, 18, 21, 25, 29, 34,
   38, 43, 48, 54, 60, 66, 72, 79, 86, 93, 101, 108, 117, 125, 133, 142,
   151, 161, 170, 180, 190, 201, 211, 222, 233, 245, 256, 268, 280, 292,
   305, 318, 330, 343, 357, 370, 384, 398, 412, 426, 440, 455, 470, 484,
   500, 515, 530, 546, 561, 577, 593, 609, 625, 641, 657, 674, 690, 707,
   724, 741, 758, 775, 792, 809, 826, 843, 860, 878, 895, 912, 930, 947,
   965, 982, 1000, 1017, 1034, 1052, 1069, 1087, 1104, 1121, 1139, 1156,
   1173, 1190, 1207, 1224, 1241, 1258, 1275, 1292, 1309, 1325, 1342,
   1358, 1374, 1390, 1406, 1422, 1438, 1453, 1469, 1484, 1500, 1515,
   1529, 1544, 1559, 1573, 1587, 1601, 1615, 1629, 1642, 1656, 1669,
   1681, 1694, 1707, 1719, 1731, 1743, 1754, 1766, 1777, 1788, 1798,
   1809, 1819, 1829, 1838, 1848, 1857, 1866, 1874, 1882, 1891, 1898,
   1906, 1913, 1920, 1927, 1933, 1939, 1945, 1951, 1956, 1961, 1965,
   1970, 1974, 1978, 1981, 1984, 1987, 1990, 1992, 1994, 1996, 1997,
   1998, 1999, 1999 };


////////////////////////////////////////////////////////////////
// Trig functions

// return 1000*sin(angle) where angle is an integer angle in degrees
long Sin(long angle) {
  while (angle < 0) angle += c360;
  return  (long)( (int)pgm_read_word(sin_table + (angle % c360)) - 1000 );
}

// return 1000*cos(angle) where angle is an integer angle in degrees
long Cos(long angle) {
  while (angle < 0) angle += c360;
  return (long)( (int)pgm_read_word(cos_table + (angle % c360)) - 1000 );
}

////////////////////////////////////////////////////////////////
// Functions to convert motor speed to linear and rotational speeds
// The good news is that experiments show the relationship
// to be very close to linear.
// Be careful that this is somewhat inaccurate because 
// the robot does not instantaneously change speed
// due to inertia. Hence, you will get better results
// with soft accelerations.

// converts a motor command to a linear speed in units of
// 0.1 mm/s. A motor command of 100 corresponds to a 
// speed of 457 mm/s (or 4570 units/s).
// This is for robot 493 with fully charged batteries.
// your mileage (or millimeterage) may vary.
long original_motor2speed(int v) {
  // v*4.7682 - 33 mm/s
  // This is robot 493 with fully charged batteries.
  // your mileage (millimeterage) may vary
  int r = ( (v>0)?v:-v )*238/5 - 330;
  r = (r>0)?r:0;
  if (v>=0) {
    return (long)(r);
  } else {
    return (long)(-r);
  }
}

// converts the two motor speeds to
// a rotational speed in degrees per second.
// Positive rotation is clockwise (like the heading of a boat).
long original_motor2angle(int ml, int mr) {
  // a = (vl-vr)*180/(pi*w) = (vl-vr)*360/(2*pi*w)
  // where: 
  // vl,vr: speeds of left and right wheels in 1/10 mm/s.
  // w: width of the robot in 1/10th of mm
  long vl = original_motor2speed(ml);
  long vr = original_motor2speed(mr);
  return (vl-vr)*c360/c5152;
}


////////////////////////////////////////////////////////////////
// OBSOLETE FUNCTION
// converts 1/2 of the speed difference between 
// motors to a rotational speed in degrees per second
// This is for robot 493 with fully charged batteries.
// In other words, if the left wheel is set to v
// and the right wheel speed to -v, this will return
// the number of degrees turned in 1 second when the
// robot is turning in place.
// THERE IS NO REAL REASON TO USE THIS FUNCTION
// rather than motor2angle().
long motor2angle_inplace(int v) {
  // a = v*6.5632 - 39.6316
  // This is robot 493 with fully charged batteries.
  // your mileage may vary
  int r = ((v<0)?-v:v)*197/30 - 40;
  r = (r>0)?r:0;
  if (v >= 0) {
    return (long)(r);
  } else {
    return (long)(-r);
  }
}

