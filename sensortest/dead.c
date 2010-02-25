/*
	 3PI template code for NYU "Intro to Robotics" course. Yann LeCun, 02/2010.
	 This program was modified from an example program from Pololu.
*/

// The 3pi include file must be at the beginning of any program that
// uses the Pololu AVR library and 3pi.
#include <pololu/3pi.h>

// This include file allows data to be stored in program space. The
// ATmega168 has 16k of program space compared to 1k of RAM, so large
// pieces of static data should be stored in program space.
#include <avr/pgmspace.h>
#include "3pi_kinematics.h"
#define MIN_MOTOR_SPEED 0
#define MAX_MOTOR_SPEED 255

// global arrays to hold min and max sensor values for calibration
unsigned int sensors[5]; // global array to hold sensor values
unsigned int minv[5] = {65000, 65000, 65000, 65000, 65000};
unsigned int maxv[5] = {0};

// A couple of simple tunes, stored in program space.
const char welcome[] PROGMEM = ">g32>>c32";
const char thank_you_music[] PROGMEM = ">>c32>g32";
const char beep_button_top[] PROGMEM = "!c32";
const char beep_button_middle[] PROGMEM = "!e32";
const char beep_button_bottom[] PROGMEM = "!g32";
const char timer_tick[] PROGMEM = "!v8>>c32";

int speed = 100; // speed of the robot
int run = 0; // if =1 run the robot, if =0 stop

// Introductory messages. The "PROGMEM" identifier
// causes the data to go into program space.
const char robotName[] PROGMEM = " TURTLE";

char display_characters[9] = { ' ', 0, 1, 2, 3, 4, 5, 6, 255 };

// This function loads custom characters into the LCD. Up to 8
// characters can be loaded; we use them for 6 levels of a bar graph
// plus a back arrow and a musical note character.
void load_custom_characters() {
	// Data for generating the characters used in load_custom_characters
	// and display_readings. By reading levels[] starting at various
	// offsets, we can generate all of the 7 extra characters needed for a
	// bargraph. This is also stored in program space.
	static const char levels[] PROGMEM = {
		0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000,
		0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111
	};

	// This character is a musical note.
	static const prog_char note[] PROGMEM = {
		0b00100, 0b00110, 0b00101, 0b00101, 0b00100, 0b11100, 0b11100, 0b00000,
	};

	// This character is a back arrow.
	static const prog_char back_arrow[] PROGMEM = {
		0b00000, 0b00010, 0b00001, 0b00101, 0b01001, 0b11110, 0b01000, 0b00100,
	};

	lcd_load_custom_character(levels+0,0); // no offset, e.g. one bar
	lcd_load_custom_character(levels+1,1); // two bars
	lcd_load_custom_character(levels+2,2); // etc...
	lcd_load_custom_character(levels+4,3); // skip level 3
	lcd_load_custom_character(levels+5,4);
	lcd_load_custom_character(levels+6,5);
	lcd_load_custom_character(back_arrow,6);
	lcd_load_custom_character(note,7);
	clear(); // the LCD must be cleared for the characters to take effect
}

// This function displays the sensor readings using a bar graph.
void display_bars(const unsigned int *s, const unsigned int *minv, const unsigned int* maxv) {
	// Initialize the array of characters that we will use for the
	// graph. Using the space, and character 255 (a full black box).
	unsigned char i;
	for (i=0;i<5;i++) {
		int c = ((int)s[i]-(int)minv[i])*9/((int)maxv[i]-(int)minv[i]);
		c = (c<0)?0:(c>8)?8:c;
		// if (i==0) { print_long(s[0]); print_long(c); }
		print_character(display_characters[c]);
	}
}

void update_bounds(const unsigned int *s, unsigned int *minv, unsigned int *maxv) {
	int i;
	for (i=0; i<5; i++) {
		unsigned int val = s[i];
		if (val<minv[i]){ minv[i] = val; }
		if (val>maxv[i]){ maxv[i] = val; }
	}
}

// Return line position
long line_position(unsigned int *s, unsigned int *minv, unsigned int *maxv) {
	int i;
	long sum = 0;
	long count = 0;
	int adjustment[5] = {-2000, -1000, 1, 1000, 2000};
	for (i = 0; i < 5; i++) {
		long min = (long)minv[i];   // tiny efficiency gain here
		long dist = (100*((long)s[i]-min))/((long)maxv[i]-min); // 0-100
		sum += dist*adjustment[i];  // 0-100's weighted by adjustment
		count += dist;              // sum of 0-100's
	}
	return sum/count; // between -2000 and +2000
}

int off_track(center_only) {
  if(center_only){
    int min = minv[2];
    long dist = (100*((long)sensors[2]-min))/((long)maxv[2]-min);
    if (dist > 50){
      return 0;
    } else {
      return 1;
    }
  }
  int i;	
  for(i = 0; i < 5; i++){       // sensors.each
    int min = minv[i];
    long dist = (100*((long)s[i]-min))/((long)maxv[i]-min); //0-100
    if(dist > 50)
      return 0;
  }
  return 1;
}
// Displays the battery voltage.
void battery_reading() {
  unsigned int bat = read_battery_millivolts();
  print_long(bat);
  print("mV");
  delay_ms(250);
}

void speed_calibrate(int first_speed, int second_speed){
  clear();
  print("Speed Test");
  lcd_goto_xy(0,1);
  
  int first_speed_time = 0;
  int second_speed_time = 0;
  int first_mark_time = 0;
  int second_mark_time = 0;
  
  wait_for_button_release(BUTTON_B);
  set_motors(first_speed, first_speed);
  while(1){
    if(off_track(0) && !first_mark_time){
      first_mark_time = millis();
    } else if(off_track(0) && !second_mark_time && (milis() - first_mark_time > 200)) {
      second_mark_time = millis();
    }
    if (second_mark_time){
      set_motors(0,0);
      first_speed_time = second_mark_time - first_mark_time;
      break;
      //requires a user rotate here
    }
  }
  wait_for_button_release(BUTTON_B);
  first_mark_time = second_mark_time = 0;
  set_motors(second_speed, second_speed);
  while(1){
    if(off_track(0) && !first_mark_time){
      first_mark_time = millis();
    } else if(off_track(0) && !second_mark_time && (millis() - first_mark_time > 200)) {
      second_mark_time = millis();
    }
    if (second_mark_time){
      set_motors(0,0);
      second_speed_time = second_mark_time - first_mark_time;
      break;
    }
  }
  //do some math to figure out the calibration coefficients from second_speed_time and first_speed_time
  return;
}

void rotation_calibrate(int first_speed, int second_speed){
  clear();
  print("Rotation Test");
  lcd_goto_xy(0,1);
  
  int first_rotation_time = 0;
  int second_rotation_time = 0;
  int first_cross_time = 0;
  int second_cross_time = 0;
  
  wait_for_button_release(BUTTON_B);
  set_motors(first_speed, -first_speed);
  while(1){
    if(off_track(1) && !first_cross_time){
      first_cross_time = millis();
    } else if (off_track(1) && !second_cross_time && (millis() - first_cross_time > 200)){
      second_cross_time = millis();
    }
    if(second_cross_time){
      first_rotation_time = second_cross_time - first_cross_time;
      break;
    }
  }
  first_cross_time = second_cross_time = 0;
  set_motors(second_speed,second_speed);
  while(1){
    if(off_track(1) && !first_cross_time){
      first_cross_time = millis();
    } else if (off_track(1) && !second_cross_time && (millis() - first_cross_time > 200)){
      second_cross_time = millis();
    }
    if(second_cross_time){
      set_motors(0,0);
      second_rotation_time = second_cross_time - first_cross_time;
      break;
    }
  }
  //do calculation math here, then save the constants appropriately
  return;
}

// Make a little dance: Turn left and right
void dance() {
  int counter;
  for(counter=0;counter<80;counter++) {
    if(counter < 20 || counter >= 60) { set_motors(40,-40);
    } else { set_motors(-40,40); }
    // Since our counter runs to 80, the total delay will be 80*20 = 1600 ms.
    read_line_sensors(sensors, IR_EMITTERS_ON);
    update_bounds(sensors,minv,maxv);
    delay_ms(20);
  }
  set_motors(0,0);
}

void idle() {
  // Display calibrated values as a bar graph.
  while(!button_is_pressed(BUTTON_B)) {
    unsigned int position = read_line(sensors,IR_EMITTERS_ON);
    clear();
    print_long(position);
    lcd_goto_xy(0,1);
    //display_readings(sensors);
    delay_ms(100);
  }
  
  run = 1;
  wait_for_button_release(BUTTON_B);
  clear();
}

// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.
void initialize() {
  pololu_3pi_init(2000);
  load_custom_characters(); // load the custom characters
  print_from_program_space(robotName);
  lcd_goto_xy(0,1);
  print("Press B");
}

// This is the main function, where the code starts. All C programs
// must have a main() function defined somewhere.
int main() {
  
  // line position relative to center
  long position = 0;
  long prev_position = 0;
  long offset = 0;
  long rotation = 20;
  long xPos = 0;
  long yPos = 0;
  long oldTheta = 0;
  long newTheta = 0;
  long alpha = 0;
  unsigned long prevTime = 0;
  unsigned long deltaTime = 0;

  // set up the 3pi, and wait for B button to be pressed
  initialize();

  idle(sensors);
  read_line_sensors(sensors,IR_EMITTERS_ON);
  dance(); // sensor calibration
  speed_calibrate(40,80);
  rotation_calibrate(40,80);

  // display calibrated sensor values as a bar graph.
  do {
    // button press adjustments (RFCT)
    if (button_is_pressed(BUTTON_B)) {
      play_from_program_space(beep_button_middle);
      run = 1-run;
      delay_ms(200);
    } 

    prevTime = millis();  //get the first time reading 		
    // read the line sensor values
    read_line_sensors(sensors, IR_EMITTERS_ON);
    // update minv and mav values and put normalized values in v
    update_bounds(sensors, minv, maxv);
    prev_position = position;         // compute line positon
    position = line_position(sensors, minv, maxv);

    // offset needs deltaTime. add to deltaTime the amount of time it took 
    // to go from the first time reading till now.
    // we need to incorporate the past loop's run-time in addition to the 
    // part of the while loop traversed so far.
    deltaTime = deltaTime + millis() - prevTime;

    offset = position;

    if (run == 1) {
      int leftMotor = rotation + offset;
      int rightMotor = rotation - offset;

      newTheta = oldTheta + (long)(motor2angle(leftMotor, rightMotor) * deltaTime);

      alpha = (oldTheta + newTheta)/2;

      xPos += (long)Sin(alpha)*deltaTime*motor2speed(rotation);
      yPos += (long)Cos(alpha)*deltaTime*motor2speed(rotation);

      oldTheta = newTheta;

      set_motors(leftMotor, rightMotor);
    }
    delay_ms(3);
    // new deltaTime
    deltaTime = millis() - prevTime;

    // debug code
    lcd_goto_xy(0,1);
    print_long(xPos);
    lcd_goto_xy(0,2);
    print_long(yPos);
    //char display[8];
    //sprintf(display,"%i %i",xPos,yPos);
    //print(display);
    clear();
  } while(!off_track(0));
  
  set_motors(0,0);
  delay_ms(250); //A short pause while holding still after finishing the line.
  // go back home
  set_motors(128,128); // direction reverses, this one needs updating

  delay_ms(10);
  return 0;
}
