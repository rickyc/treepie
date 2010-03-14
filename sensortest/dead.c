/*
	 3PI template code for NYU "Intro to Robotics" course. Yann LeCun, 02/2010.
	 This program was modified from an example program from Pololu.
*/
#include <pololu/3pi.h> // Required for all 3pi programs
#include <avr/pgmspace.h> // Required for the use of program space for data
#include "calibration.h" // Alters the standard behavior of the 3pi_kinematics.h file

#define MIN_MOTOR_SPEED 0
#define MAX_MOTOR_SPEED 255
#define MILLION 1000000
#define DEBUG 1

// Global arrays to hold min and max sensor values for calibration
unsigned int sensors[5]; // global array to hold sensor values
unsigned int minv[5] = {65000, 65000, 65000, 65000, 65000};
unsigned int maxv[5] = {0};

// Global for robot base motor setting
int rotation = 40;

// Global to track whether the robot is to be running in the main loop
int run = 0; // if =1 run the robot, if =0 stop

// Globals that track position relative to the robot boot location (origin)
long xPos = 0;
long yPos = 0;

// A couple of simple tunes, stored in program space.
const char welcome[] PROGMEM = ">g32>>c32";
const char thank_you_music[] PROGMEM = ">>c32>g32";
const char beep_button_top[] PROGMEM = "!c32";
const char beep_button_middle[] PROGMEM = "!e32";
const char beep_button_bottom[] PROGMEM = "!g32";
const char timer_tick[] PROGMEM = "!v8>>c32";

// Introductory messages. The "PROGMEM" identifier
// causes the data to go into program space.
const char robotName[] PROGMEM = "PONY";

char display_characters[9] = { ' ', 0, 1, 2, 3, 4, 5, 6, 255 };

void idle_until_button_pressed(button) {
	while(!button_is_pressed(button)) { delay_ms(100); }
	wait_for_button_release(button);
}

// helper functions
void toggleRun() { run = 1-run; }
void stop_motors() { set_motors(0,0); delay_ms(250);}

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
long line_position() {
	int i;
	long sum = 0;
	long count = 0;
	int adjustment[5] = {-1000, -500, 0, 500, 1000};
	for (i = 0; i < 5; i++) {
		long min = (long)minv[i];   // tiny efficiency gain here
		long dist = (10*((long)sensors[i]-min))/((long)maxv[i]-min); // 0-100
		sum += dist*adjustment[i];  // 0-100's weighted by adjustment amount
		count += dist;              // sum of 0-100's
	}
	return sum/count; // between -1000 and +1000
}

// Returns 1 if off the line, 0 if on the line
int off_track(center_only) {
	int i = 0;
	int count = 5;
	if(center_only){ // Sets loop to look only at center sensor
		i = 2;
		count = 3;
	}
	for(; i < count; i++){
		int min = minv[i];
		long dist = (100*((long)sensors[i]-min))/((long)maxv[i]-min); // 0-100
		if (dist > 25) { return 0; } // Threshold for declaring not-a-line
	}
	return 1; // Fell through the guard, thus off the line
}

// Calculates the time taken to travel a set distance
// Currently assumed to be 20cm.
int two_line_time(speed){
	idle_until_button_pressed(BUTTON_A);
	set_motors(speed, speed);
	int speed_time = 0;
	int first_mark_time = 0;
	int second_mark_time = 0;

	while(1) {
		read_line_sensors(sensors, IR_EMITTERS_ON);
		if(off_track(0) && !first_mark_time){
			first_mark_time = millis();
		} else if(off_track(0) && !second_mark_time && (millis()-first_mark_time > 2000)) {
			second_mark_time = millis();
		}
		if (second_mark_time){
			stop_motors();
			speed_time = second_mark_time - first_mark_time;
			break;
		}
	}
	return speed_time;
}

// Automatically determines values for motor2speed internal constants
void speed_calibrate(int first_speed, int second_speed){
	int first_speed_time;
	int second_speed_time;

	clear();
	lcd_goto_xy(0,0);
	print("Speed Test");
	first_speed_time = two_line_time(first_speed);

	//requires a user rotate here
	lcd_goto_xy(0,1);
	print("Test 2");
	second_speed_time = two_line_time(second_speed);

	update_calibration(first_speed, second_speed, first_speed_time, second_speed_time);

	clear();
	return;
}

// Make a little dance: Turn left and right
void dance() {
	int counter;
	for(counter=0;counter<80;counter++) {
		if(counter <= 20 || counter >= 60) { set_motors(40,-40);
		} else { set_motors(-40,40); }
		// Since our counter runs to 80, the total delay will be 80*20 = 1600 ms.
		read_line_sensors(sensors, IR_EMITTERS_ON);
		update_bounds(sensors,minv,maxv);
		delay_ms(20);
	}
	stop_motors();
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

// Debugger Code
void debug_1(long l) { lcd_goto_xy(0,0); print_long(l); }
void debug_2(long l) { lcd_goto_xy(0,1); print_long(l); }
void debug_a(long l_one, long l_two) { clear(); debug_1(l_one); debug_2(l_two); }

// running the motors can cause the 3pi to go straight or make turns
void run_motors_for_X_seconds(leftMotor,rightMotor,secondsToTurn) {
	int i;
	for (i=0;i<secondsToTurn;++i) { 
		set_motors(leftMotor,rightMotor);
		delay_ms(10);
	}
	stop_motors();
}

void run_motor_for_X_seconds(motorSpeed,seconds) {
	run_motors_for_X_seconds(motorSpeed,motorSpeed,seconds);
}

// This is the main function, where the code starts. All C programs
// must have a main() function defined somewhere.
int main () {  
  
  // line position relative to center
  long position = 0;
  long oldPosition = 0;
  long derivative = 0;
  int offset = 0;
  int leftMotor = 0;
  int rightMotor = 0;
  long integral = 0;
  int rotation = 35;
  int i;
  long oldTheta = 0;
  long newTheta = 0;
  long marginalTheta = 0;
  long alpha = 0;
  unsigned long prevTime = 0;
  unsigned long deltaTime = 0;
  
  // set up the 3pi, and wait for B button to be pressed
  initialize();
  idle_until_button_pressed(BUTTON_B);
  read_line_sensors(sensors,IR_EMITTERS_ON);
  dance(); // sensor calibration
  // speed_calibrate(30,60); // call this line to auto-calibrate the robot.
  
  do {
    if (button_is_pressed(BUTTON_B)) {
      play_from_program_space(beep_button_middle);
      toggleRun();
      delay_ms(200);
    }
    
		  oldPosition = position;	// compute line positon
		  prevTime = millis();  //get the first time reading
		  read_line_sensors(sensors, IR_EMITTERS_ON);
		  update_bounds(sensors, minv, maxv);
		  position = line_position();		//get the line position.
		
		if (run == 1) {	
		  
		  // position = -1000 to 1000
		  derivative = position - oldPosition;
		  integral = position + oldPosition;
		  offset = 	position/20 + derivative/25 + integral/35;
    
      leftMotor = rotation + offset;
      rightMotor = rotation - offset;

			// truncation on positives.
      leftMotor = (leftMotor > MAX_MOTOR_SPEED) ? MAX_MOTOR_SPEED : leftMotor;
      rightMotor = (rightMotor > MAX_MOTOR_SPEED) ? MAX_MOTOR_SPEED : rightMotor;

      // truncation on negatives for safety
      leftMotor = (leftMotor < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : leftMotor;
      rightMotor = (rightMotor < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : rightMotor;

			//get the new marginal theta calculation.
      marginalTheta = (long)( motor2angle(leftMotor, rightMotor) * deltaTime);
			
			// add it to the old theta calculation to get the new theta
      newTheta = oldTheta + marginalTheta;
			
			// avg the two theta calculations to get the alpha angle.
      alpha = (oldTheta + newTheta)/2;
			
			// alpha is 10^3 too big. as will be Sin & Cos. deltaTime is in terms of millisecs. 
			// so, divide the resulting product by a million (1000*1000)
			long speedCalc = deltaTime*motor2speed((leftMotor+rightMotor)/2);
      xPos += (long)(Sin(alpha/1000)*speedCalc)/MILLION; 
      yPos += (long)(Cos(alpha/1000)*speedCalc)/MILLION;

      oldTheta = newTheta;

      set_motors(leftMotor, rightMotor);
    }
    
    // debug code
		if (DEBUG) {
			clear();
			lcd_goto_xy(0,0);
			print_long(marginalTheta);
			lcd_goto_xy(1,1);
			print_long(alpha/1000);
		}
    
    delay_ms(40);	//delay the run loop to decrease the number of readings taken.
    deltaTime = millis() - prevTime;	// new deltaTime
    
  } while(!off_track(0));

  // Stop the motors, set the base speed to 40 and attempt to go home.
  stop_motors();
  rotation = 40;

	// debugging.
	if (DEBUG) {
		clear();
		lcd_goto_xy(0,0);
		print("GO HOME");
	}

  int targetTheta = oldTheta/1000; // Reduce tracking-mode theta to scale
	long oldXPos = xPos;					// save these values for later.
	long oldYPos = yPos;
	
	// figure out which direction to turn in to make the robot point
	// 180 degrees from its initial starting position. it is in one of four
	// quadrants based upon the last theta calculation.
  if (targetTheta > 0 && targetTheta <= 180) {
    targetTheta = 180 - targetTheta;
    leftMotor = rotation; //Clockwise
    rightMotor = -rotation;
  } else if (targetTheta < -180) {
    targetTheta = targetTheta + 360;
    leftMotor = rotation; //Clockwise
    rightMotor = -rotation;
  } else if (targetTheta > 180) {
    targetTheta = 360 - targetTheta;
    leftMotor = -rotation; //Counter-clockwise
    rightMotor = rotation;
  } else { //Thus tTheta is between -180 and 0
    targetTheta = 180 + targetTheta;
    leftMotor = -rotation; //Counter-clockwise
    rightMotor = rotation;
  }
 
  // turn the robot by this many centiseconds
  long secondsToTurn = motor2angle(leftMotor,rightMotor); //Unit: Deg/sec
  secondsToTurn = (110*targetTheta)/secondsToTurn; //Unit: Deg/(Deg/sec)
  if (secondsToTurn < 0) secondsToTurn = -secondsToTurn;  //Flip if negative turn time

	// debug code.
	if (DEBUG) {
		clear();
		print_long(secondsToTurn);
		lcd_goto_xy(1,1);
		print_long(targetTheta);
	}

 	// a loop that takes 10ms to process each centisecond. Ergo, we
 	// turn the robot by this many milliseconds.
	run_motors_for_X_seconds(leftMotor,rightMotor,secondsToTurn);
  
	prevTime = millis();	// reset the deltaTime tracking for this loop.
	deltaTime = 0;			// resetting deltaTime
	while (yPos > 0) { 
		set_motors(rotation, rotation);
		yPos -= motor2speed(rotation)*deltaTime/1000;		//deltaTime is in terms of seconds.
		delay_ms(100);									// slow the loop down some.													
		deltaTime = millis()-prevTime;		
		prevTime = millis();						// reset the previous time value
	}
	stop_motors(); 

	// done turning and traversing the yDistance.
  // turn by 90 degrees to the right or left.
  targetTheta = 90;

  // turn robot to the proper angle based upon where it began.
  // this is determined by what quadrant it was in when it detected the 
  // end of the line. the values of the x and y coordinates tell us which
  // way to turn to get back to the origin after hitting the x-axis 
  if ((oldXPos > 0 && oldYPos > 0) || (oldXPos < 0 && oldYPos < 0)) {
   	leftMotor = rotation;
   	rightMotor = -rotation;		// q1 && q3
   } else {
  	leftMotor = -rotation;		// q2 & q4
  	rightMotor = rotation;
  }

	// re-calculate the time to rotate again, this time to point to the origin.
  secondsToTurn = motor2angle(leftMotor,rightMotor); //Unit: Deg/sec
  secondsToTurn = (110*targetTheta)/secondsToTurn; //Unit: Deg/(Deg/sec)
  
  if (secondsToTurn < 0) secondsToTurn = -secondsToTurn;  //Flip if negative turn time

	// debugging.
	if (DEBUG) {
		clear();
		print_long(secondsToTurn);
		lcd_goto_xy(1,1);
		print_long(targetTheta);
	}

  // do the rotation. same code as above for the yPos.
	run_motors_for_X_seconds(leftMotor,rightMotor,secondsToTurn);

	if (xPos < 0) xPos = -xPos;		// flip x if it's negative.	
	  
	prevTime = millis();	// reset the deltaTime tracking for this loop.
	deltaTime = 0;			// resetting deltaTime
	while (xPos > 0) { 	// this time, decrement X till we're done.
		set_motors(rotation, rotation);
		xPos -= motor2speed(rotation)*deltaTime/1000;	
		delay_ms(100);		// delay to slow the loop down.
		deltaTime = millis()-prevTime;
		prevTime = millis();
	}
	stop_motors(); 
	
	// DONE! YAYYYYYYYYYYY! :)
  return 0;
}

