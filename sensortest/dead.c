/*
	 3PI template code for NYU "Intro to Robotics" course. Yann LeCun, 02/2010.
	 This program was modified from an example program from Pololu.
*/
// Required for all 3pi programs
#include <pololu/3pi.h>
// Required for the use of program space for data
#include <avr/pgmspace.h>
// Alters the standard behavior of the 3pi_kinematics.h file
#include "calibration.h"

#define MIN_MOTOR_SPEED 0
#define MAX_MOTOR_SPEED 255

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
const char robotName[] PROGMEM = "MAGIC PONY";

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

// (TODO) 4 Alex Lobasian. Yah, know how you put a space after a period. Like this?
// So you put a space after a comment -.-
//Not like This
//---- FYI same practice in ruby (DELETE previous four lines after you are done)
//Calculates the time taken to travel a set distance
//Currently assumed to be 20cm.
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
		} else if(off_track(0) && !second_mark_time && (millis()-first_mark_time > 200)) {
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

void run_motors_for_X_seconds(leftMotor,rightMotor,seconds) {
	long i;
	for (i=0;i<seconds;++i) { 
		set_motors(leftMotor,rightMotor);
		delay_ms(10);
	}
	stop_motors();
}

void run_motor_for_X_seconds(motorSpeed,seconds) {
	run_motors_for_X_seconds(motorSpeed,motorSpeed,seconds);
}

void turn_to_angle(long angle, int leftMotor, int rightMotor) {
	// int leftMotor = rotation;	completely wrong.
	// int rightMotor = -rotation;	ditto.

	long secondsToTurn = motor2angle(leftMotor,rightMotor); // Unit: Deg/sec
	// MAGIC NUMBER!!! This line was called in the turn to X Axis
	// secondsToTurn = (100*targetTheta)/secondsToTurn; // Unit: Deg/(Deg/sec)

	// This was called in turn 90 degrees
	secondsToTurn = (100*angle)/secondsToTurn; // Unit: Deg/(Deg/sec)
												//note: maybe 120 instead?

	if (secondsToTurn < 0) secondsToTurn = -secondsToTurn;  // Flip if negative turn time
	debug_a(secondsToTurn, angle);
	run_motors_for_X_seconds(leftMotor,rightMotor,secondsToTurn);

	// if (xPos < 0) xPos = -xPos;							// TODO: WTF IS THIS?
	// long xSeconds = (xPos*100)/motor2speed(rotation); // TODO: Explain units
	// run_motor_for_X_seconds(rotation,xSeconds);
}

int main() {
	// line position relative to center
	long position = 0;
	long oldPosition = 0;
	long derivative = 0;
	int offset = 0;
	int leftMotor = 0;
	int rightMotor = 0;
	long integral = 0;
	long i;
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

	// Alex's Calibration Madness
	// speed_calibrate(25,50); // TODO: Turn on when its actually working
	idle_until_button_pressed(BUTTON_B);

	do {
		// (TODO) remove comment on next iteration, this does work, basically if button is hit
		// the robot stops running
		if(button_is_pressed(BUTTON_B)) { 
			play_from_program_space(beep_button_middle); 
			toggleRun();
			delay_ms(200);
		}

		if (run == 1) {
			
			oldPosition = position;	// compute line positon 
			prevTime = millis();  //get the first time reading TODO: Is this necessary at this point?
			read_line_sensors(sensors, IR_EMITTERS_ON);
			update_bounds(sensors, minv, maxv);
			position = line_position();

			// offset needs deltaTime. add to deltaTime the amount of time it took
			// to go from the first time reading till now.
			// we need to incorporate the past loop's run-time in addition to the
			// part of the while loop traversed so far.
			//    deltaTime = millis() - prevTime;
			//    prevTime = millis();

			derivative = position - oldPosition; // TODO*
			integral = position + oldPosition; // tracks long-running position offset TODO*
			offset = 	position/11 + derivative/30 + integral/50; //TODO *
	
			leftMotor = rotation + offset;
			rightMotor = rotation - offset;
      // Guard against higher-than-acceptable values
			leftMotor = (leftMotor > MAX_MOTOR_SPEED) ? MAX_MOTOR_SPEED : leftMotor;
			rightMotor = (rightMotor > MAX_MOTOR_SPEED) ? MAX_MOTOR_SPEED : rightMotor;
			// Guard against lower-than-acceptable values
			leftMotor = (leftMotor < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : leftMotor;
			rightMotor = (rightMotor < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : rightMotor;

			marginalTheta = (long)( motor2angle(leftMotor, rightMotor) * deltaTime); //TODO: Units

			newTheta = oldTheta + marginalTheta; //TODO: Perhaps scale down to 0-360 or -180-+180 here

			alpha = (newTheta+oldTheta)/2; 

			xPos += (Sin(alpha/1000)*deltaTime*motor2speed((leftMotor+rightMotor)/2))/1000000; // LOVE MAGIC NUMBERS TODO: Units
			yPos += (Cos(alpha/1000)*deltaTime*motor2speed((leftMotor+rightMotor)/2))/1000000; 

			oldTheta = newTheta;

			set_motors(leftMotor, rightMotor);
		}

		// debug code
		debug_a(xPos, yPos);
		delay_ms(10);

		deltaTime = millis() - prevTime; // new deltaTime /*TODO: This happens so much it's not even funny. Lock it down.*/
		// ^ I don't like this either

	} while(!off_track(0));

	// Stop the motors, set the base speed to 40 and attempt to go home.
	stop_motors();
	rotation = 40;
	
	clear();
	lcd_goto_xy(0,0);
	print("I am lost"); // aka go home

	int targetTheta = oldTheta/1000; // Reduce tracking-mode theta to scale

	// if it's a positive angle, subtract it from 180 and then make the right motor neg 
	// and the left motor positive to spin clockwise.
	if (targetTheta > 0 && targetTheta <= 180) { //TODO: Clean up the logic/verify truth
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

	clear();
	print_long(targetTheta/1000);	// TODO: WTF? Why divide this by 1000 AGAIN? 

	// The 180 degrees turn is now complete and it should be facing 100 //Um. 100 degrees?
	// degrees to its starting position.
	turn_to_angle(targetTheta, leftMotor, rightMotor); 

	// Go Y distance
	// flip the yPos value if negative
	if (yPos < 0) yPos = -yPos;

//	long ySeconds = (yPos*100)/motor2speed(rotation);	agreed: try function 2 first.
	// the 100 is to prevent the number from reduction to zero ^, yPos = 10/300 = 0

//	run_motor_for_X_seconds(rotation,ySeconds);		see above.

//	turn_to_angle(90); //Shouldn't it be turn 90degrees CW, aka turn to -90||+270
	//travel distance of yPos, but flip the yPos value if negative
	if (yPos < 0) yPos = -yPos;
	
	prevTime = millis();	// reset the deltaTime tracking for this loop.
	deltaTime = 0;			// resetting deltaTime
	while (yPos > 0) { //TODO: Factor  into fn: drive(dist, motor setting); -- This was factored, except 
		// there are two algorithms that do the same thing, Basically this is the same as run_motor_for_X_seconds
		// I have to say this algorithm would probably be more accurate assuming the X,Y is accurate just due to the
		// fact it's reversing the count
		set_motors(rotation, rotation);
		yPos += (Cos(180)*motor2speed(rotation)*deltaTime)/1000000;		// TODO: cosine is * 1000 and delaTime is in milliseconds.
		delay_ms(10);													// divide by 1000*1000 = million.
		deltaTime = millis()-prevTime;
		prevTime = millis();
	}
	stop_motors(); //end fn "drive"
	
/*	one step at a time, gentlemen...

	// turn by 90 degrees to the right or left.
	targetTheta = 90;
	// (BUG) There is no turn here, the theta was just reset to 90 and uses
	// a different algorithm to do the turn

	// turn robot to the proper angle based upon where it began. 
	// (TODO) questa e` una diversi funzione per andare a casa. (That's definitely incorrect italian)
	// farsi
	if (oldXPos > 0 && oldYPos > 0) set_motors(rotation, 0); //Quadrant 1
    else if (oldXPos < 0 && oldYPos > 0) set_motors(0, rotation); //2
    else if (oldXPos < 0 && oldYPos < 0) set_motors(rotation, 0); //3
	else set_motors(0, rotation); //4

	deltaTime = millis(); //Definitely incorrect (TODO)
	while (targetTheta > 0) {
		targetTheta -= motor2speed(10)*deltaTime; //TODO: change the motor speed in here, also factor
		delay_ms(10);
		deltaTime = millis() - deltaTime;
	}
	stop_motors();
	
  // flip xPos (TODO) bound check unnecessary now? I mean might as well leave it there
	if (xPos < 0) xPos = -xPos;
	// drive(xPos, rotation)
	set_motors(rotation,rotation);
	deltaTime = millis();
	while (xPos > 0) {
		xPos -= motor2speed(rotation) * deltaTime;
		delay_ms(10);
		deltaTime = millis() - deltaTime;
	}//end drive

	stop_motors();
*/
	return 0;
}

