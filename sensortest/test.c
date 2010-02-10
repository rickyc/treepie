/*
  3PI template code for NYU "Intro to Robotics" course. Yann LeCun, 02/2010.
  This program was modified from an example program from Pololu. 
*/

// The 3pi include file must be at the beginning of any program that
// uses the Pololu AVR library and 3pi.
#include <pololu/3pi.h>

// This include file allows data to be stored in program space.  The
// ATmega168 has 16k of program space compared to 1k of RAM, so large
// pieces of static data should be stored in program space.
#include <avr/pgmspace.h>

#define MIN_MOTOR_SPEED -255
#define MAX_MOTOR_SPEED 255

// A couple of simple tunes, stored in program space.
const char welcome[] PROGMEM = ">g32>>c32";
const char thank_you_music[] PROGMEM = ">>c32>g32";
const char beep_button_top[] PROGMEM = "!c32";
const char beep_button_middle[] PROGMEM = "!e32";
const char beep_button_bottom[] PROGMEM = "!g32";
const char timer_tick[] PROGMEM = "!v8>>c32";

int speed = 100;	// speed of the robot
int run = 0;		// if =1 run the robot, if =0 stop

// Introductory messages.  The "PROGMEM" identifier 
// causes the data to go into program space.
const char robotName[] PROGMEM = " TURTLE";

char display_characters[9] = { ' ', 0, 1, 2, 3, 4, 5, 6, 255 };

// This function loads custom characters into the LCD.  Up to 8
// characters can be loaded; we use them for 6 levels of a bar graph
// plus a back arrow and a musical note character.
void load_custom_characters() {
	// Data for generating the characters used in load_custom_characters
	// and display_readings.  By reading levels[] starting at various
	// offsets, we can generate all of the 7 extra characters needed for a
	// bargraph.  This is also stored in program space.
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
	// graph.  Using the space, and character 255 (a full black box).
	unsigned char i;
	for (i=0;i<5;i++) {
		int c = ((int)s[i]-(int)minv[i])*9/((int)maxv[i]-(int)minv[i]);
		c = (c<0)?0:(c>8)?8:c;
		// if (i==0) {print_long(s[0]); print_long(c); }
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
	int adjustment[5] = {-2000, -1000, 0, 1000, 2000};
	
	for (i = 0; i < 5; i++) {
		long minv = (long)minv[i];
		long dist = (100*((long)s[i]-minv))/((long)maxv[i]-minv);
		sum += dist*adjustment[i];
		count += dist;	//sum of 0-100's
	}
	
	return sum/count;	//between -2000 and +2000
}

// Displays the battery voltage.
void battery_reading() {
	unsigned int bat = read_battery_millivolts_svp();
	print_long(bat);
	print("mV");
	delay_ms(250);
}

// Make a little dance: Turn left and right
void dance(unsigned int *s, unsigned int *minv, unsigned int *maxv) {
	int counter;
	for(counter=0;counter<80;counter++)	{
		if(counter < 20 || counter >= 60) { set_motors(40,-40); }
    else { set_motors(-40,40); }
		// Since our counter runs to 80, the total delay will be
		// 80*20 = 1600 ms.
		read_line_sensors(s, IR_EMITTERS_ON);
		update_bounds(s,minv,maxv);
		delay_ms(20);
	}
	set_motors(0,0);
}

// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.
void initialize() {
  // This must be called at the beginning of 3pi code, to set up the
  // sensors.  We use a value of 2000 for the timeout, which
  // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
  pololu_3pi_init(2000);
  load_custom_characters(); // load the custom characters
  print_from_program_space(robotName);
  lcd_goto_xy(0,1);
  print("Press B");
}

// This is the main function, where the code starts.  All C programs
// must have a main() function defined somewhere.
int main() {
  unsigned int sensors[5];			// global array to hold sensor values
  unsigned int minv[5], maxv[5];	// global arrays to hold min and max sensor values for calibration

  // line position relative to center
  long position = 0;
  long prev_position = 0;
  long integral = 0;
  int delta = 0;
  long offset = 0;
  long rotation = 110;
  int i;

  // set up the 3pi, and wait for B button to be pressed
  initialize();

  read_line_sensors(sensors,IR_EMITTERS_ON);
  for (i=0; i<5; i++) { minv[i] = maxv[i] = sensors[i]; }

  dance(sensors, minv, maxv); // sensor calibration

  // display calibrated sensor values as a bar graph.
  while(1) {
		// button press adjustments (RFCT)
    if (button_is_pressed(BUTTON_A)) { 
			play_from_program_space(beep_button_top);
			rotation -= 10; 
			delay_ms(100); 
		} else if (button_is_pressed(BUTTON_B)) { 
			play_from_program_space(beep_button_middle);
			run = 1-run; 
			delay_ms(200); 
		} else if (button_is_pressed(BUTTON_C)) { 
			play_from_program_space(beep_button_bottom);
			rotation += 10; 
			delay_ms(100); 
		}

    // read the line sensor values
    read_line_sensors(sensors,IR_EMITTERS_ON);
    // update minv and mav values,
    // and put normalized values in v
    update_bounds(sensors,minv,maxv);

    // compute line positon
    prev_position = position;
    position = line_position(sensors,minv,maxv);

    delta = (position - prev_position);
    integral += position; // tracks long running position offset
    offset = position/8 + delta/20; //TODO: + integral/5000;

    //if (offset > rotation) { rotation = offset; }
    //if (offset < -rotation) { offset = -rotation; }
		
    if (run == 1) {
      short leftMotor = rotation + offset;
      short rightMotor = rotation - offset;
      short motorsMax = (offset < 0) ? rightMotor : leftMotor;

      if (motorsMax > MAX_MOTOR_SPEED) {     // then scale motors down to <255
        int scaledMotorSpeed = MAX_MOTOR_SPEED/motorsMax;
        leftMotor = leftMotor * scaledMotorSpeed;
        rightMotor = rightMotor * scaledMotorSpeed;
      }

      // truncation on negatives for safety
      leftMotor = (leftMotor < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : leftMotor;
      rightMotor = (rightMotor < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : rightMotor;

      set_motors(leftMotor, rightMotor);
    }

    // display bargraph
    clear();
    print_long(position);
    lcd_goto_xy(0,1);
    // for (i=0; i<8; i++) { print_character(display_characters[i]); }
    display_bars(sensors,minv,maxv);

    delay_ms(10);
  }
}
