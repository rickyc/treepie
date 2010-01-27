/*
  3PI template code for NYU "Intro to Robotics" course.
  Yann LeCun, 02/2009.
  This program was modified from an example program from Pololu. 
 */

// The 3pi include file must be at the beginning of any program that
// uses the Pololu AVR library and 3pi.
#include <pololu/3pi.h>

// This include file allows data to be stored in program space.  The
// ATmega168 has 16k of program space compared to 1k of RAM, so large
// pieces of static data should be stored in program space.
#include <avr/pgmspace.h>

// speed of the robot
int speed = 100;
// if =1 run the robot, if =0 stop
int run=0;

// Introductory messages.  The "PROGMEM" identifier 
// causes the data to go into program space.
const char hello[] PROGMEM = " NYU/CBLL";

// Data for generating the characters used in load_custom_characters
// and display_readings.  By reading levels[] starting at various
// offsets, we can generate all of the 7 extra characters needed for a
// bargraph.  This is also stored in program space.
const char levels[] PROGMEM = {
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111
};

char display_characters[9] = { ' ', 0, 1, 2, 3, 4, 5, 6, 255 };

// This function loads custom characters into the LCD.  Up to 8
// characters can be loaded; we use them for 7 levels of a bar graph.
void load_custom_characters()
{
	lcd_load_custom_character(levels+0,0); // no offset, e.g. one bar
	lcd_load_custom_character(levels+1,1); // two bars
	lcd_load_custom_character(levels+2,2); // etc...
	lcd_load_custom_character(levels+3,3);
	lcd_load_custom_character(levels+4,4);
	lcd_load_custom_character(levels+5,5);
	lcd_load_custom_character(levels+6,6);
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
		if (s[i]<minv[i]) minv[i] = s[i];
		if (s[i]>maxv[i]) maxv[i] = s[i];
	}
}

// return line position
// YOU MUST WRITE THIS.
int line_position(unsigned int *s, unsigned int *minv, unsigned int *maxv) {
	return 0;
}

// Make a little dance: Turn left and right
void dance() {
	int counter;
	for(counter=0;counter<80;counter++)	{
		if(counter < 20 || counter >= 60)
			set_motors(40,-40);
		else
			set_motors(-40,40);
		// Since our counter runs to 80, the total delay will be
		// 80*20 = 1600 ms.
		delay_ms(20);
	}
	set_motors(0,0);
}

// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.
void initialize()
{
  // This must be called at the beginning of 3pi code, to set up the
  // sensors.  We use a value of 2000 for the timeout, which
  // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
  pololu_3pi_init(2000);
  load_custom_characters(); // load the custom characters
  // display message
  print_from_program_space(hello);
  lcd_goto_xy(0,1);
  print("Press B");

}

// This is the main function, where the code starts.  All C programs
// must have a main() function defined somewhere.
int main()
{
  // global array to hold sensor values
  unsigned int sensors[5]; 
  // global arrays to hold min and max sensor values
  // for calibration
  unsigned int minv[5], maxv[5]; 
  // line position relative to center
  int position = 0;
  
  int i;

  // set up the 3pi, and wait for B button to be pressed
  initialize();
  
  read_line_sensors(sensors,IR_EMITTERS_ON);
  for (i=0; i<5; i++) { minv[i] = maxv[i] = sensors[i]; }

  // Display calibrated sensor values as a bar graph.
  while(1) {
    if (button_is_pressed(BUTTON_B)) { run = 1-run; delay(200); }
    if (button_is_pressed(BUTTON_A)) { speed -= 10; delay(100); }
    if (button_is_pressed(BUTTON_C)) { speed += 10; delay(100); }
    
    // Read the line sensor values
    read_line_sensors(sensors,IR_EMITTERS_ON);
    // update minv and mav values,
    // and put normalized values in v
    update_bounds(sensors,minv,maxv);

    // compute line positon
    position = line_position(sensors,minv,maxv);

    // display bargraph
    clear();
    print_long(position);
    lcd_goto_xy(0,1);
    // for (i=0; i<8; i++) { print_character(display_characters[i]); }
    display_bars(sensors,minv,maxv);
    
    delay_ms(10);
  }
}
