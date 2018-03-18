/*
***********************************************************************
 ECE 362 - Experiment 9 - Spring 2015
***********************************************************************
 Completed by: < Young Jin Jung >
               < 0500-J >
               < 3 >
 Academic Honesty Statement:  In entering my name above, I hereby certify
 that I am the individual who created this HC(S)12 source file and that I
 have not copied the work of any other student (past or present) while
 completing it. I understand that if I fail to honor this agreement, I will
 receive a grade of ZERO and be subject to possible disciplinary action.
***********************************************************************
 The objective of this experiment is to implement an analog signal sampling
 and reconstruction application that allows the user to efficiently cycle
 through different input and output sampling frequencies.
 The following design kit resources will be used:
 - left pushbutton (PAD7): cycles through input sampling frequency choices
                           (5000 Hz, 10,000 Hz, and 20,000 Hz)
 - right pushbutton (PAD6): cycles through output sampling frequency choices
                           (23,529 Hz, 47,059 Hz, and 94,118 Hz)
 - LCD: displays current values of input and output sampling frequencies
 - Shift Register: performs SPI -> parallel conversion for LCD interface
***********************************************************************
*/

/*
	Final version 3/30/2016
*/

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <mc9s12c32.h>

/* All funtions after main should be initialized here */
char inchar(void);
void outchar(char);
void fdisp(void);
void shiftout(char);
void lcdwait(void);
void send_byte(char);
void send_i(char);
void chgline(char);
void print_c(char);
void pmsglcd(char[]);
void print(unsigned long);

/*  Variable declarations */
char goteam = 0;  // "go team" flag (used to start reaction timer)
char leftpb	= 0;  // left pushbutton flag
char rghtpb	= 0;  // right pushbutton flag
char prevpb	= 0;  // previous pushbutton state

char prevpbl = 0;  // previous pushbutton state
char prevpbr = 0;
char runstp	= 0;  // run/stop flag
int rticnt = 0;
int frequency_i = 5000;
unsigned long frequency_o = 23529;
int temp = 0;
int i = 0;
int in = 0;
int out = 0;


/* LCD COMMUNICATION BIT MASKS */
#define RS 0x04		// RS pin mask (PTT[2])
#define RW 0x08		// R/W pin mask (PTT[3])
#define LCDCLK 0x10	// LCD EN/CLK pin mask (PTT[4])

/* LCD INSTRUCTION CHARACTERS */
#define LCDON 0x0F	// LCD initialization command
#define LCDCLR 0x01	// LCD clear display command
#define TWOLINE 0x38	// LCD 2-line enable command
#define CURMOV 0xFE	// LCD cursor move instruction
#define LINE1 0x80	// LCD line 1 cursor position
#define LINE2 0xC0	// LCD line 2 cursor position.
#define RGHTLED PTT_PTT0
#define LEFTLED PTT_PTT1

/*
***********************************************************************
 Initializations
***********************************************************************
*/

void  initializations(void) {

/* Set the PLL speed (bus clock = 24 MHz) */
  CLKSEL = CLKSEL & 0x80; //; disengage PLL from system
  PLLCTL = PLLCTL | 0x40; //; turn on PLL
  SYNR = 0x02;            //; set PLL multiplier
  REFDV = 0;              //; set PLL divider
  while (!(CRGFLG & 0x08)){  }
  CLKSEL = CLKSEL | 0x80; //; engage PLL


/* Disable watchdog timer (COPCTL register) */
  COPCTL = 0x40   ; //COP off; RTI and COP stopped in BDM-mode

/* Initialize asynchronous serial port (SCI) for 9600 baud, no interrupts */
  SCIBDH =  0x00; //set baud rate to 9600
  SCIBDL =  0x9C; //24,000,000 / 16 / 156 = 9600 (approx)
  SCICR1 =  0x00; //$9C = 156
  SCICR2 =  0x0C; //initialize SCI for program-driven operation
  DDRB   =  0x10; //set PB4 for output mode
  PORTB  =  0x10; //assert DTR pin on COM port


/* Add additional port pin initializations here */

  ATDDIEN = 0xc0;   
  prevpbl = 0;
	prevpbr = 0;



/* Initialize the SPI to 6 Mbs */

  DDRM = 0xff;        // Port M init
  SPICR1 = 0x50;
  SPICR2 = 0x00;
  SPIBR = 0x01;



/* Initialize digital I/O port pins */

  DDRT = 0xff;


/* Initialize the LCD
     - pull LCDCLK high (idle)
     - pull R/W' low (write state)
     - turn on LCD (LCDON instruction)
     - enable two-line mode (TWOLINE instruction)
     - clear LCD (LCDCLR instruction)
     - wait for 2ms so that the LCD can wake up
*/
  PTT_PTT4 = 1;		  // LCD Clock pull high
  PTT_PTT3 = 0;		  // R/W' pull low
  send_i(LCDON);	  // turn on LCD
  send_i(TWOLINE);	// enable two line mode
  send_i(LCDCLR);	  // clear out LCD
  lcdwait();  		  // wait

/* Initialize RTI for 2.048 ms interrupt rate */

  RTICTL = 0x1f;     // from the chart
  CRGINT = CRGINT | 0x80;

/* Initialize TIM Ch 7 (TC7) for periodic interrupts every 1.000 ms
     - enable timer subsystem
     - set channel 7 for output compare
     - set appropriate pre-scale factor and enable counter reset after OC7
     - set up channel 7 to generate 1 ms interrupt rate
     - initially disable TIM Ch 7 interrupts
*/

  TSCR1 = 0x80;	    		// Timer subsystem enable
  TSCR2 = 0x0c;	        // Timer counter reset by successful output compare 7
					          	  // Timer prescale of 16

  TIOS = 0x80;	    		// Set channel 7 for output compare, other remain input compare
  TIE = 0x80;       		// enable ch7 interrupts
  TC7 = 300;			    
  
  
  
  // ATD Init
  // From Prev Lab
	ATDCTL2 = 0x80;
	ATDCTL3 = 0b00010000;
	ATDCTL4 = 0b10000101;

	ATDCTL5 = 0x10;

	while(!ATDSTAT0_SCF) {}
	
  // From Demo File
  MODRR = 0x01;           // PT0 used as PWM Ch0 output
  PWME  = 0x01;           // enable PWM Ch 0
  PWMPOL  = 0x01;         // set active high polarity
  PWMCTL  = 0x00;         // no concatenate (8bit)
  PWMCAE  = 0x00;         // left-aligned output mode
  PWMPER0 = 0xff;         // set maximum 8-bit period
  PWMDTY0 = 0x00;         // initially clear Duty Register
  PWMCLK  = 0x00;         // select Clock A for Ch 0
  PWMPRCLK  = 0x02;       // set Clock A = 12 MHz (prescaler 2) rate
}

/*
***********************************************************************
 Main
***********************************************************************
*/

void main(void) {
  DisableInterrupts;
	initializations();
	EnableInterrupts;



  for(;;) {

    /* write your code here */
    if(leftpb)
    {
      leftpb = 0;
      in++;
      in %= 3;
      if(in == 0)
      {
        TC7 = 300;
        frequency_i = 5000; 
      }
      if(in == 1)
      {
        TC7 = 150;
        frequency_i = 10000;
      }
      if(in == 2)
      {
        TC7 = 75;
        frequency_i = 20000;
      }

      
    }


    if(rghtpb)
    {
      rghtpb = 0;
      out++;
      out %= 3;
      
      if(out == 0)
      {
        PWMPRCLK = 2;
        frequency_o = 23529;
      }
      if(out == 1)
      {
        PWMPRCLK = 1;
        frequency_o = 47059; 
      }
      if(out == 2)
      {
        PWMPRCLK = 0;
        frequency_o = 94118; 
      }
      
    }
    fdisp();









  } /* loop forever */

}  /* do not leave main */




/*
***********************************************************************
 RTI interrupt service routine: RTI_ISR
  Initialized for 2.048 ms interrupt rate
  Samples state of pushbuttons (PAD7 = left, PAD6 = right)
  If change in state from "high" to "low" detected, set pushbutton flag
     leftpb (for PAD7 H -> L), rghtpb (for PAD6 H -> L)
     Recall that pushbuttons are momentary contact closures to ground
***********************************************************************
*/

interrupt 7 void RTI_ISR(void)
{
  	// clear RTI interrupt flag
  	CRGFLG = CRGFLG | 0x80;

    /* PAD = 0 PREV = 0 -> Button is pressed currently, but it wasn't pressed previously
  	   PAD = 0 PREV = 1 -> Button is pressed currently, and it was pressed previously
  	   PAD = 1 PREV = 0 -> Button is not pressed currently, and it wasn't pressed previously
  	   PAD = 1 PREV = 1 -> Button is not pressed currently, but it was pressed previously
  	*/

	  if((PORTAD0_PTAD7 + prevpb) == 0)
	  {
		  prevpbl = 1;
		  leftpb = 1;
	  } else if (PORTAD0_PTAD7)
	  {
		  prevpbl = 0;
	  }

	  if((PORTAD0_PTAD6 + prevpb) == 0)
	  {
		  prevpbr = 1;
		  rghtpb = 1;
	  } else if (PORTAD0_PTAD6)
	  {
		  prevpbr = 0;
	  }

}

/*
***********************************************************************
  TIM interrupt service routine
  used to initiate ATD samples (on Ch 0 and Ch 1)
***********************************************************************
*/

interrupt 15 void TIM_ISR(void)
{

        // clear TIM CH 7 interrupt flag
 	TFLG1 = TFLG1 | 0x80;

 	ATDCTL5 = 0x10;
  while(ATDSTAT0);

 	PWMDTY0 = (ATDDR0H * ATDDR1H) / 256;

}

/*
***********************************************************************
  fdisp: Display "ISF = NNNNN Hz" on the first line of the LCD and display
         and "OSF = MMMMM Hz" on the second line of the LCD (where NNNNN and
         MMMMM are the input and output sampling frequencies, respectively
***********************************************************************
*/

void fdisp()
{
  chgline(LINE1);
  pmsglcd("ISF: ");
  print(frequency_i);
  pmsglcd(" Hz");
  
  chgline(LINE2);
  pmsglcd("OSF: ");
  print(frequency_o);
  pmsglcd(" Hz");

}

void print(unsigned long num)
{
  print_c((num/10000) + 0x30);			      // 10000s
	print_c((num/1000) % 10 + 0x30);		    // 1000s
	print_c((num/100) % 10 + 0x30);         // 100s
	print_c((num/10) % 10 + 0x30);			    // 10s
	print_c((num%10) + 0x30);		            // 1s
}

/*
***********************************************************************
  shiftout: Transmits the character x to external shift
            register using the SPI.  It should shift MSB first.
            MISO = PM[4]
            SCK  = PM[5]
***********************************************************************
*/

void shiftout(char x)

{

  // read the SPTEF bit, continue if bit is 1
  // write data to SPI data register
  // wait for 30 cycles for SPI data to shift out

  while(!SPISR_SPTEF);
  SPIDR = x;
  lcdwait();

}

/*
***********************************************************************
  lcdwait: Delay for approx 2 ms
***********************************************************************
*/

void lcdwait()
{
  int a = 2;
  int b = 3910;
  while (a)
  {
    while(b)
    {
      b--;
    }

    a--;
  }

}

/*
***********************************************************************
  send_byte: writes character x to the LCD
***********************************************************************
*/

void send_byte(char x)
{
     // shift out character
     // pulse LCD clock line low->high->low
     // wait 2 ms for LCD to process data
   shiftout(x);
	 PTT_PTT4 = 0;
	 PTT_PTT4 = 1;
	 PTT_PTT4 = 0;
	 lcdwait();
}

/*
***********************************************************************
  send_i: Sends instruction byte x to LCD
***********************************************************************
*/

void send_i(char x)
{
        // set the register select line low (instruction data)
        // send byte
  PTT_PTT2 = 0;
	send_byte(x);
}

/*
***********************************************************************
  chgline: Move LCD cursor to position x
  NOTE: Cursor positions are encoded in the LINE1/LINE2 variables
***********************************************************************
*/

void chgline(char x)
{
  send_i(CURMOV);
	send_i(x);
}

/*
***********************************************************************
  print_c: Print (single) character x on LCD
***********************************************************************
*/

void print_c(char x)
{

  PTT_PTT2 = 1;
	send_byte(x);
}

/*
***********************************************************************
  pmsglcd: print character string str[] on LCD
***********************************************************************
*/

void pmsglcd(char str[])
{
  int i = 0;
	while(str[i] != '\0')
	{
		print_c(str[i]);
		i++;
	}

}


/*
***********************************************************************
 Character I/O Library Routines for 9S12C32 (for debugging only)
***********************************************************************
 Name:         inchar
 Description:  inputs ASCII character from SCI serial port and returns it
 Example:      char ch1 = inchar();
***********************************************************************
*/

char inchar(void) {
  /* receives character from the terminal channel */
        while (!(SCISR1 & 0x20)); /* wait for input */
    return SCIDRL;
}

/*
***********************************************************************
 Name:         outchar
 Description:  outputs ASCII character x to SCI serial port
 Example:      outchar('x');
***********************************************************************
*/

void outchar(char x) {
  /* sends a character to the terminal channel */
    while (!(SCISR1 & 0x80));  /* wait for output buffer empty */
    SCIDRL = x;
}