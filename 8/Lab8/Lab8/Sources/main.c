/*
***********************************************************************
 ECE 362 - Experiment 8 - Fall 2016
***********************************************************************
	 	   			 		  			 		  		
 Completed by: < Yifan Li >
               < 8956-L >
               < div 8 >
               < 11/3/16 >


 Academic Honesty Statement:  In entering my name above, I hereby certify
 that I am the individual who created this HC(S)12 source file and that I 
 have not copied the work of any other student (past or present) while 
 completing it. I understand that if I fail to honor this agreement, I will 
 receive a grade of ZERO and be subject to possible disciplinary action.

***********************************************************************

 The objective of this experiment is to implement a reaction time assessment
 tool that measures, with millisecond accuracy, response to a visual
 stimulus -- here, both a YELLOW LED and the message "Go Team!" displayed on 
 the LCD screen.  The TIM module will be used to generate periodic 
 interrupts every 1.000 ms, to serve as the time base for the reaction measurement.  
 The RTI module will provide a periodic interrupt at a 2.048 ms rate to serve as 
 a time base for sampling the pushbuttons and incrementing the variable "random" 
 (used to provide a random delay for starting a reaction time test). The SPI
 will be used to shift out data to an 8-bit SIPO shift register.  The shift
 register will perform the serial to parallel data conversion for the LCD.

 The following design kit resources will be used:

 - left LED (PT1): indicates test stopped (ready to start reaction time test)
 - right LED (PT0): indicates a reaction time test is in progress
 - left pushbutton (PAD7): starts reaction time test
 - right pushbutton (PAD6): stops reaction time test (turns off right LED
                    and turns left LED back on, and displays test results)
 - LCD: displays status and result messages
 - Shift Register: performs SPI -> parallel conversion for LCD interface

 When the right pushbutton is pressed, the reaction time is displayed
 (refreshed in place) on the first line of the LCD as "RT = NNN ms"
 followed by an appropriate message on the second line 
 e.g., 'Ready to start!' upon reset, 'Way to go HAH!!' if a really 
 fast reaction time is recorded, etc.). The GREEN LED should be turned on
 for a reaction time less than 250 milliseconds and the RED LED should be
 turned on for a reaction time greater than 1 second.

***********************************************************************
*/

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <mc9s12c32.h>

/* All funtions after main should be initialized here */
char inchar(void);
void outchar(char x);
void tdisp();
void shiftout(char x);
void lcdwait(void);
void send_byte(char x);
void send_i(char x);
void chgline(char x);
void print_c(char x);
void pmsglcd(char[]);

/* Variable declarations */  	   			 		  			 		       
char goteam 	= 0;  // "go team" flag (used to start reaction timer)
char leftpb	= 0;  // left pushbutton flag
char rghtpb	= 0;  // right pushbutton flag
char prevpb	= 0;  // previous pushbutton state
char runstp	= 0;  // run/stop flag
int random	= 0;  // random variable (2 bytes)
int react	= 0;  // reaction time (3 packed BCD digits)

int diff = 0;
int thres = 0;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
char prevpbleft = 0;     //previous pshbutton state
char prevpbright = 0;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/* ASCII character definitions */
#define CR 0x0D	// ASCII return character   

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
#define LINE2 0xC0	// LCD line 2 cursor position

/* LED BIT MASKS */
#define GREEN 0x20
#define RED 0x40
#define YELLOW 0x80
	 	   		
/*
***********************************************************************
 Initializations
***********************************************************************
*/

void  initializations(void) {

/* Set the PLL speed (bus clock = 24 MHz) */
  CLKSEL = CLKSEL & 0x80; // disengage PLL from system
  PLLCTL = PLLCTL | 0x40; // turn on PLL
  SYNR = 0x02;            // set PLL multiplier
  REFDV = 0;              // set PLL divider
  while (!(CRGFLG & 0x08)){  }
  CLKSEL = CLKSEL | 0x80; // engage PLL

/* Disable watchdog timer (COPCTL register) */
  COPCTL = 0x40;   //COP off, RTI and COP stopped in BDM-mode

/* Initialize asynchronous serial port (SCI) for 9600 baud, no interrupts */
  SCIBDH =  0x00; //set baud rate to 9600
  SCIBDL =  0x9C; //24,000,000 / 16 / 156 = 9600 (approx)  
  SCICR1 =  0x00; //$9C = 156
  SCICR2 =  0x0C; //initialize SCI for program-driven operation
  DDRB   =  0x10; //set PB4 for output mode
  PORTB  =  0x10; //assert DTR pin on COM port
         
         
/* Add additional port pin initializations here */
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  DDRM = 0x30; // Port M Data Direction Register: 
  DDRAD = 0;   //setting AD to input mode
  ATDDIEN = 0xC0;  //setting PAD7 and PAD6 as digital inputs
  
   ATDCTL2 = 0x80;    //ATD enabled   10000000
      ATDCTL3 = 0x10;    //2 conversions per sequence 00010000
      ATDCTL4 = 0x85;     //10000101 8bit reso, 2ATD clk periods, divide by 12 by default
      ATDCTL5 = 0x00;   
/* Initialize SPI for baud rate of 6 Mbs */

  SPICR1 = 0x50; // 01010000, spi system enabled, master mode enabled.
  SPICR2 = 0x00; // nothing to change on SPICR2
  SPIBR = 0x10; // SPPR=1, SPR=0, (1+1)*2^(0+1)=2*2=4, 24MHZ/4=6Mbps
/* Initialize digital I/O port pins */
  
  DDRT = 0xFF; // set port T to output
  PTT = 0xFF;
  PTT_PTT0 = 0;  // right LED: test is in progress
  PTT_PTT1 = 1; //left LED: reaction timer is stopped( rdy to start a new test)
  PTT_PTT5 = 0;  // GREEN LED off
  PTT_PTT6 = 0;  // RED LED off
  PTT_PTT7 = 0;  //YELLOW LED off
/* Initialize the LCD
     - pull LCDCLK high (idle)
     - pull R/W' low (write state)
     - turn on LCD (LCDON instruction)
     - enable two-line mode (TWOLINE instruction)
     - clear LCD (LCDCLR instruction)
     - wait for 2ms so that the LCD can wake up     
*/ 
   PTT_PTT4 = 1; // pull LCDCLK high
   PTT_PTT3 = 0; // pull RW' low
   send_i(LCDON); //turn on LCD
   send_i(TWOLINE); // enable 2-line mode
   send_i(LCDCLR); //clear LCD
   lcdwait(); //wait to wake up
   
/* Initialize RTI for 2.048 ms interrupt rate */	
   RTICTL = 0x1F;  //00011111 from chart
   CRGINT = 0x80;  // RTI interrupt enabled
   
/* Initialize TIM Ch 7 (TC7) for periodic interrupts every 1.000 ms
     - enable timer subsystem
     - set channel 7 for output compare
     - set appropriate pre-scale factor and enable counter reset after OC7
     - set up channel 7 to generate 1 ms interrupt rate
     - initially disable TIM Ch 7 interrupts      
*/
 
   TSCR1 = 0x80; // bit7: timer functions enabled
   TSCR2 = 0x0C; //00001100 bit3:counter reset by successful output compare 7
                 // bit2-0 :100 prescale factor :16         24MHZ/16 = 1.5M
   TIOS  = 0x80; // channel 7 for output compare, others input compare
   TC7 = 1500; // 1.5M/1000= 1500 for 1ms period
   TIE_C7I = 0; // disable TIM ch7 interrupts
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  
   
   while(!ATDSTAT0_SCF) {
     }
    thres = ATDDR0H * 2;
    chgline(LINE1);
    pmsglcd("Thresh = ");
  print_c((thres/100)+48);      // hundred  convert to ASCII
  print_c(((thres/10)%10) + 48);   //tens
  print_c((thres%10)+48);           //ones
  pmsglcd(" ms");
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
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*  If the left pushbutton ("start reaction test") flag is set, then:
     - clear left pushbutton flag
     - set the "run/stop" flag
     - display message "Ready, Set..." on the first line of the LCD
     - turn off the left LED (PT1)
     - turn on the right LED (PT0)
    Endif   
*/
    if(leftpb == 1) {    //If the left pushbutton ("start reaction test") flag is set
      leftpb = 0;        //clear left pushbutton flag
      runstp = 1;        //set the "run/stop" flag
      send_i(LCDCLR);    //Send instruction byte to LCD
      chgline(LINE1);    //Move LCD cursor to the position indicated 
      //pmsglcd("R U Ready?...");    //display message "Ready, Set..." on the first line of the LCD
        pmsglcd("Thresh = ");
       print_c((thres/100)+48);      // hundred  convert to ASCII
       print_c(((thres/10)%10) + 48);   //tens
       print_c((thres%10)+48);           //ones
       pmsglcd(" ms");
      PTT_PTT1 = 0;    //turn off the left LED (PT1)
      PTT_PTT0 = 1;    //turn on the right LED (PT0)
      
      PTT_PTT5 = 0;
      PTT_PTT6 = 0;   // clear previous LEDs 
      PTT_PTT7 = 0;
    }
/*  If the "run/stop" flag is set, then:
     - If the "goteam" flag is NOT set, then:
        + If "random" = $0000, then:
          - set the "goteam" flag
          - clear TCNT register (of TIM)
          - clear "react" variable (2 bytes)
          - enable TIM Ch7 interrupts
          - turn on YELLOW LED 
          - display message "Go Team!" on the second line of the LCD
       + Endif
     - Endif
    Endif     
*/
    if(runstp == 1) {   // If the "run/stop" flag is set
      if(goteam == 0) {  // If the "goteam" flag is NOT set
        if(random == 0) { // If "random" = $0000
          goteam = 1;   //  set the "goteam" flag
          TCNT = 0;    //   clear TCNT register (of TIM)
          react = 0;   //   clear "react" variable (2 bytes)
          TIE_C7I = 1;   //  enable TIM Ch7 interrupts
          PTT_PTT7 = 1; //  turn on YELLOW LED 
          chgline(LINE2);  
          pmsglcd("GO GO GO!");  // display message "Go Team!" on the second line of the LCD
        }
      }
    }

/*  If the right pushbutton ("stop reaction test") flag is set, then:
     - clear right pushbutton flag
     - clear the "run/stop" flag
     - clear the "goteam" flag
     - turn off yellow LED 
     - disable TIM Ch 7 interrupts
     - call "tdisp" to display reaction time message
     - turn off right LED (PT0)
     - turn on left LED (PT1)
    Endif
*/
    if(rghtpb == 1) {  //If the right pushbutton ("stop reaction test") flag is set
      rghtpb = 0;   //   clear right pushbutton flag
      runstp = 0;   //   clear the "run/stop" flag
      goteam = 0;   //   clear the "goteam" flag
      PTT_PTT7 = 0;  //   turn off yellow LED
      TIE_C7I = 0;    //   disable TIM Ch 7 interrupts
      tdisp();       //   call "tdisp" to display reaction time message
      PTT_PTT0 = 0;  //   turn off right LED (PT0)
      PTT_PTT1 = 1;  //   turn on left LED (PT1)
    }
      


/*  If "react" = 999 (the maximum 3-digit BCD value), then:
     - clear the "run/stop" flag
     - turn off yellow LED, turn on red LED
     - disable TIM Ch 7 interrupts
     - display message "Time = 999 ms" on the first line of the LCD
     - display message "Too slow!" on the second line of the LCD 
     - turn off right LED (PT0)
     - turn on left LED (PT1)
    Endif
*/  
    if(react == 999) {   // If "react" = 999
      runstp = 0;        // clear the "run/stop" flag
      PTT_PTT7 = 0;      // turn off yellow LED
      PTT_PTT6 = 1;      // turn on red LED
      TIE_C7I = 0;        // disable TIM Ch 7 interrupts
       
      chgline(LINE1);
      pmsglcd("Time = 999 ms");  // display message "Time = 999 ms" on the first line of the LCD
      chgline(LINE2);
      pmsglcd("So fuking slow!");      // display message "Too slow!" on the second line of the LCD 
      PTT_PTT0 = 0;              //turn off right LED (PT0)
      PTT_PTT1 = 1;              //turn on left LED (PT1)
      goteam = 0;
      react = 0;
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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

  Also, increments 2-byte variable "random" each time interrupt occurs
  NOTE: Will need to truncate "random" to 12-bits to get a reasonable delay 
***********************************************************************
*/

interrupt 7 void RTI_ISR(void)
{
  	// clear RTI interrupt flag
  	CRGFLG = CRGFLG | 0x80; 

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 	  if(PORTAD0_PTAD7 == 0) {     // check left pushbutton
 	    if(prevpbleft == 1) {
 	      leftpb = 1;
 	    }
 	  }
 	  prevpbleft = PORTAD0_PTAD7;
 	  
 	  if(PORTAD0_PTAD6 == 0) {     // check right pushbutton
 	    if(prevpbright == 1) {
 	      rghtpb = 1;
 	    }
 	  }
 	  prevpbright = PORTAD0_PTAD6;
 	  
 	  random++;   // increment "random" variable
 	  random &= 0x03FF; // truncate to 12 bits

 	  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

/*
*********************************************************************** 
  TIM Channel 7 interrupt service routine
  Initialized for 1.00 ms interrupt rate
  Increment (3-digit) BCD variable "react" by one
***********************************************************************
*/

interrupt 15 void TIM_ISR(void)
{
	// clear TIM CH 7 interrupt flag
 	TFLG1 = TFLG1 | 0x80; 
 	
 	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 	
 	  
 	  react++;                      //increment 'react' variable
 	
 	
 	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

}

/*
*********************************************************************** 
  tdisp: Display "RT = NNN ms" on the first line of the LCD and display 
         an appropriate message on the second line depending on the 
         speed of the reaction.  
         
         Also, this routine should set the green LED if the reaction 
         time was less than 250 ms.

         NOTE: The messages should be less than 16 characters since
               the LCD is a 2x16 character LCD.
***********************************************************************
*/
 
void tdisp()
{
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  send_i(LCDCLR);
  chgline(LINE1);
  pmsglcd("RT = ");   // Display "RT =
  print_c((react/100)+48);      // hundred  convert to ASCII
  print_c(((react/10)%10) + 48);   //tens
  print_c((react%10)+48);           //ones
  pmsglcd(" ms");
  
 /* if(react < 250) {          //if the reaction time was less than 250ms
    PTT_PTT5 = 1;           //set the green LED
    chgline(LINE2);
    pmsglcd("U R so good!");    //comments
  } else if (react < 999) {
    chgline(LINE2);
    pmsglcd("Not so good");        //comments
  }*/
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if(react < thres) {
    diff = thres - react;
    chgline(LINE2);
    print_c((diff/100)+48);      // hundred  convert to ASCII
   print_c(((diff/10)%10) + 48);   //tens
   print_c((diff%10)+48);           //ones
    pmsglcd(" ms faster");
    }
   
   else if(react > thres) {
     diff = react - thres;
     chgline(LINE2);
     print_c((diff/100)+48);      // hundred  convert to ASCII
   print_c(((diff/10)%10) + 48);   //tens
   print_c((diff%10)+48);           //ones
    pmsglcd(" ms slower");
    }
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
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int k;
  while(SPISR_SPTEF == 0) {    //read the SPTEF bit, continue if bit is 1
  }
  SPIDR = x;    //write data to SPI data register
  
  for(k=0;k<30;k++) {    // wait for 30 cycles 
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

/*
***********************************************************************
  lcdwait: Delay for approx 2 ms
***********************************************************************
*/

void lcdwait()
{
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // 24Mhz * 2/1000 = 48000 cycles
  int i;
  for (i=0; i<5000; i++) {
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
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
     //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     shiftout(x);    // shift out character
     PTT_PTT4 = 0;
     PTT_PTT4 = 1;   // pulse LCD clock line low->high->low
     PTT_PTT4 = 0;
     lcdwait();      // wait 2 ms for LCD to process data
     //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        PTT_PTT2 = 0;    //set register select to low (since sending an instruction)
        send_byte(x);    //send byte
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

/*
***********************************************************************
  chgline: Move LCD cursor to position x
  NOTE: Cursor positions are encoded in the LINE1/LINE2 variables
***********************************************************************
*/

void chgline(char x)
{
      //~~~~~~~~~~~~~~~~~~~~~~~
      send_i(CURMOV);    // move LCD cursor to position x
      send_i(x);
      //~~~~~~~~~~~~~~~~~~~~~~~~
}

/*
***********************************************************************
  print_c: Print (single) character x on LCD            
***********************************************************************
*/
 
void print_c(char x)
{
      //~~~~~~~~~~~~~~~~~~~~~~~~
      PTT_PTT2 = 1;     //set register select to high (since sending a character)
      send_byte(x);
      //~~~~~~~~~~~~~~~~~~~~~~~
}

/*
***********************************************************************
  pmsglcd: print character string str[] on LCD
***********************************************************************
*/

void pmsglcd(char str[])
{
     //~~~~~~~~~~~~~~~~~~~~~~~~~~
     int j = 0;
     while(str[j]) {
      print_c(str[j]);
      j++;
     }
     //~~~~~~~~~~~~~~~~~~~~~~~~~
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
