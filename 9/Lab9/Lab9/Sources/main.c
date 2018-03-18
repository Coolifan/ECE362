/*
***********************************************************************
 ECE 362 - Experiment 9 - Fall 2016
***********************************************************************
	 	   			 		  			 		  		
 Completed by: < Yifan Li >
               < 8956-L >
               < div 8 >
               < 11/9/16 >


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

/*  Variable declarations */ 	   			 		  			 		       
char leftpb	= 0;  // left pushbutton flag
char rghtpb	= 0;  // right pushbutton flag
char prevpb	= 0;  // previous pushbutton state

char prevleftpb = 0;
char prevrghtpb = 0;
int leftpb_i = 0; //counter for leftpb
int rghtpb_i = 0; //counter for rghtpb

int bonus = 0;
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
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

/* Initialize the SPI to 6 Mbs */
  SPICR1 = 0x50; //01010000, spi system enabled, master mode enabled.
  SPICR2 = 0;	// nothing to change on SPICR2
  SPIBR = 0x01; // SPR=1,SPPR=0 (0+1)*2^(1+1)=4, 24Mhz/4=6Mbps
	 	   			 		  			 		  		
/* Initialize digital I/O port pins */
  DDRT = 0xff; //set port T to output
  ATDDIEN = 0xC0; // 11000000 setting PAD7 and PAD6 as digital inputs
  DDRM = 0xff; 
  
  ATDCTL2 = 0x80;    //ATD enabled   10000000
  ATDCTL3 = 0x10;    //2 conversions per sequence 00010000
  ATDCTL4 = 0x85;     //10000101 8bit resolution, 2ATD clk periods, divide by 12 by default
  ATDCTL5 = 0x10; //00010000 sample across multiple channels

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
  TIE_C7I = 1; // enable TIM ch7 interrupts

  while(!ATDSTAT0_SCF) {
  }
  // Initialize PWM ch0 from demo code
  MODRR = 0x01; // PT0 used as PWM Ch 0 output
  PWME = 0x01;	//enable PWM Ch 0
  PWMPOL = 0x01; //set active high polarity
  PWMCTL = 0; //no concatenate (8-bit)
  PWMCAE = 0; //left-aligned output mode
  PWMPER0 = 0xff; //set maximum 8-bit period
  PWMDTY0 = 0; //initially clear DUTY register
  PWMCLK = 0; //select Clock A for Ch 0
  PWMPRCLK = 0x00; //set Clock A = 12 MHz (prescaler = 2) rate
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  
	      
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
	if(leftpb == 1) {
		leftpb = 0;
		leftpb_i++;
		leftpb_i %= 3;
		if(leftpb_i == 1) {
			TC7 = 300; //5000hz
		}
		else if(leftpb_i == 2) {
			TC7 = 150; //10000hz
		}
		else if(leftpb_i == 0) {
			TC7 = 75; //20000hz
		}

	} 

	if(rghtpb == 1) {
		rghtpb = 0;
		rghtpb_i++;
		rghtpb_i %= 3;
		if(rghtpb_i == 1) {
			PWMPRCLK = 2; //prescale A clk select, 
		}
		else if(rghtpb_i == 2) {
			PWMPRCLK = 1;
		}
		else if(rghtpb_i == 0) {
			PWMPRCLK = 0; 
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

  	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 	  if(PORTAD0_PTAD7 == 0) {     // check left pushbutton
 	    if(prevleftpb == 1) {
 	      leftpb = 1;
 	    }
 	  }
 	  prevleftpb = PORTAD0_PTAD7;
 	  
 	  if(PORTAD0_PTAD6 == 0) {     // check right pushbutton
 	    if(prevrghtpb == 1) {
 	      rghtpb = 1;
 	    }
 	  }
 	  prevrghtpb = PORTAD0_PTAD6;
 	  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

 	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 	ATDCTL5 = 0x10;     //sample across multiple channels
 	while(!ATDSTAT0_SCF) {

 	}
 //	PWMDTY0 = (ATDDR0H * ATDDR1H) / 255;
 
 
 	//bonus~~~~~~~~~~~~~~~~~~~~~~~
 	TC7 = ATDDR0H;
 	if(bonus>128) {
 	  bonus = 0;
 	}
 	PWMDTY0 = (bonus++ * ATDDR1H) / 255;
 	//bonus over~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 	

 	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
 	if(leftpb_i == 1) {
 		pmsglcd("ISF: 5000  Hz");
 	}
 	else if(leftpb_i == 2) {
 		pmsglcd("ISF: 10000 Hz");
 	}
 	else if(leftpb_i == 0) {
 		pmsglcd("ISF: 20000 Hz");
 	}

 	chgline(LINE2);
 	if(rghtpb_i == 1) {
 		pmsglcd("OSF: 23529 Hz");
 	}
 	else if(rghtpb_i == 2) {
 		pmsglcd("OSF: 47059 Hz");
 	}
 	else if(rghtpb_i == 0) {
 		pmsglcd("OSF: 94118 Hz");
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
