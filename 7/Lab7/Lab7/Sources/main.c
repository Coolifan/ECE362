// ***********************************************************************
//  ECE 362 - Experiment 7 - Fall 2016
//
// Dual-channel LED bar graph display                    
// ***********************************************************************
//	 	   			 		  			 		  		
// Completed by: < Yifan Li >
//               < 8956L >
//               < 8 >
//               < 10/26/16 >
//
//
// Academic Honesty Statement:  In entering my name above, I hereby certify
// that I am the individual who created this HC(S)12 source file and that I 
// have not copied the work of any other student (past or present) while 
// completing it. I understand that if I fail to honor this agreement, I will 
// receive a grade of ZERO and be subject to possible disciplinary action.
//
// ***********************************************************************

#include <hidef.h>           /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <mc9s12c32.h>

// All funtions after main should be initialized here

// Note: inchar and outchar can be used for debugging purposes

char inchar(void);
void outchar(char x);
			 		  		
//  Variable declarations  	   			 		  			 		       
int tenthsec = 0;  // one-tenth second flag
int leftpb = 0;    // left pushbutton flag
int rghtpb = 0;    // right pushbutton flag
int runstp = 0;    // run/stop flag                         
int rticnt = 0;    // RTICNT (variable)
int prevpb = 0;    // previous state of pushbuttons (variable)

//-----------------------//
int prevpbleft = 0;    //previous state of left pb
int prevpbright = 0;  //previous state of rght pb
int i=0;            //loop counter
int channel0=0;      //OUTPUT CH0 
int channel1=0;      //OUTPUT CH1


//----------------------//
	 	   		
// Initializations
 
void  initializations(void) {

// Set the PLL speed (bus clock = 24 MHz)

  		CLKSEL = CLKSEL & 0x80; // disengage PLL from system
  		PLLCTL = PLLCTL | 0x40; // turn on PLL
  		SYNR = 0x02;            // set PLL multiplier
  		REFDV = 0;              // set PLL divider
  		while (!(CRGFLG & 0x08)){  }
  		CLKSEL = CLKSEL | 0x80; // engage PLL
  
// Disable watchdog timer (COPCTL register)

      COPCTL = 0x40;    //COP off - RTI and COP stopped in BDM-mode

// Initialize asynchronous serial port (SCI) for 9600 baud, no interrupts

      SCIBDH =  0x00; //set baud rate to 9600
      SCIBDL =  0x9C; //24,000,000 / 16 / 156 = 9600 (approx)  
      SCICR1 =  0x00; //$9C = 156
      SCICR2 =  0x0C; //initialize SCI for program-driven operation
         
//  Initialize Port AD pins 7 and 6 for use as digital inputs

	    DDRAD = 0; 		//program port AD for input mode
      ATDDIEN = 0xC0; //program PAD7 and PAD6 pins as digital inputs
         
//  Add additional port pin initializations here  (e.g., Other DDRs, Ports) 

    //--------------------------//
      DDRT = 0xFF; //Port T Data Direction Register, enable all inputs
    //---------------------------//
    
//  Define bar graph segment thresholds (THRESH1..THRESH5)
//  NOTE: These are binary fractions
      #define THRESH1 0x02 
      #define THRESH2 0x03 
      #define THRESH3 0x05  
      #define THRESH4 0x07  
      #define THRESH5 0x0A 

//  Add RTI/interrupt initializations here
    
    //---------------------------//
      rticnt = 0;
      ATDCTL2 = 0x80;    //ATD enabled   10000000
      ATDCTL3 = 0x10;    //2 conversions per sequence 00010000
      ATDCTL4 = 0x85;     //10000101 8bit reso, 2ATD clk periods, divide by 12 by default
      ATDCTL5 = 0x10;     //00010000 sample across multiple channels, start with channel 0
    
      RTICTL = 0x70;       // 8.192ms RTI interrupt rate   1110000
      CRGINT = 0x80;       // enable RTI interrupts
    //----------------------------//

}
	 		  			 		  		
 
// Main (non-terminating loop)
 
void main(void) {
	initializations(); 		  			 		  		
	EnableInterrupts;


  for(;;) {


// Main program loop (state machine)
// Start of main program-driven polling loop

	 	   			 		  			 		  		
//  If the "tenth second" flag is set, then
//    - clear the "tenth second" flag
//    - if "run/stop" flag is set, then
//       - initiate ATD coversion sequence
//       - apply thresholds to converted values
//       - determine 5-bit bar graph bit settings for each input channel
//       - transmit 10-bit data to external shift register
//    - endif
//  Endif

    //--------------------------------------//
	 	if (tenthsec == 1) {
	 	  tenthsec = 0; //clear the tenth sec flag
	 	  if (runstp == 1) {
	 	    ATDCTL5 = 0x10; //initialize ATD conversion sequence, sample across multiple channels,select input channel 0
	 	    channel0 =0; // initial low voltage  , all LED off
	 	    channel1 =0; // initial low voltage  . all LED off
	 	    
	 	    while (ATDSTAT0_SCF == 0) {
	 	    }
	 	    
	 	    //channel 0 threshold
	 	    if ((ATDDR0H >= THRESH1) && (ATDDR0H < THRESH2)) {     //ATD Conversion Result Register 0 High
	 	      channel0=1;                                          //green 
	 	    }
	 	    if ((THRESH2 <= ATDDR0H) && (ATDDR0H < THRESH3)) {     // green+yellow
	 	      channel0=2;
	 	    }
	 	    if ((THRESH3 <= ATDDR0H) && (ATDDR0H < THRESH4)) {     //GYY
	 	      channel0=3;
	 	    }
	 	    if ((THRESH4 <= ATDDR0H) && (ATDDR0H < THRESH5)) {     //GYYR
	 	      channel0=4;
	 	    }
	 	    if (ATDDR0H >= THRESH5) {                              //GYYRR
	 	      channel0=5;
	 	    }
	 	    //channel 1 threshold
	 	    if ((ATDDR1H >= THRESH1) && (ATDDR1H < THRESH2)) {     //ATD Conversion Result Register 1 High;
	 	      channel1=1;
	 	    }
	 	    if ((THRESH2 <= ATDDR1H) && (ATDDR1H < THRESH3)) {     
	 	      channel1=2;
	 	    }
	 	    if ((THRESH3 <= ATDDR1H) && (ATDDR1H < THRESH4)) {     
	 	      channel1=3;
	 	    }
	 	    if ((THRESH4 <= ATDDR1H) && (ATDDR1H < THRESH5)) {     
	 	      channel1=4;
	 	    }
	 	    if (ATDDR1H >= THRESH5) {     
	 	      channel1=5;
	 	    }
	 	    
	 	    //       - determine 5-bit bar graph bit settings for each input channel
        //       - transmit 10-bit data to external shift register
        for (i=0; i<10; i++) {
          PTT_PTT4 =0; //PT4=Clock
          
          
          
          if (i<5) {   //sample and transfer, channel0
            if (channel0 >0) {
              PTT_PTT3 = 1; //PT3 = input data
              channel0-=1;
            } else {
              PTT_PTT3 = 0;
            } 
            
          } else {            //channel 1
            
            if (channel1 >0) {
              PTT_PTT3 = 1;
              channel1-=1;
            } else {
              PTT_PTT3 = 0;
            }
          }
          PTT_PTT4=1;   //clk
        }
	 	  }
	 	}
            
              
             
	 	    
	 	//--------------------------------------//
	 	   			 		  			 		  		
//  If the left pushbutton ("stop BGD") flag is set, then:
//    - clear the left pushbutton flag
//    - clear the "run/stop" flag (and "freeze" BGD)
//    - turn on left LED/turn off right LED (on docking module)
//  Endif
   	// ---------------------------------//
   	if (leftpb ==1){
   	  leftpb = 0; //clear left pushbutton flag
   	  runstp = 0; //clear the runstop flag
   	  PTT_PTT1 = 1; //turn on left LED on docking module
   	  PTT_PTT0 = 0; //TURN off right LED
   	}
   	//----------------------------------//  

//  If the right pushbutton ("start BGD") flag is set, then
//    - clear the right pushbutton flag
//    - set the "run/stop" flag (enable BGD updates)
//    - turn off left LED/turn on right LED (on docking module)
//  Endif
	 	//-------------------------------------//
	 	if (rghtpb ==1) {
	 	  rghtpb = 0; //clear right pshbt flag
	 	  runstp = 1; // set the runstop flag
	 	  PTT_PTT1 = 0; //turn off left LED
	 	  PTT_PTT0 = 1; //turn on right LED
	 	}
	 	//-------------------------------------//

  }/* loop forever */
  
}/* make sure that you never leave main */



// ***********************************************************************                       
// RTI interrupt service routine: rti_isr
//
//  Initialized for 5-10 ms (approx.) interrupt rate - note: you need to
//    add code above to do this
//
//  Samples state of pushbuttons (PAD7 = left, PAD6 = right)
//
//  If change in state from "high" to "low" detected, set pushbutton flag
//     leftpb (for PAD7 H -> L), rghtpb (for PAD6 H -> L)
//     Recall that pushbuttons are momentary contact closures to ground
//
//  Also, keeps track of when one-tenth of a second's worth of RTI interrupts
//     accumulate, and sets the "tenth second" flag         	   			 		  			 		  		
 
interrupt 7 void RTI_ISR( void)
{
 // set CRGFLG bit to clear RTI device flag
  	CRGFLG = CRGFLG | 0x80; 
 
 //-----------------------------------------------//
 	  if(PTAD_PTAD7 == 0) {   
 	    if(prevpbleft == 1) {
 	      leftpb = 1;
 	    }
 	  }      //left pshbt pressed, H->L       
 	         //set previous left pb and left pb
 	  prevpbleft = PTAD_PTAD7;
 	  
 	  
 	  
 	  if(PORTAD0_PTAD6 == 0) { // check right pushbutton
      if(prevpbright == 1) {
        rghtpb = 1;
      }
    }
    prevpbright = PTAD_PTAD6; //lecture notes pp.256
 	  
    if (rticnt == 12) {    
      tenthsec = 1;       //0.1s, set the tenthsec flag
      rticnt = 0;
    } else {
      rticnt++;
    }
}


// ***********************************************************************
// Character I/O Library Routines for 9S12C32 (for debugging only)
// ***********************************************************************
// Name:         inchar
// Description:  inputs ASCII character from SCI serial port and returns it
// ***********************************************************************
char  inchar(void) {
  /* receives character from the terminal channel */
        while (!(SCISR1 & 0x20)); /* wait for RDR input */
    return SCIDRL;
 
}

// ***********************************************************************
// Name:         outchar
// Description:  outputs ASCII character passed in outchar()
//                  to the SCI serial port
// ***********************************************************************/
void outchar(char ch) {
  /* transmits a character to the terminal channel */
    while (!(SCISR1 & 0x80));  /* wait for TDR empty */
    SCIDRL = ch;
}

