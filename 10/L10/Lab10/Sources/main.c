/*
************************************************************************
 ECE 362 - Experiment 10
***********************************************************************
	 	   			 		  			 		  		
 Completed by: < Young Jin Jung >
               < 0500-J >
               < 3 >
               < 4/6/2016 >
 Academic Honesty Statement:  In entering my name above, I hereby certify
 that I am the individual who created this HC(S)12 source file and that I 
 have not copied the work of any other student (past or present) while 
 completing it. I understand that if I fail to honor this agreement, I will 
 receive a grade of ZERO and be subject to possible disciplinary action.
***********************************************************************
 The objective of this lab is to control the speed of a small D.C.
 motor using pulse width modulation.  The PWM duty cycle will be
 a function of an input analog D.C. voltage (range: 0 to 5 volts).
 The speed of the motor will be determined by the number of pulses
 detected by the pulse accumulator (estimated over a 1.0 second
 integration period); an updated estimate of the motor RPM will
 be displayed once every second.  The timer (TIM) will be used
 to drive the ATD sample/PWM update rate (every one-tenth second)
 and the display update rate (every second).  The RPM estimate  
 will be based on the number of pulses accumulated from the motor's 
 64-hole chopper over a 1.0 second integration period (divided by 28,
 to estimate the gear head output shaft speed).  The real time   
 interrupt (RTI) will be used to sample the pushbutton state.
 In addition, message strings will be continously output via the
 SCI to an emulated terminal (TeraTerm) once every second (done
 via a buffered, interrupt-driven device driver).
 The docking module pushbuttons and LEDs will be used as follows:
 - left pushbutton (PAD7): stop motor (if running)
 - right pushbutton (PAD6): start motor (if stopped)
 - left LED (PT1): on if motor stopped
 - right LED (PT0): on if motor running
 The RPM value will be displayed on the first line of the LCD. 
 A bar graph showing the percent-of-max will be displayed on 
 the second line of the LCD.
 For bonus credit, the time-of-day clock created for Experiment 4
 can be integrated into the system. The time will be displayed on
 the first line of the LCD in an alternating fashion with the RPM
 (display mode will toggle once each second). After reset, the
 current time can be initialized 
 All parts needed to build (your part of) the motor interface circuit
 are included in the DK-3 - see schematic in lab document.
***********************************************************************
*/


/* final */

#include <hidef.h>       
#include "derivative.h"	 
#include <mc9s12c32.h>

/* All functions after main should be initialized here */
void outchar(char x);	// for debugging use only
char inchar(void);	// for bonus option (terminal input for setting clock)
void rdisp(void);		// RPM display
void bco(char x);		// SCI buffered character output
void shiftout(char);	// LCD drivers (written previously)
void lcdwait(void);
void send_byte(char);
void send_i(char);
void chgline(char);
void print_c(char);
void pmsglcd(char[]);
void print_rpm(int num);
void print_bar(int bar);
void print_percentage(int percent, int space);
void print_time();
void pmsgterm(char str[]);
void pmsgsci(char str[]);



/* Variable declarations */ 	   			 		  			 		       
char leftpb	= 0;	// left pushbutton flag
char rghtpb	= 0;	// right pushbutton flag
char prevpb	= 0;	// previous pushbutton state
char prevpbr  = 0;
char prevpbl  = 0;
char runstp	= 0;	// motor run/stop flag
char onesec = 0;	// one second flag
char tenths	= 0;	// tenth of a second flag
char tin	= 0;	// SCI transmit display buffer IN pointer
char tout	= 0;	// SCI transmit display buffer OUT pointer
int pulscnt 	= 0;	// pulse count (read from PA every second)

int maxrpm = 170;

int tencnt  = 0;
int onecnt  = 0;
int per_ratio = 0;
int percent = 0;
int space = 0;

/* BONUS */
char set = 0;
char ampm = 0;
int tmp = 0;
int tmp_1 = 0;
int hrs_t = 0;
int hrs_o = 0;
int min_t = 0;
int min_o = 0;
int sec_t = 0;
int sec_o = 0;

int hrs = 0;
int min = 0;
int secns = 0;
int leftpbT = 0;


#define TSIZE 81	// transmit buffer size (80 characters)
char tbuf[TSIZE];	// SCI transmit display buffer

#define CR 0x0D		// ASCII return 
#define LF 0x0A		// ASCII new line 

/* LCD COMMUNICATION BIT MASKS (note - different than previous labs) */
#define RS 0x10		// RS pin mask (PTT[4])
#define RW 0x20		// R/W pin mask (PTT[5])
#define LCDCLK 0x40	// LCD EN/CLK pin mask (PTT[6])

/* LCD INSTRUCTION CHARACTERS */
#define LCDON 0x0F	// LCD initialization command
#define LCDCLR 0x01	// LCD clear display command
#define TWOLINE 0x38	// LCD 2-line enable command
#define CURMOV 0xFE	// LCD cursor move instruction
#define LINE1 0x80	// LCD line 1 cursor position
#define LINE2 0xC0	// LCD line 2 cursor position


#define RGHTLED PTT_PTT0
#define LEFTLED PTT_PTT1

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
  COPCTL = 0x40   ; // COP off; RTI and COP stopped in BDM-mode

/* Initialize asynchronous serial port (SCI) for 9600 baud, interrupts off initially */
  SCIBDH =  0x00; //set baud rate to 9600
  SCIBDL =  0x9C; //24,000,000 / 16 / 156 = 9600 (approx)  
  SCICR1 =  0x00; //$9C = 156
  SCICR2 =  0x0C; //initialize SCI for program-driven operation
  DDRB   =  0x10; //set PB4 for output mode
  PORTB  =  0x10; //assert DTR pin on COM port
         
         
/* 
   Initialize TIM Ch 7 (TC7) for periodic interrupts every 10.0 ms  
    - Enable timer subsystem                         
    - Set channel 7 for output compare
    - Set appropriate pre-scale factor and enable counter reset after OC7
    - Set up channel 7 to generate 10 ms interrupt rate
    - Initially disable TIM Ch 7 interrupts	 	   			 		  			 		  		
*/
    TSCR1 = 0x80;
    TSCR2 = 0x0c;
    TIOS = 0x80;
    TIE = 0x80;
    TC7 = 15000;
    	 	   			 		  			 		  		



/*
 Initialize the PWM unit to produce a signal with the following
 characteristics on PWM output channel 3:
   - sampling frequency of approximately 100 Hz
   - left-aligned, negative polarity
   - period register = $FF (yielding a duty cycle range of 0% to 100%,
     for duty cycle register values of $00 to $FF 
   - duty register = $00 (motor initially stopped)
                         
 IMPORTANT: Need to set MODRR so that PWM Ch 3 is routed to port pin PT3
*/

    MODRR = 0x08;           // PWM output channel 3
    //PWME  = 0x08;
    PWME_PWME3  = 1;        // enable PWM Ch 3
    PWMPOL_PPOL3  = 0;      // set active low polarity
    PWMCTL  = 0x00;         // no concatenate (8bit)
    PWMCAE  = 0x00;         // left-aligned output mode
    PWMPER3 = 0xff;         // set maximum 8-bit period
    PWMDTY3 = 0x00;         // initially clear Duty Register
    PWMSCLB = 37;
    
    
    PWMCLK_PCLK3  = 1;      // select Clock SB for Ch 3
    PWMPRCLK  = 0xff;       // set Clock B = -- MHz (prescaler 128) rate
     	   			 		  			 		  		
 



/* 
 Initialize the ATD to sample a D.C. input voltage (range: 0 to 5V)
 on Channel 0 (connected to a 10K-ohm potentiometer). The ATD should
 be operated in a program-driven (i.e., non-interrupt driven), normal
 flag clear mode using nominal sample time/clock prescaler values,
 8-bit, unsigned, non-FIFO mode.
                         
 Note: Vrh (the ATD reference high voltage) is connected to 5 VDC and
       Vrl (the reference low voltage) is connected to GND on the 
       9S12C32 kit.  An input of 0v will produce output code $00,
       while an input of 5.00 volts will produce output code $FF
*/

    DDRAD   = 0;
    ATDDIEN = 0xC0;
    ATDCTL2 = 0x80;
    ATDCTL3 = 0x10;
    ATDCTL4 = 0x85;

    	 	   			 		  			 		  		


	 	   			 		  			 		  		

/* 
  Initialize the pulse accumulator (PA) for event counting mode,
  and to increment on negative edges (no PA interrupts will be utilized,
  since overflow should not occur under normal operating conditions)
*/

    PACTL = 0x40;         // pulse accumulator system enable (1) (bit 6)
                          // pulse acc mode - event counter (0) (bit 5)
                          // count on falling edges (0) (bit 4)  
    		     



/*
  Initialize the RTI for an 2.048 ms interrupt rate
*/

    RTICTL = 0x1f;
    CRGINT = CRGINT | 0x80;
    
  
/*
  Initialize SPI for baud rate of 6 Mbs, MSB first
  (note that R/S, R/W', and LCD clk are on different PTT pins)
*/
    PTM = 0;
    DDRM = 0xff;        
    SPICR1 = 0x50;
    SPICR2 = 0x00;
    SPIBR = 0x01;


/* Initialize digital I/O port pins */

    DDRT = 0x7f;
    


/* 
   Initialize the LCD
     - pull LCDCLK high (idle)
     - pull R/W' low (write state)
     - turn on LCD (LCDON instruction)
     - enable two-line mode (TWOLINE instruction)
     - clear LCD (LCDCLR instruction)
     - wait for 2ms so that the LCD can wake up     
*/ 
    PTT_PTT6 = 1;
    PTT_PTT5 = 0;
    send_i(LCDON);
    send_i(TWOLINE);
    send_i(LCDCLR);
    lcdwait();
    
/* Initialize additional variables */

    tencnt  = 0;
    onecnt  = 0;
    prevpbr = 0;
    prevpbl = 0;
    per_ratio = 0;
    percent = 0;
    space = 0;
    
    set = 0;
    ampm = 0;
    tmp = 0;
    tmp_1 = 0;
    hrs_t = 0;
    hrs_o = 0;
    min_t = 0;
    min_o = 0;
    sec_t = 0;
    sec_o = 0;

    hrs = 0;
    min = 0;
    secns = 0;
    leftpbT = 0;	 	   			 		  			 		  		

	      
}

/*	 		  			 		  		
***********************************************************************
Main
***********************************************************************
*/
void main(void) {
  	DisableInterrupts
	initializations(); 		  			 		  		
	EnableInterrupts;
	
	while (1)
	{
	  pmsgterm("Enter hours: ");
    tmp = inchar();
    outchar(tmp);
    tmp_1 = inchar();
    outchar(tmp_1);
  
    hrs_t = tmp - 0x30;
    hrs_o = tmp_1 - 0x30;
  
    hrs = hrs_t * 10 + hrs_o;
  
    if(hrs > 12 || hrs < 0)
    {
      pmsgterm("Invalid hours!");
    } else
    {
      break;
    }
  
	}
	
	while (1)
	{
	  pmsgterm("Enter mins: ");
    tmp = inchar();
    outchar(tmp);
    tmp_1 = inchar();
    outchar(tmp_1);
  
    min_t = tmp - 0x30;
    min_o = tmp_1 - 0x30;
  
    min = min_t * 10 + min_o;
  
    if(min > 59 || min < 0)
    {
      pmsgterm("Invalid minutes!");
    } else
    {
      break;
    }
  
	}
	
	while (1)
	{
	  pmsgterm("Enter seconds: ");
    tmp = inchar();
    outchar(tmp);
    tmp_1 = inchar();
    outchar(tmp_1);
  
    sec_t = tmp - 0x30;
    sec_o = tmp_1 - 0x30;
  
    secns = sec_t * 10 + sec_o;
  
    if(secns > 59 || secns < 0)
    {
      pmsgterm("Invalid minutes!");
    } else
    {
      break;
    }
  
	}
	
	while (1)
	{
	  pmsgterm("a/p: ");
    ampm = inchar();
    outchar(ampm);
    
    
    //tmp_1 = inchar();
    //outchar(tmp_1);
  
    
    //sec_o = tmp_1 - 0x30;
  
    //secns = sec_t * 10 + sec_o;
  
    if(ampm == 0x61)    // a
    {
      ampm = 0; 
      break;
    } else if (ampm == 0x70)    // p
    {
      ampm = 1;
      break;
    } else 
    {
      pmsgterm("Invalid ampm!");
    }
  
	}
	
  
    

 for(;;) {

  
  

/* If right pushbutton pressed, start motor and turn on right LED (left LED off) */



    if(rghtpb) {
      
      rghtpb  = 0;
      //PWMDTY3 = 1;
      PWMDTY3 = 0x0F;
      RGHTLED = 1;
      LEFTLED = 0;
      runstp  = 1;
       
    }
      

/* If left pushbutton pressed, stop motor and turn on left LED (right LED off) */

    if(leftpb) {
    
      leftpb  = 0;
      PWMDTY3 = 0;
      RGHTLED = 0;
      LEFTLED = 1;
      runstp  = 0;
      leftpbT = !leftpbT;
    }

/*
 Every one-tenth second (when "tenths" flag set):
   - perform ATD conversion on Ch 0 
   - copy the converted value to the PWM duty register for Ch 0
*/

    if(tenths) {
      
      tenths  = 0;
      ATDCTL5 = 0x10;     // start conversion on ATD Ch 0
      while ((ATDSTAT0 & 0x80)!= 0x80);
      
      if(!runstp) {
        
        PWMDTY3 = ATDDR0H;
      }
 
      
    }
      
      
      
	
/* 	   			 		  			 		  		
 Every second (when "onesec" flag set):
   - read the PACNT value into pulscnt, then clear the PACNT register
   - convert the PACNT value to a 3-digit BCD  number and display it
     + on first line of LCD in "RPM = NNNN" format
     + on second line of LCD display as bar graph   
*/
    if(onesec) {
    
      onesec = 0; 
      
      rdisp();
    }
      
      
   
    
     
  } /* loop forever */
   
}   /* do not leave main */



/*
***********************************************************************                       
 RTI interrupt service routine: RTI_ISR
 Initialized for 8.192 ms interrupt rate
  Samples state of pushbuttons (PAD7 = left, PAD6 = right)
  If change in state from "high" to "low" detected, set pushbutton flag
     leftpb (for PAD7 H -> L), rghtpb (for PAD6 H -> L)
     Recall that pushbuttons are momentary contact closures to ground	
************************************************************************
*/

interrupt 7 void RTI_ISR(void)
{
  	// clear RTI interrupt flagt 
  	CRGFLG = CRGFLG | 0x80;
  	
  	/* PAD = 0 PREV = 0 -> Button is pressed currently, but it wasn't pressed previously
  	   PAD = 0 PREV = 1 -> Button is pressed currently, and it was pressed previously
  	   PAD = 1 PREV = 0 -> Button is not pressed currently, and it wasn't pressed previously
  	   PAD = 1 PREV = 1 -> Button is not pressed currently, but it was pressed previously
  	*/
  	
  	if((PORTAD0_PTAD7 + prevpbl) == 0)        // if left button is pressed and detected high to low edge
  	{
  		prevpbl = 1;                            // set previous push button left 
  		leftpb = 1;                             // set push button left 
  	} else if (PORTAD0_PTAD7)                 // if left putton is not pressed
  	{
  		prevpbl = 0;                            // clear previous push button left
  	}
  	
  	if((PORTAD0_PTAD6 + prevpbr) == 0)
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
  Initialized for 10.0 ms interrupt rate
  Uses variable "tencnt" to track if one-tenth second has accumulated
     and sets "tenths" flag 
                         
  Uses variable "onecnt" to track if one second has accumulated and
     sets "onesec" flag		 		  			 		  		
;***********************************************************************
*/

interrupt 15 void TIM_ISR(void)
{
  	// clear TIM CH 7 interrupt flag 
 	TFLG1 = TFLG1 | 0x80;
 	
 	tencnt++;
 	
 	if(tencnt == 10) {
 	  tenths = 1;
 	  tencnt = 0;
 	  onecnt++;
 	}
 	
 	if(onecnt == 10) {
 	  
 	  onesec = 1;
 	  onecnt = 0;
 	  secns++;
 	  
 	}
 	
 	if(secns == 60)
 	{
 	  secns = 0;
 	  min++;
 	}
 	
 	if(min == 60) {
 	
 	  hrs++;
 	  min = 0;
 	   
 	}
 	
 	if(hrs > 12)
 	{
 	  hrs = 1;
 	  ampm = !ampm;
 	}
 	


}

/*
***********************************************************************                       
  SCI (transmit section) interrupt service routine
                         
    - read status register to enable TDR write
    - check status of TBUF: if EMPTY, disable SCI transmit interrupts and exit; else, continue
    - access character from TBUF[TOUT]
    - output character to SCI TDR
    - increment TOUT mod TSIZE	
  NOTE: DO NOT USE OUTCHAR (except for debugging)	  			 		  		
***********************************************************************
*/

interrupt 20 void SCI_ISR(void)
{

  if(SCISR1 && SCISR1_TDRE_MASK){
    if(tin == tout){
      SCICR2_SCTIE = 0;
    }else{
      SCIDRL = tbuf[tout];
      tout = (tout + 1) % TSIZE;
    }
  }
 
}

void pmsgsci(char str[])
{
  int j = 0;
  while (str[j] != '\0') bco(j++);
}

/*
***********************************************************************                              
  SCI buffered character output routine - bco
  Places character x passed to it into TBUF
   - check TBUF status: if FULL, wait for space; else, continue
   - place character in TBUF[TIN]
   - increment TIN mod TSIZE
   - enable SCI transmit interrupts
  NOTE: DO NOT USE OUTCHAR (except for debugging)
***********************************************************************
*/

void bco(char x)
{

  if(((tin + 1) % TSIZE) != tout){
    tbuf[tin] = x;
    tin = (tin + 1) % TSIZE;
    SCICR2_SCTIE = 1;
  }
  

}

/*
***********************************************************************                              
 RPM display routine - rdisp
                         
 This routine starts by reading (and clearing) the 16-bit PA register.
 It then calculates an estimate of the RPM based on the number of
 pulses accumulated from the 64-hole chopper over a one second integration
 period and divides this value by 28 to estimate the gear head output
 shaft speed. Next, it converts this binary value to a 3-digit binary coded
 decimal (BCD) representation and displays the converted value on the
 terminal as "RPM = NNN" (updated in place).  Finally this RPM value, along
 with a bar graph showing ther percent-of-max of the current RPM value
 are shifted out to the LCD using pmsglcd.
***********************************************************************
*/

void rdisp()
{
  send_i(LCDCLR);
  chgline(LINE1);
  //PACNT = 15002;
  pulscnt = PACNT /30;                  // apprx of /28 /64 * 60                  
                                        // 64:pulses per revolution
                                        // 28:gear ratio 
                                        // 60: seconds to minutes  
  PACNT = 0;
  
  if(pulscnt <= maxrpm){
    
    if(leftpbT) {
      
      print_rpm(pulscnt);
      pmsglcd(" RPM");
    } else {
      print_time();
    }
  
    chgline(LINE2);
    per_ratio = pulscnt * 10 / maxrpm;      // 7
    percent = per_ratio * 10;               // 72
  
    space = 10 - per_ratio + 3;
  
    print_bar(per_ratio);
  
    print_percentage(percent, space);
  
  } else {
    
    pmsglcd("**   ERROR!   **");
    chgline(LINE2);
    pmsglcd("**RPM OVER MAX**");
    
  }

  

}


void print_rpm(int num) {
    
  print_c((num/1000) % 10 + 0x30);		    // 1000s
	print_c((num/100) % 10 + 0x30);         // 100s
	print_c((num/10) % 10 + 0x30);			    // 10s
  print_c((num%10) + 0x30);		            // 1s

  
}

void print_bar(int bar) {
  while(bar > 0)
  {
    print_c(0xff);
    bar--;
  }
  
}

void print_percentage(int percentage, int blank) {

  while(blank > 0)
  {
    print_c(0x20);
    blank--;  
    
  }
  
  print_c((percentage/10) % 10 + 0x30);			    // 10s
	print_c((percentage%10) + 0x30);		          // 1s
	print_c(0x25);                                // '%'

}

void print_time() {
  pmsglcd("TIME ");
  print_c(hrs/10 + 0x30);
  print_c(hrs%10 + 0x30);
  print_c(0x3a);
  print_c(min/10 + 0x30);
  print_c(min%10 + 0x30);
  print_c(0x3a);
  print_c(secns/10 + 0x30);
  print_c(secns%10 + 0x30);
  print_c(0x20);
  print_c(0x20);
    
  if(!ampm)
  {
    print_c(0x41);
  } else
  {
    print_c(0x50);
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
 
  // test the SPTEF bit: wait if 0; else, continue
  // write data x to SPI data register
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
	   PTT_PTT6 = 0;
	   PTT_PTT6 = 1;
	   PTT_PTT6 = 0;
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
  
  PTT_PTT4 = 0;
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

  PTT_PTT4 = 1;
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
  CUSTOM :: BONUS
  pmsgterm: print character string str[] on Terminal
***********************************************************************
*/

void pmsgterm(char str[])
{
  int i = 0;
	while(str[i] != '\0')
	{
		outchar(str[i]);
		i++;
	}
}


/*
***********************************************************************
 Character I/O Library Routines for 9S12C32 
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
 Name:         outchar    (use only for DEBUGGING purposes)
 Description:  outputs ASCII character x to SCI serial port
 Example:      outchar('x');
***********************************************************************
*/

void outchar(char x) {
  /* sends a character to the terminal channel */
    while (!(SCISR1 & 0x80));  /* wait for output buffer empty */
    SCIDRL = x;
}


