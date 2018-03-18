; ECE 362 - Lab 3 - Step 3

; Name: < Yifan Li >

; Lab: < 8 >

; Class Number: < 8956L >

; Use Code Warrior (CW) in Full Chip Simulation mode

;***********************************************************************
; Write a subroutine "correl" that calculates the correlation between
; two 16-bit binary values, where the correlation is defined as the
; number of corresponding bit positions that are identical.
;
; At entry, the X and Y registers contain the two 16-bit operands;
; at exit, the B register contains the correlation value (which can
; range from $00 to $10).
;
; Examples:
;
; if   (X) = %10101010 01010101 = $AA55 and (Y) = %01010101 10101010 = $55AA
; then (B) = $00
;
; if   (X) = %11001100 11001100 = $CCCC and (Y) = %01010101 10101010 = $55AA
; then (B) = $08
;
; if   (X) = %00000000 00000000 = $0000 and (Y) = %00000000 00000000 = $0000
; then (B) = $10
;
;***********************************************************************
;
; To test and auto-grade your solution:
;	- Use CodeWarrior to assemble your code and launch the debugger
;	- Load the Auto-Grader (L3AG-3.s19) into the debugger
;		> Choose File -> Load Application
;		> Change the file type to "Motorola S-Record (*.s*)"
;		> Navigate to the 'AutoGrade' folder within your project
;		> Open 'L3AG-3.s19'
; - Open and configure the SCI terminal as a debugger component
;	- Start execution at location $800
;
; The score displayed is the number of test cases your code has passed.
; If nothing is displayed (i.e., your code "crashes"), no points will be
; awarded - note that there is no way to "protect" the application that
; tests your code from stack errors inflicted by mistakes in your code.
;
; Also note: If the message "STACK CREEP!" appears, it means that the
; stack has not been handled correctly (e.g., more pushes than pops or
; data passed to it not de-allocated correctly). 
;
;***********************************************************************

	org	$A00	; DO NOT REMOVE OR MODIFY THIS LINE

; Place your code for "correl" below

correl

      
      PSHX
      EXG Y,D
      LDX #$0 
  
      EORA 0,SP
      COMA
      
      EORB 1,SP
      COMB
       
 
loop1 CPD #$0
      BEQ over
      ASLD
      BCC loop1 
      INX
      BRA loop1
      
over  EXG B,X
      PULX
      
      
      
                                 
  

	rts

	end
