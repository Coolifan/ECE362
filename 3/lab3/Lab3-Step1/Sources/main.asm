; ECE 362 - Lab 3 - Step 1

; Name: < Yifan Li >

; Lab: < 8 >

; Class Number: < 8956L >

; Use Code Warrior (CW) in Full Chip Simulation mode

;***********************************************************************
;
; Write a REENTRANT subroutine "findc" that counts the number of instances
; of a specified (ASCII) character in the string that is passed to it.
; Recall that a reentrant subroutine does not use any global variables
; (i.e., any local variables utilized should be allocated on the stack.
;
; At entry, the (ASCII) character to match along with the string to search
; is passed via the stack, as shown; assume the string terminates with the
; ASCII NULL character (the string can be any length up to 255 characters).
;
; At exit, the number of instances of the specified character is returned
; via the stack (as an unsigned 8-bit binary number) in place of the character
; that was matched (the string should be deallocated from the stack).
;
; At entry:
;
;      +--------------------+
; SP ->| return address - H |
;      +--------------------+
;      | return address - L |
;      +--------------------+
;      | character to match |
;      +--------------------+
;      | string - firstchar |
;      +--------------------+
;      |       char         |
;      +--------------------+
;      |       char         |
;      +--------------------+
;      | string - lastchar  |
;      +--------------------+
;      |     ASCII NULL     |
;      +--------------------+
;
;
; Just prior to exit:
;
;      +--------------------+
; SP ->| return address - H |
;      +--------------------+
;      | return address - L |
;      +--------------------+
;      | number of matches  |
;      +--------------------+
;
;***********************************************************************
;
; To test and auto-grade your solution:
;	- Use CodeWarrior to assemble your code and launch the debugger
;	- Load the Auto-Grader (L3AG-1.s19) into the debugger
;		> Choose File -> Load Application
;		> Change the file type to "Motorola S-Record (*.s*)"
;		> Navigate to the 'AutoGrade' folder within your project
;		> Open 'L3AG-1.s19'
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

; Place your code for "findc" below


findc
  
      PULX    ; pull return address (2B) to X
      PULA    ; pull char to be tested (1B) to A
      LDY #$0 ; initialize total # of matches in Y
  

loop  PULB    ; pull 1 byte of string
      CBA     ; compare with specified char
      BEQ cmpmatch    ; if equal, match + 1
      
      CMPB #$00  ; compare with NULL
      BEQ over   ; if = NULL, over
      BRA loop
     
 
cmpmatch INY     ; # of matches +1
      BRA loop   ; return to loop

      
over  EXG A,Y    ; exchange Y register with A register    2->1byte
      PSHA       ; push back # of matches (1B)
      PSHX       ; push back return address (2B)
       
 
  


	rts

	end
