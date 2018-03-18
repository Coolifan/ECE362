;
; ECE 362 - Experiment 5 - Step 2
;
; Equates

PEAR    equ     $000A
MODE    equ     $000B

        org     $0800

; Initialization for normal expanded wide mode with IVIS bit set

        movb    #$0C,PEAR ; enable ECLK, LSTRB, and R/W output
        movb    #$E9,MODE	; switch to normal expanded wide mode
			                  	; with internal visibility enabled
	jmp     loop
			  
	org     $0900                	
loop   	nop
        nop

; Two byte reads followed by two byte writes and two word writes

        ldaa    $0A00
        ldab    $0A01
        staa    $0A02
        stab    $0A03
        std     $0B00
        exg     a,b
        std     $0B82

        bra     loop

; Test Data

        org     $0A00

        fcb     $55
        fcb     $CC
       
        end
