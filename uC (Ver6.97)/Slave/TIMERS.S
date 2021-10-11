/*
This function produces a delay - works only with 80MHz clock!!!
w0 - delay in usec
*/
 .global _Delay
_Delay:
	clr TMR2
	
	mov #20, w1
	mul.uu w0, w1, w0

	; setting TMR2 period
	mov w0, PR2

	; select 1:1 prescaler
	bclr T2CON, #5
	bclr T2CON, #4

	; 16 bit mode
	bclr T2CON, #3		; clear bit T32

	; turn on timer
	bset T2CON, #15		; start timer by setting TON

    bclr IFS0, #6		; clear interrupt flag, T2IF

Loop1: btss IFS0, #6
	bra Loop1
    
    return


