;
; gcd.asm
;
;  Created: 8/8/2017 8:14:38 PM
; Author : Ethan Xu, z5108944
;Calculating the greatest common divisor of two numbers) into AVR assembly code (gcd.asm).
; Assume the size of an integer is 1 byte.

;this is a minimum execution time version


.include "m2560def.inc"
.def a = r16
.def b = r17

main:
	loop:
		cp a, b;compare a, b
		breq end; if a == b, stop

		brlo else; if b > a, goto else
			rjmp loop1
		else:
			rjmp loop2
		
	end:

	loop1:
		cp a, b
		breq end1
		brlo end1

		sub a, b; a = a - b
		rjmp loop1

	end1:

	loop2:
		cp a, b
		breq end2
		brsh end2

		sub b, a; a = a - b
		rjmp loop2

	end2:

	;	rjmp end