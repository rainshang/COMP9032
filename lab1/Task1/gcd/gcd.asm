;
; gcd.asm
;
; Created: 8/8/2017 7:59:12 PM
; Author : Ethan Xu, z5108944
;Calculating the greatest common divisor of two numbers) into AVR assembly code (gcd.asm).
; Assume the size of an integer is 1 byte.

;this is a minimum code size version


.include "m2560def.inc"
.def a = r16
.def b = r17

main:
	loop:
		cp a, b;compare a, b
		breq end; if a == b, stop

		brlo else; if b > a, goto else
			sub a, b; a = a - b
			rjmp loop
		else:
			sub b, a;b = b - a
			rjmp loop
		
	end:
	;	rjmp end