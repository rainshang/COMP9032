/*
 * PowerSum2.asm
 *
 *  Created: 8/15/2017 7:44:11 PM
 *   Author: Ethan

 7 registers version!!
 */ 

 .include "m2560def.inc"

.def sum0 = r16
.def sum1 = r17
.def a = r18;need to set the value when running
.def n = r19

;a1:a0 * b0
;-->
;		 a1:a0
;X			b0
;-------------
;		 a0*b0
;	  a1*b0
;-------------
;	  c2:c1:c0

;multi a1, a0, b0
;using x, result is x
.macro mul16_8
	mul @1, @2;a0*b0
	mov xh, r1
	mov xl, r0;x=a0*b0
	mul @0, @2;a1*b0
	add xh, r0;then xh = c1 = high(a0*b0) + low(a1*b0), so x = c1:c0, c2 is abandoned
.endmacro

;addw a1, a0, b1, b0
;result is a1:a0
.macro add16_16
	add @1, @3
	adc @0, @2;add with carry
.endmacro

main:
	ldi a, 1
	ldi n, 250
	.def i = r20
	ldi i, 1
	ldi xh, 0
	ldi xl, 1;a[0]=1
	clr sum1
	clr sum0
	loop:
		cp n, i
		brlo end
		mul16_8 xh, xl, a;a[n]=a[n-1]*a
		add16_16 sum1, sum0, xh, xl; 16 bits add
		inc i
		rjmp loop
	end:
		rjmp end