/*
 * PowerSum.asm
 *
 *  Created: 8/13/2017 10:40:40 AM
 *   Author: Ethan Xu, z5108944
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
.macro multi
	mul @1, @2;a0*b0
	mov xh, r1
	mov xl, r0;x=a0*b0
	mul @0, @2;a1*b0
	add xh, r0;then xh = c1 = high(a0*b0) + low(a1*b0), so x = c1:c0, c2 is abandoned
.endmacro

;power a, n
;using yl, z, result is z
.macro power
	ldi zh, 0
	ldi zl, 1;z=00:01, because a pow 0 = 1
	ldi yl, 1; yl as i for loop count
	loop:
		cp @1, yl
		brlo endloop
		multi zh, zl, @0
		mov zh, xh
		mov zl, xl; let z = x
		inc yl;i++
		rjmp loop
	endloop:
.endmacro

;addw a1, a0, b1, b0
;result is a1:a0
.macro addw
	add @1, @3
	adc @0, @2;add with carry
.endmacro

main:
	.def i = r20
	ldi i, 1
	clr sum1
	clr sum0
	loop:
		cp n, i
		brlo end
		power a, i
		addw sum1, sum0, zh, zl; 16 bits add
		inc i
		rjmp loop
	end:
		rjmp end