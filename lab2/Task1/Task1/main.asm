;
; Task1.asm
;
; Created: 8/19/2017 10:10:00 AM
; Author : Ethan
;


; Replace with your application code
;.include "m2560def.inc"

.def dividendH = r16
.def dividendL = r17
.def divisorH = r18
.def divisorL = r19

.def quotientH = r20
.def quotientL = r21
.def bit_positionH = r22
.def bit_positionL = r23

.equ divisoreThreshold = 0x8000

;aH, aL, bH, bL
.macro cpw
	cp @0, @2
	brne endCpw;if there is result for high bits comparision, return
	cp @1, @3;high bit equal, then compare low bits
	endCpw:
.endmacro

;aH, aL, 0x0000
.macro cpwi
	cpi @0, high(@2)
	brne endCpw;if there is result for high bits comparision, return
	cpi @1, low(@2);high bit equal, then compare low bits
	endCpw:
.endmacro

;addw aH, aL, bH, bL
;result is aH:aL
.macro addw
	add @1, @3
	adc @0, @2;add with carry
.endmacro

;subw aH, aL, bH, bL
;result is aH:aL
.macro subw
	sub @1, @3
	sbc @0, @2;sub with carry
.endmacro

;shift left aH, aL
.macro lslw
	lsl @1
	brcs lshc;if has carry, lsl high bits with carry
		lsl @0
		rjmp endLsw
	lshc:
		lsl @0
		inc @0;add carray
	endLsw:
.endmacro

;shift right aH, aL
.macro lsrw
	lsr @0
	brcs lslc;if has carry, lsr low bits with carry
		lsr @1
		rjmp endLsw
	lslc:
		lsr @1
		ldi xl, 0x80
		add @1, xl;add carry (1000:0000)
	endLsw:
.endmacro

main:
	.cseg 
	dividend_divisor: .dw 20000, 256;0x0C91-->3217, 0x0010--->16

	ldi zh, high(dividend_divisor<<1)
	ldi zl, low(dividend_divisor<<1)

	lpm dividendL, z+;little end storage strategy
	lpm dividendH, z+
	lpm divisorL, z+
	lpm divisorH, z+

	ldi quotientH, 0
	ldi quotientL, 0
	ldi bit_positionH, 0
	ldi bit_positionL, 1
	
	loop1:
		cpw divisorH, divisorL, dividendH, dividendL
		brsh endLoop1;divisor>=dividend, break
			cpwi divisorH, divisorL, divisoreThreshold
			brsh endLoop1;divisor>=0x8000, break
				lslw divisorH, divisorL
				lslw bit_positionH, bit_positionL
				rjmp loop1
	endLoop1:

	loop2:
		cpwi bit_positionH, bit_positionL, 1
		brsh normal;bit_position >= 1
			rjmp endLoop2
		normal:
			cpw dividendH, dividendL, divisorH, divisorL
			brlo then;dividend >= divisor
				subw dividendH, dividendL, divisorH, divisorL
				addw quotientH, quotientL, bit_positionH, bit_positionL
			then:
			lsrw divisorH, divisorL
			lsrw bit_positionH, bit_positionL
			rjmp loop2
	endLoop2:;dividend is remainder

	.dseg;declare the two valiable in data memory, address is from 0x0200 (default start) and 0x0202
	quotient: .byte 2
	remainder: .byte 2

	.cseg
	ldi xh, high(quotient)
	ldi xl, low(quotient);load *quotient into x
	ldi yh, high(remainder)
	ldi yl, low(remainder);load *remainder into y

	st x+, r20
	st x, r21;save quotientH:quotientL into quotient
	st y+, r16
	st y, r17;save dividendH:dividendL into remainder

end:
	rjmp end