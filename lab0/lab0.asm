.include "m2560def.inc"
.def a = r16
.def b = r17
.def c = r10
.def d = r11
.def e = r12

main:
	ldi a, 10
	ldi b, -20
	mov c, a
	add c, b
	mov d, a
	sub d, b
	lsl c
	asr d
	mov e, c
	add e, d
halt:
	rjmp halt
