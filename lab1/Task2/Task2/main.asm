;
; Task2.asm
;
; Created: 8/12/2017 6:00:12 PM
; Author : Ethan
;


; Replace with your application code

.include "m2560def.inc"

.set student_id=0
.set name=student_id+4
.set WAM=name+20
.set STUDENT_RECORD_SIZE=WAM+1

.cseg
start:	
	ldi zh, high(s1_value<<1)
	ldi zl, low(s1_value<<1)
	ldi yh, high(s1)
	ldi yl, low(s1)
	clr r16
load:
	cpi r16, STUDENT_RECORD_SIZE
	brge end
	lpm r10, z+
	st y+, r10
	inc r16
	rjmp load
end:
	rjmp end

s1_value:
	.dw LWRD(123456)
	.dw HWRD(123456)
	.db "John Smith       ", 0
	.db 75

.dseg
.org 0x200

s1:
	.byte STUDENT_RECORD_SIZE