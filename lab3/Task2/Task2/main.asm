;
; Task2.asm
;
; Created: 9/10/2017 1:55:31 PM
; Author : Ethan
;


; Replace with your application code

.equ loop_count=1777777;(16Mhz - 7) / 9 = 0x1B2071
.def i0=r23
.def i1=r24
.def i2=r25;r24:r25:r26
.def count0=r16
.def count1=r17
.def count2=r18;r16:r17:r18 = 0x1B2071
.def zero=r19;
.macro oneSecondDelay
	ldi count0, byte3(loop_count);1 cycle
	ldi count1, byte2(loop_count);1
	ldi count2, byte1(loop_count);1
	clr zero;1
	clr i0;1
	clr i1;1
	clr i2;1
	loop1:
		cp i2, count2;1
		cpc i1, count1;1
		cpc i0, count0;1
		brsh done;1 / 2
		adiw i1:i2, 1;2
		adc i0, zero;1
		rjmp loop1;2
	done:
.endmacro

jmp RESET
.org INT0addr
jmp EXT_INT0
reti

RESET:
	ser r20
	out ddrc, r20;pc0~led2, ..., pc7~led9

	clr r20
	out ddrd, r20;pd7~button PB0

	;set INT0 as falling edge triggered interrupt; but I think it is the low level of INT0 triggered interrupt
	ldi r20, (2 << ISC00)
	sts EICRA, r20

	;enable INT0
	in r20, EIMSK
	ori r20, (1<<INT0)
	out EIMSK, r20

	sei
	jmp loop

EXT_INT0:
	jmp end

.cseg
patterns: .db 0x01, 0x04, 0x10, 0x40;0000 0001, 0000 0100, 0001 0000, 0100 0000

loop:
	ldi zh, high(patterns<<1)
	ldi zl, low(patterns<<1)
	lpm r20, z+
	out portc, r20
	oneSecondDelay
	lpm r20, z+
	out portc, r20
	oneSecondDelay
	lpm r20, z+
	out portc, r20
	oneSecondDelay
	lpm r20, z+
	out portc, r20
	oneSecondDelay
	jmp loop
end:
	rjmp end