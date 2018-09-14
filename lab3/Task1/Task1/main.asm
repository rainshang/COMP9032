;
; Task1.asm
;
; Created: 9/3/2017 12:27:13 PM
; Author : Ethan
;


; Replace with your application code

.macro checkPb0Click
	sbis pind, 0;pd7, is on bit 0; and bit0 = 0 when pressed down, 1; bit0 = 1 goto checkPb0Click, 1 + 2 = 3
	rjmp end;if 0, 2
.endmacro

.equ loop_count=1333333;(16Mhz - 7) / 12 = 0x145855
.def i0=r23
.def i1=r24
.def i2=r25;r24:r25:r26
.def count0=r16
.def count1=r17
.def count2=r18;r16:r17:r18 = 0x145855
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
		checkPb0Click;3
		cp i2, count2;1
		cpc i1, count1;1
		cpc i0, count0;1
		brsh done;1 / 2
		adiw i1:i2, 1;2
		adc i0, zero;1
		rjmp loop1;2
	done:
.endmacro

.cseg
patterns: .db 0x01, 0x04, 0x10, 0x40;0000 0001, 0000 0100, 0001 0000, 0100 0000

ser r20
;out ddrg, r20;pg2~led0, pg3~led1
out ddrc, r20;pc0~led2, ..., pc7~led9

clr r20
out ddrd, r20; pd7~button PB0

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
	rjmp loop
end:
	rjmp end

