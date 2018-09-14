;
; Task3.asm
;
; Created: 9/16/2017 11:58:23 AM
; Author : Ethan
;

.equ loop_count=444444;0.25s = (16Mhz / 4 - 7) / 9
.def i0=r16
.def i1=r26
.def i2=r27
.def count0=r17
.def count1=r18
.def count2=r19
.def o=r20;
.macro falshDelay
	ldi count0, byte3(loop_count);1 cycle
	ldi count1, byte2(loop_count);1
	ldi count2, byte1(loop_count);1
	clr o;1
	clr i0;1
	clr i1;1
	clr i2;1
	falshDelayLoop:
		cp i2, count2;1
		cpc i1, count1;1
		cpc i0, count0;1
		brsh falshDelayDone;1 / 2
		adiw i1:i2, 1;2
		adc i0, o;1
		rjmp falshDelayLoop;2
	falshDelayDone:
.endmacro

.macro flashLED
	ser temp1
	out PORTC, temp1
	falshDelay
	clr temp1
	out PORTC, temp1
	falshDelay
	ser temp1
	out PORTC, temp1
	falshDelay
	clr temp1
	out PORTC, temp1
	falshDelay
	ser temp1
	out PORTC, temp1
	falshDelay
	clr temp1
	out PORTC, temp1
.endmacro

.def row=r16; current row number
.def col=r17; current column number
.def rmask=r18; mask for current row during scan
.def cmask=r19; mask for current column during scan
.def temp1=r20
.def temp2=r21

.def multiplier0=r22
.def multiplier1=r23
.def flag_input=r24;see WHICH_INPUT_BIT
.def last_input_number=r25; record last input number, in case of the fucking signal

.equ PORTL_DIR=0xF0; PL7-4: output, PF3-0: input
.equ INIT_COL_MASK=0xEF; scan from the leftmost column
.equ INIT_ROW_MASK=0x01; scan from the top row
.equ ROW_MASK=0x0F;	for obtaining input from PortL

.equ WHICH_INPUT_BIT=0;
.equ WHICH_INPUT_M0=0; is inputting multiplier0
.equ WHICH_INPUT_M1=1; is inputting multiplier1
.equ TEN=10
.equ NON_DUPLICATE=10;numbers from keypad is 0~9

reset:
	ldi temp1, PORTL_DIR; PL7:4/PL3:0, out/in
	ldi zh, high(DDRL); Port A~G have seperate I/O addresses; Port H~L have memory-mapped addresses
	ldi zl, low(DDRL)
	st z, temp1

	ser temp1
	out DDRC, temp1;pc0~led2, ..., pc7~led9

	clr multiplier0
	clr multiplier1
	ldi flag_input, WHICH_INPUT_M0; is inputting multiplier0
	ldi last_input_number, NON_DUPLICATE

init_col:
	ldi cmask, INIT_COL_MASK; initialise column mask
	clr col; initialise column

col_loop:
	cpi col, 4; 2560's keyboard is 4*4
	breq init_col; if all keys are scanned, repeat
	ldi zh, high(PORTL)
	ldi zl, low(PORTL)
	st z, cmask; otherwise, scan a column

	ldi temp1, 0xFF; slow down the scan operation
delay:
	dec temp1
	brne delay

	ldi zh, high(PINL)
	ldi zl, low(PINL)
	ld temp1, z;read Pin L
	andi temp1, ROW_MASK; get the keyborad output value
	cpi temp1, 0xF; check if any row is low
	breq next_col

init_row:
	ldi rmask, INIT_ROW_MASK; if yes, find which row is low; initialise for row check
	clr row

row_loop:
	cpi row, 4
	breq next_col; the row scan is over
	mov temp2, temp1
	and temp2, rmask; check un-marked bit
	breq convert; if bit is clear, the key is pressed
	inc row; else move to the next row
	lsl rmask
	jmp row_loop

next_col:; if row scan is over
	inc col; move to the next col
	lsl cmask
	jmp col_loop

convert:
	cpi col, 3; if the pressed key is in col 3
	breq letters; is letter

	cpi row, 3; if the pressed key in in row 3
	breq symbols; is symbol or 0

	mov temp1, row; is number 1~9
	lsl temp1
	add temp1, row
	add temp1, col; temp1 = row * 3 + col
	subi temp1, -1; add 1
	jmp numbers

letters:
	ldi temp1, 'A'
	add temp1, row; the ASCII value of the pressed key
	jmp convert_end

symbols:
	cpi col, 0; check if is '*'
	breq star
	cpi col, 1; if is '0'
	breq zero
	ldi temp1, '#'; otherwise, it is '#'
	jmp calculate

star:
	ldi temp1, '*'
	ldi flag_input, WHICH_INPUT_M1; is inputting multiplier1
	jmp convert_end

zero:
	ldi temp1, 0

numbers:
	cp last_input_number, temp1
	breq wtfSignal; same, maybe fucking signal
	mov last_input_number, temp1; not same, record this number
	sbrs flag_input, WHICH_INPUT_BIT;check inout for which multiplier
	rjmp handleMultiplier0
	jmp handleMultiplier1

wtfSignal:
	ldi last_input_number, NON_DUPLICATE
	falshDelay
	jmp convert_end

handleMultiplier0:
	;mov multiplier0, temp1
	;out PORTC, multiplier0; display multiplier0 on LED
	;jmp convert_end

	ldi temp2, TEN
	mul multiplier0, temp2
	mov temp2, r1
	cpi temp2, 0
	brne multiplier0Overflow; multiplier1 * 10 overflow
	mov multiplier0, r0
	add multiplier0, temp1
	brcs multiplier0Overflow; multiplier1 + temp1 overflow
	out PORTC, multiplier0
	jmp convert_end

multiplier0Overflow:
	clr multiplier0; invalidate multiplier0
	flashLED
	jmp convert_end

handleMultiplier1:
	;mov multiplier1, temp1
	;out PORTC, multiplier1; display multiplier0 on LED
	;jmp convert_end

	ldi temp2, TEN
	mul multiplier1, temp2
	mov temp2, r1
	cpi temp2, 0
	brne multiplier1Overflow; multiplier1 * 10 overflow
	mov multiplier1, r0
	add multiplier1, temp1
	brcs multiplier1Overflow; multiplier1 + temp1 overflow
	out PORTC, multiplier1
	jmp convert_end

multiplier1Overflow:
	clr multiplier1; invalidate multiplier1
	flashLED
	jmp convert_end

calculate:
	mul multiplier0, multiplier1
	mov temp2, r1
	cpi temp2, 0
	brne calculateOverflow
	out PORTC, r0; diplay result on LED
	ldi flag_input, WHICH_INPUT_M0
	jmp convert_end

calculateOverflow:
	flashLED
	ldi flag_input, WHICH_INPUT_M0
	jmp convert_end

convert_end:
	jmp init_col; restart scan keyboard input