;
; Task1.asm
;
; Created: 9/30/2017 7:38:50 PM
; Author : Ethan
;

.macro STORE
	.if @0 > 63
		sts @0, @1; store directly to SRAM
	.else
		out @0, @1; out to Port
	.endif
.endmacro

.macro LOAD
	.if @1 > 63
		lds @0, @1; load directly from SRAM
	.else
		in @0, @1; in from port
	.endif
.endmacro

.macro do_lcd_command
	ldi r16, @0
	rcall lcd_command; relative call subroutine
	rcall lcd_wait
.endmacro

.macro do_lcd_data
	mov r16, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro

.macro lcd_set
	sbi LCD_CTRL_PORT, @0
.endmacro

.macro lcd_clr
	cbi LCD_CTRL_PORT, @0
.endmacro

;keypad
.def row=r16; current row number
.def col=r17; current column number
.def rmask=r18; mask for current row during scan
.def cmask=r19; mask for current column during scan
.def temp1=r20
.def temp2=r21
.def last_input_number=r22; record last input number, in case of the fucking signal
.def input_count=r23; how many symbols displaying

.equ PORTL_DIR=0xF0; PL7-4: output, PF3-0: input
.equ INIT_COL_MASK=0xEF; scan from the leftmost column
.equ INIT_ROW_MASK=0x01; scan from the top row
.equ ROW_MASK=0x0F;	for obtaining input from PortL
.equ MAX_SYMBOLS_PER_LINE=16
.equ MAX_SYMBOLS_PER_SCREEN=MAX_SYMBOLS_PER_LINE * 2

;LCD
.equ LCD_CTRL_PORT = PORTA
.equ LCD_CTRL_DDR = DDRA
.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4	

.equ LCD_DATA_PORT = PORTF
.equ LCD_DATA_DDR = DDRF
.equ LCD_DATA_PIN = PINF

reset:
	;keypad
	ldi temp1, PORTL_DIR; PL7:4/PL3:0, out/in
	ldi zh, high(DDRL); Port A~G have seperate I/O addresses; Port H~L have memory-mapped addresses
	ldi zl, low(DDRL)
	st z, temp1
	clr last_input_number

	;LCD
	ldi r16, low(RAMEND)
	out SPL, r16
	ldi r16, high(RAMEND)
	out SPH, r16

	ser r16
	STORE LCD_DATA_DDR, r16
	STORE LCD_CTRL_DDR, r16
	clr r16
	STORE LCD_DATA_PORT, r16
	STORE LCD_CTRL_PORT, r16

	clr input_count

	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001110 ; Cursor on, bar, no blink

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
	subi temp1, -'1'; add '1'
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
	jmp convert_end

star:
	ldi temp1, '*'
	jmp convert_end

zero:
	ldi temp1, '0'
	jmp convert_end

numbers:
	rjmp convert_end

convert_end:
	cp last_input_number, temp1
	breq wtfSignal; same, maybe fucking signal
	cpi input_count, MAX_SYMBOLS_PER_SCREEN
	breq clean_screen
	cpi input_count, MAX_SYMBOLS_PER_LINE
	brne display
	do_lcd_command 0b11000000; move cursor to next line
	rjmp display
	clean_screen:
		do_lcd_command 0b00000001; clear display
		clr input_count
display:
	mov last_input_number, temp1; not same, record this number
	do_lcd_data temp1
	inc input_count
	jmp init_col; restart scan keyboard input

wtfSignal:
	clr last_input_number
	rcall sleep_keypad
	jmp init_col

;Subroutines
lcd_command:
	STORE LCD_DATA_PORT, r16
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	lcd_clr LCD_E
	rcall sleep_1ms
	ret; Subroutine Return

lcd_data:
	STORE LCD_DATA_PORT, r16
	lcd_set LCD_RS
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	lcd_clr LCD_E
	rcall sleep_1ms
	lcd_clr LCD_RS
	ret

lcd_wait:
	push r16
	clr r16
	STORE LCD_DATA_DDR, r16
	STORE LCD_DATA_PORT, r16
	lcd_set LCD_RW
	lcd_wait_loop:
		rcall sleep_1ms
		lcd_set LCD_E
		rcall sleep_1ms
		LOAD r16, LCD_DATA_PIN
		lcd_clr LCD_E
		sbrc r16, 7
		rjmp lcd_wait_loop
	lcd_clr LCD_RW
	ser r16
	STORE LCD_DATA_DDR, r16
	pop r16
	ret

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4; 4 cycles per iteration - setup/call-return overhead

sleep_1ms:
	push r24
	push r25
	ldi r25, high(DELAY_1MS)
	ldi r24, low(DELAY_1MS)
	delayloop_1ms:
		sbiw r25:r24, 1
		brne delayloop_1ms
	pop r25
	pop r24
	ret

sleep_5ms:
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	ret

.equ DELAY_KEYPAD = (F_CPU / 4 - 7 - 2 * 7 * 2) / 9; 250ms
.def i0=r16
.def i1=r24
.def i2=r25
.def count0=r17
.def count1=r18
.def count2=r19
.def o=r20; zero

sleep_keypad:
	push r16; 2 cycles
	push r24; 2
	push r25; 2
	push r17; 2
	push r18; 2
	push r19; 2
	push r20; 2

	ldi count0, byte3(DELAY_KEYPAD);1
	ldi count1, byte2(DELAY_KEYPAD);1
	ldi count2, byte1(DELAY_KEYPAD);1
	clr o;1
	clr i0;1
	clr i1;1
	clr i2;1
	loop_keypad:
		cp i2, count2;1
		cpc i1, count1;1
		cpc i0, count0;1
		brsh loop_keypad_done;1 / 2
		adiw i1:i2, 1;2
		adc i0, o;1
		rjmp loop_keypad;2

loop_keypad_done:
	pop r20; 2 cycles
	pop r19; 2
	pop r18; 2
	pop r17; 2
	pop r25; 2
	pop r24; 2
	pop r16; 2
	ret