;
; DroneProject.asm
;
; Created: 10/17/2017 11:52:37 AM
; Author : Ethan
;
; This is the assembly code for ATmega2560 to control a drone
; 2 PB buttons generate interrupt INT0 and INT3
; Timer0 generates 16ms interrupt as a timer to drive event handling

; LCD
; PORTF 0:7 ~ D0:7
; PORTE 5 ~ BL; not use
; PORTA 4:7 ~ BE,RW,E,RS

; LED
; PORTG 2:3 ~ 0:1
; PORTC 0:7 ~ 2:9

; PB
; PORTD 4 (RDX2) ~ PB1
; PORTD 7 (RDX4) ~ PB0

; KEYPAD
; PORTL 0:7 ~ C3:C0,R3:R0

; MOTOR
; PORTE 2 ~ MOT

; macros
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

.macro do_lcd_data_i
	ldi r16, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro

.macro lcd_set
	sbi LCD_CTRL_PORT, @0
.endmacro

.macro lcd_clr
	cbi LCD_CTRL_PORT, @0
.endmacro

; when drone is grounded, show the UI for user to re-input the accident location
.macro reinput_display
	do_lcd_command 0b00000001 ; clear display
	do_lcd_data_i 'G'
	do_lcd_data_i 'r'
	do_lcd_data_i 'o'
	do_lcd_data_i 'u'
	do_lcd_data_i 'n'
	do_lcd_data_i 'd'
	do_lcd_data_i 'e'
	do_lcd_data_i 'd'
	do_lcd_command 0b11000000; move cursor to next line
	do_lcd_data_i 'S'
	do_lcd_data_i 'e'
	do_lcd_data_i 't'
	do_lcd_data_i ' '
	do_lcd_data_i 'L'
	do_lcd_data_i 'o'
	do_lcd_data_i 'c'
	do_lcd_data_i '.'
	do_lcd_data_i ':'
.endmacro

; clear all the accident relative setting and memory
.macro reset_accident
	ldi yh, high(AccidentX)
	ldi yl, low(AccidentX)
	ldi inputXorY, -1
	st y+, inputXorY; x of accident is -1, invalid 
	st y, inputXorY; y of accident is -1, invalid 
	clr inputXorY; waiting for x input
	clr xyBitsCount
	clr inputX
	clr inputY
.endmacro

.macro display_search
	do_lcd_command 0b00000001 ; clear display
	do_lcd_data_i 'S'
	do_lcd_data_i 'e'
	do_lcd_data_i 'a'
	do_lcd_data_i 'r'
	do_lcd_data_i 'c'
	do_lcd_data_i 'h'
	do_lcd_data_i 'i'
	do_lcd_data_i 'n'
	do_lcd_data_i 'g'
	do_lcd_data_i '.'
	do_lcd_data_i '.'
	do_lcd_data_i 'H'
	do_lcd_data_i ':'
	mov r16, current_mountain_z; the height of the mountain at this point (drone needs to detect)
	rcall display_3bits_number
	do_lcd_command 0b11000000; move cursor to next line
	do_lcd_data_i 'D'
	do_lcd_data_i 'r'
	do_lcd_data_i 'o'
	do_lcd_data_i 'n'
	do_lcd_data_i 'e'
	do_lcd_data_i '('
	mov r16, drone_x
	rcall display_2bits_number
	do_lcd_data_i ','
	mov r16, drone_y
	rcall display_2bits_number
	do_lcd_data_i ','
	mov r16, drone_z
	rcall display_3bits_number
	do_lcd_data_i ')'
.endmacro

.macro display_halt
	do_lcd_command 0b00000001 ; clear display
	do_lcd_data_i 'H'
	do_lcd_data_i 'a'
	do_lcd_data_i 'l'
	do_lcd_data_i 't'
	do_lcd_data_i 'i'
	do_lcd_data_i 'n'
	do_lcd_data_i 'g'
	do_lcd_data_i '.'
	do_lcd_data_i '.'
	do_lcd_data_i '.'
	do_lcd_data_i ' '
	do_lcd_data_i 'H'
	do_lcd_data_i ':'
	mov r16, current_mountain_z
	rcall display_3bits_number
	do_lcd_command 0b11000000; move cursor to next line
	do_lcd_data_i 'D'
	do_lcd_data_i 'r'
	do_lcd_data_i 'o'
	do_lcd_data_i 'n'
	do_lcd_data_i 'e'
	do_lcd_data_i '('
	mov r16, drone_x
	rcall display_2bits_number
	do_lcd_data_i ','
	mov r16, drone_y
	rcall display_2bits_number
	do_lcd_data_i ','
	mov r16, drone_z
	rcall display_3bits_number
	do_lcd_data_i ')'
.endmacro

.macro display_abort
	do_lcd_command 0b00000001 ; clear display
	do_lcd_data_i 'M'
	do_lcd_data_i 'i'
	do_lcd_data_i 's'
	do_lcd_data_i 's'
	do_lcd_data_i 'i'
	do_lcd_data_i 'o'
	do_lcd_data_i 'n'
	do_lcd_data_i ' '
	do_lcd_data_i 'a'
	do_lcd_data_i 'b'
	do_lcd_data_i 'o'
	do_lcd_data_i 'r'
	do_lcd_data_i 't'
	do_lcd_data_i 'e'
	do_lcd_data_i 'd'
	do_lcd_command 0b11000000; move cursor to next line
	do_lcd_data_i 'S'
	do_lcd_data_i 'e'
	do_lcd_data_i 't'
	do_lcd_data_i ' '
	do_lcd_data_i 'L'
	do_lcd_data_i 'o'
	do_lcd_data_i 'c'
	do_lcd_data_i '.'
	do_lcd_data_i ':'
.endmacro

.macro display_found
	do_lcd_command 0b00000001 ; clear display
	do_lcd_data_i 'A'
	do_lcd_data_i 'c'
	do_lcd_data_i 'c'
	do_lcd_data_i 'i'
	do_lcd_data_i 'd'
	do_lcd_data_i 'e'
	do_lcd_data_i 'n'
	do_lcd_data_i 't'
	do_lcd_data_i ' '
	do_lcd_data_i 'f'
	do_lcd_data_i 'o'
	do_lcd_data_i 'u'
	do_lcd_data_i 'n'
	do_lcd_data_i 'd'
	do_lcd_data_i ':'
	do_lcd_command 0b11000000; move cursor to next line
	do_lcd_data_i '('
	mov r16, drone_x
	rcall display_2bits_number
	do_lcd_data_i ','
	mov r16, drone_y
	rcall display_2bits_number
	do_lcd_data_i ','
	mov r16, current_mountain_z
	rcall display_3bits_number
	do_lcd_data_i ')'
.endmacro

.macro display_not_found
	do_lcd_command 0b00000001 ; clear display
	do_lcd_data_i 'N'
	do_lcd_data_i 'o'
	do_lcd_data_i ' '
	do_lcd_data_i 'a'
	do_lcd_data_i 'c'
	do_lcd_data_i 'c'
	do_lcd_data_i 'i'
	do_lcd_data_i 'd'
	do_lcd_data_i 'e'
	do_lcd_data_i 'n'
	do_lcd_data_i 't'
	do_lcd_data_i '.'
	do_lcd_command 0b11000000; move cursor to next line
	do_lcd_data_i 'S'
	do_lcd_data_i 'e'
	do_lcd_data_i 't'
	do_lcd_data_i ' '
	do_lcd_data_i 'L'
	do_lcd_data_i 'o'
	do_lcd_data_i 'c'
	do_lcd_data_i '.'
	do_lcd_data_i ':'
.endmacro

.dseg
DroneStatus: .byte 1; drone status

; accident location, saved in .dseg to release registers
AccidentX: .byte 1
AccidentY: .byte 1

LEDTickCounter: .byte 1; count tick to TIMER_TICK_COUNT_1S
LEDStatus: .byte 1; indicate LED is on(1) or off(0)

MotorStatus: .byte 1; indicate motor is on(1) or off(0) (to generate half duty PWM signal)

.cseg
; enum of drone status
.equ GROUNDED=0
.equ SEARCHING=1
.equ HALTING=2
.equ ABORTED=3

.equ MAX_SCAN_DISTANCE=10; the max distance drone can see

.equ TIMER_TICK_COUNT_1S=62; timer0 generates interrupt every 16ms, 1s/16ms=62.5

; drone search
.def accident_x=r17
.def accident_y=r18
.def current_mountain_z=r19; when drone_x == accident_x && drone_y == accident_y, current_mountain_z is actually the accident point's z
.def drone_x=r20; current drone location
.def drone_y=r21
.def drone_z=r22
.def drone_ground_distance=r23; current drone to current mountain ground distance, drone can detect the ground when in (0, MAX_SCAN_DISTANCE]

; keypad
.def row=r16; current row number
.def col=r17; current column number
.def rmask=r18; mask for current row during scan
.def cmask=r19; mask for current column during scan
.def temp1=r20
.def temp2=r21
.def last_input_number=r22; record last input number, in case of the fucking signal

; the format of accident location must be xx*yy
.def inputXorY=r23; is waiting for x input(0) or y(1)
.def xyBitsCount=r24; to check whether input is 2 bits. input must satisfy the format xx, like 01, 34, etc.
.def inputX=r25; temp x,y, if valid they will be saved into .dseg to release registers
.def inputY=r26

.equ PORTL_DIR=0xF0; PL7-4: output, PF3-0: input
.equ INIT_COL_MASK=0xEF; scan from the leftmost column
.equ INIT_ROW_MASK=0x01; scan from the top row
.equ ROW_MASK=0x0F;	for obtaining input from PortL

; LCD
.equ LCD_CTRL_PORT=PORTA
.equ LCD_CTRL_DDR=DDRA
.equ LCD_RS=7
.equ LCD_E=6
.equ LCD_RW=5
.equ LCD_BE=4

.equ LCD_DATA_PORT=PORTF
.equ LCD_DATA_DDR=DDRF
.equ LCD_DATA_PIN=PINF

; program begin
.org 0x0000; main
jmp RESET

.org INT0addr; PORTD 7, PB0, start mission
jmp EXT_INT0

.org INT3addr; PORTD 4, PB1, abort mission
jmp EXT_INT3

.org OVF0addr; Timer Overflow Address
jmp Timer0OVF

RESET:
	; init statck pointer
	ldi r16, low(RAMEND)
	out SPL, r16
	ldi r16, high(RAMEND)
	out SPH, r16

	; init LCD
	ser r16
	STORE LCD_DATA_DDR, r16
	STORE LCD_CTRL_DDR, r16
	clr r16
	STORE LCD_DATA_PORT, r16
	STORE LCD_CTRL_PORT, r16

	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001110 ; Cursor on, bar, no blink
	reinput_display

	; init LED
	ser r16
	out DDRC, r16

	; init .dseg
	ldi yh, high(DroneStatus)
	ldi yl, low(DroneStatus)
	ldi r16, GROUNDED
	st y, r16; DroneStatus = GROUNDED; y = inputX

	ldi yh, high(LEDTickCounter)
	ldi yl, low(LEDTickCounter)
	clr r16
	st y+, r16; LEDTickCounter = 0; y = LEDStatus
	st y+, r16; LEDStatus = 0 (false); y = MotorStatus
	st y, r16; MotorStatus = 0 (false)

	; init PB buttons
	ldi r16, (2 << ISC00) | (2 << ISC30); set INT0~PB0, INT3~PB1 as falling edge triggered interrupt
	sts EICRA, r16
	in r16, EIMSK
	ori r16, (1<<INT0) | (1<<INT3); enable INT0~PB0, INT3~PB1 
	out EIMSK, r16

	; init keypad
	ldi temp1, PORTL_DIR; PL7:4/PL3:0, out/in
	ldi zh, high(DDRL); Port A~G have seperate I/O addresses; Port H~L have memory-mapped addresses
	ldi zl, low(DDRL)
	st z, temp1
	clr last_input_number

	; init accident 
	reset_accident

	; set timer interrupt
	ldi r16, 0x00; Set-up Timer0 Overflow, 8 bits counter
	out TCCR0A, r16
	ldi r16, 0x05; prescaling value=1024, 256*1024/16Mhz=16ms, thus, TIMER_TICK_COUNT_1S = 62
	out TCCR0B, r16
	ldi r16, 1<<TOIE0; enable the Overflow Interrupt
	sts TIMSK0, r16; T/C0 interrupt enable

	; keypad event loop
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
		;ldi temp1, 'A'
		;add temp1, row; the ASCII value of the pressed key
		;jmp convert_end
		jmp init_col; only numbers accepted

	symbols:
		cpi col, 0; check if is '*'
		breq star
		cpi col, 1; if is '0'
		breq zero
		;ldi temp1, '#'; otherwise, it is '#'
		;jmp convert_end
		jmp init_col; only numbers accepted

	star:
		;ldi temp1, '*'
		;jmp convert_end
		jmp init_col; only numbers accepted

	zero:
		ldi temp1, '0'
		jmp convert_end

	numbers:
		rjmp convert_end

	convert_end:
		cp last_input_number, temp1
		breq wtfSignal; same, maybe fucking ghost signal
		rjmp display

	wtfSignal:
		clr last_input_number
		rcall sleep_250ms
		jmp init_col
		
	display:
		mov last_input_number, temp1; not same, record this number
		do_lcd_data temp1
		subi temp1, '0'; char to int
		inc xyBitsCount
		cpi inputXorY, 0; is x inout or y
		breq computeX
		rjmp computeY
		computeX:; add to previous input bit
			cpi xyBitsCount, 2
			breq xMul10
			mov inputX, temp1; current is the the first bit of x
			jmp init_col
			xMul10:; current is the the second bit of x
				; inputX = inputX * 10 + temp1
				ldi temp2, 10
				mul inputX, temp2
				mov inputX, r0; cause even 9 * 10 = 90 can be stored in one register, so the lower byte is valid
				add inputX, temp1
				cpi inputX, 64; MOUNTAIN is 64*64
				brsh re_input; >= 64, invalid input
				do_lcd_data_i '*'
				ldi inputXorY, 1; waiting for y input
				clr xyBitsCount
				jmp init_col
		computeY:
			cpi xyBitsCount, 2
			breq yMul10
			mov inputY, temp1
			jmp init_col
			yMul10:
				; inputY = inputY * 10 + temp1
				ldi temp2, 10
				mul inputY, temp2
				mov inputY, r0
				add inputY, temp1
				cpi inputY, 64
				brsh re_input; >= 64, invalid input
				rjmp saveAccident
		re_input:; invalid input, re input
			reset_accident
			reinput_display
			jmp init_col

		; save accident point to .dseg, to release registers
		saveAccident:
			ldi yh, high(AccidentX)
			ldi yl, low(AccidentX)
			st y+, inputX
			st y, inputY
			; display accident has been set info
			do_lcd_command 0b00000001 ; clear display
			do_lcd_data_i 'L'
			do_lcd_data_i 'o'
			do_lcd_data_i 'c'
			do_lcd_data_i '.'
			do_lcd_data_i ' '
			do_lcd_data_i 'i'
			do_lcd_data_i 's'
			do_lcd_data_i ' '
			do_lcd_data_i 's'
			do_lcd_data_i 'e'
			do_lcd_data_i 't'
			do_lcd_data_i '.'
			do_lcd_command 0b11000000; move cursor to next line
			do_lcd_data_i 'P'
			do_lcd_data_i 'B'
			do_lcd_data_i '0'
			do_lcd_data_i ' '
			do_lcd_data_i 't'
			do_lcd_data_i 'o'
			do_lcd_data_i ' '
			do_lcd_data_i 's'
			do_lcd_data_i 't'
			do_lcd_data_i 'a'
			do_lcd_data_i 'r'
			do_lcd_data_i 't'
			do_lcd_data_i '!'
			jmp init_col

EXT_INT0:
	push yh
	push yl
	push accident_x
	push accident_y
	push zh
	push zl
	push r16

	sei; Global Interrupt Enable, nested interrupt support

	; read accident info
	ldi yh, high(AccidentX)
	ldi yl, low(AccidentX)
	ld accident_x, y+
	ld accident_y, y
	; 
	ldi yh, high(DroneStatus)
	ldi yl, low(DroneStatus)
	ld r16, y
	cpi r16, GROUNDED
	breq INT0_GROUNDED_or_ABORTED
	cpi r16, ABORTED
	breq INT0_GROUNDED_or_ABORTED
	rjmp INT0_END; not GROUNDED or ABORTED, do not start mission
	INT0_GROUNDED_or_ABORTED:
	; set drone status flag SEARCHING
	ldi r16, SEARCHING
	st y, r16
	; scan from (0,0)
	clr drone_x
	clr drone_y
	clr drone_z
	ldi zh, high(MOUNTAIN<<1)
	ldi zl, low(MOUNTAIN<<1)
scan_moutain_loop:
	lpm current_mountain_z, z+
	display_search
	rcall sleep_250ms

	; check is abort
	ld r16, y
	cpi r16, ABORTED
	breq mission_abort0
	jmp scan_moutain_loop_go_on
mission_abort0:
	display_abort
	jmp INT0_END
scan_moutain_loop_go_on:
	; set drone status flag HALTING
	ldi r16, HALTING
	st y, r16
compare_distance:
	display_halt
	rcall sleep_250ms

	; check is abort
	ld r16, y
	cpi r16, ABORTED
	breq mission_abort1
	jmp compare_distance_go_on
mission_abort1:
	display_abort
	jmp INT0_END
compare_distance_go_on:
	mov drone_ground_distance, drone_z
	sub drone_ground_distance, current_mountain_z
	cpi drone_ground_distance, 1
	brge cp10; >=1
	subi drone_z, -10; drone up 5
	rjmp compare_distance
	cp10:
	cpi drone_ground_distance, 11
	brlt drone_can_see; <11
	subi drone_z, 10; drone down 5
	rjmp compare_distance
drone_can_see:
	; if drone_x == accident_x && drone_y == accident_y, find; or next loop
	cp drone_x, accident_x
	brne next_scan
	cp drone_y, accident_y
	brne next_scan
	jmp accident_found
next_scan:
	; check is abort
	ld r16, y
	cpi r16, ABORTED
	breq mission_abort2
	jmp next_scan_go_on
mission_abort2:
	display_abort
	jmp INT0_END
next_scan_go_on:
	ldi r16, SEARCHING
	st y, r16
	display_search
	; move to next position
	cpi drone_x, 63
	breq check_y
	inc drone_x; drone_x !=63, drone_x++
	jmp scan_moutain_loop
	check_y:; drone_x ==63
	cpi drone_y, 63
	brne move_to_next_y
	jmp accident_not_found; drone_x ==63 && drone_y ==63
move_to_next_y:
	clr drone_x; move to next y
	inc drone_y
	jmp scan_moutain_loop
accident_found:
	display_found
	ldi r16, GROUNDED; accident found, make drone GROUNDED
	st y, r16
	jmp INT0_END
accident_not_found:
	display_not_found
	ldi r16, GROUNDED; accident not found, make drone GROUNDED
	st y, r16
INT0_END:
	pop r16
	pop zl
	pop zh
	pop accident_y
	pop accident_x
	pop yl
	pop yh
	reti

EXT_INT3:
	push yh
	push yl
	push r16

	sei

	ldi yh, high(DroneStatus)
	ldi yl, low(DroneStatus)
	ld r16, y
	cpi r16, SEARCHING
	breq INT3_SEARCHING_or_HALTING
	cpi r16, HALTING
	breq INT3_SEARCHING_or_HALTING
	cpi r16, GROUNDED
	breq INT3_reset_accident_input
	rjmp INT3_END; ABORTED
	INT3_SEARCHING_or_HALTING:
	; set drone status flag ABORTED
	ldi r16, ABORTED; save status, the event will be handled in EXT_INT0
	st y, r16
	INT3_reset_accident_input:
	reset_accident
	reinput_display

	INT3_END:
	pop r16
	pop yl
	pop yh
	reti

Timer0OVF:
	push yh
	push yl
	push r16

	sei
	
	ldi yh, high(DroneStatus)
	ldi yl, low(DroneStatus)
	ld r16, y
	cpi r16, SEARCHING
	breq handle_MOTOR_SEARCHING
	cpi r16, HALTING
	breq handle_MOTOR_HALTING
	clr r16
	out PORTE, r16; MOTOR off
	jmp handle_LED; not SEARCHING or HALTING

handle_MOTOR_SEARCHING:
	ser r16
	out PORTE, r16; MOTOR on, full power
	rjmp handle_LED

; turn motor on and off intervally to make it run at half speed
handle_MOTOR_HALTING:
	ldi yh, high(MotorStatus)
	ldi yl, low(MotorStatus)
	ld r16, y
	cpi r16, 0
	breq motorOn
		clr r16; if !=0, off
		rjmp saveMotor
	motorOn:
		ser r16
	saveMotor:
		st y, r16
		out PORTE, r16

handle_LED:
	ldi yh, high(LEDTickCounter)
	ldi yl, low(LEDTickCounter)
	ld r16, y
	cpi r16, TIMER_TICK_COUNT_1S; check LED tick loop is 1s
	brne LEDTickCounter_INC
		clr r16
		st y, r16; reset LED tick
		; if LEDStatus is on, then off LED and save the status; vice versa
		ldi yh, high(LEDStatus)
		ldi yl, low(LEDStatus)
		ld r16, y
		cpi r16, 0
		breq lightLED
			clr r16; if !=0, off
			rjmp saveLED
		lightLED:
			ser r16
		saveLED:
			st y, r16
			out PORTC, r16
			rjmp TIMER_END
	LEDTickCounter_INC:
		inc r16
		st y, r16

	TIMER_END:
		pop r16
		pop yl
		pop yh
		reti

; functions
.def digit2=r17
.def digit1=r18
display_2bits_number:
	push digit2
	push digit1

	clr digit2
	clr digit1
	ten_2bits:						;Subtract 10 until temp1 is less than 10
		cpi r16, 10
		brlo one_2bits
		subi r16, 10
		inc digit2
		rjmp ten_2bits
	one_2bits:						;The remainder is put to digit1
		mov digit1, r16

	ldi r16, 48; ASCII conversion
	add digit2, r16
	add digit1, r16
	do_lcd_data digit2
	do_lcd_data digit1

	pop digit1
	pop digit2
	ret

.def digit3=r17
.def digit2=r18
.def digit1=r19
display_3bits_number:
	push digit3
	push digit2
	push digit1

	clr digit3
	clr digit2
	clr digit1
	hundred_3bits:					;Subtract 100 until temp1 is less than 100
	cpi r16, 100
		brlo ten_3bits
		subi r16, 100
		inc digit3
		rjmp hundred_3bits
	ten_3bits:						;Subtract 10 until temp1 is less than 10
		cpi r16, 10
		brlo one_3bits
		subi r16, 10
		inc digit2
		rjmp ten_3bits
	one_3bits:						;The remainder is put to digit1
		mov digit1, r16

	ldi r16, 48; ASCII conversion
	add digit3, r16
	add digit2, r16
	add digit1, r16
	do_lcd_data digit3
	do_lcd_data digit2
	do_lcd_data digit1

	pop digit1
	pop digit2
	pop digit3
	ret

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

.equ DELAY_250MS = (F_CPU / 4 - 7 - 2 * 7 * 2) / 9; 1s
.def i0=r16
.def i1=r24
.def i2=r25
.def count0=r17
.def count1=r18
.def count2=r19
.def o=r20; zero

sleep_250ms:
	push r16; 2 cycles
	push r24; 2
	push r25; 2
	push r17; 2
	push r18; 2
	push r19; 2
	push r20; 2

	ldi count0, byte3(DELAY_250MS);1
	ldi count1, byte2(DELAY_250MS);1
	ldi count2, byte1(DELAY_250MS);1
	clr o;1
	clr i0;1
	clr i1;1
	clr i2;1
	delayloop_250ms:
		cp i2, count2;1
		cpc i1, count1;1
		cpc i0, count0;1
		brsh delayloop_250ms_done;1 / 2
		adiw i1:i2, 1;2
		adc i0, o;1
		rjmp delayloop_250ms;2

	delayloop_250ms_done:
	pop r20; 2 cycles
	pop r19; 2
	pop r18; 2
	pop r17; 2
	pop r25; 2
	pop r24; 2
	pop r16; 2
	ret

sleep_1s:
	rcall sleep_250ms
	rcall sleep_250ms
	rcall sleep_250ms
	rcall sleep_250ms
	ret

; a 64*64 8-bit signed array in range [-10, 127]
MOUNTAIN: 
	.db 100,  93,  73,  75,  1,  69,  19,  60,  116,  108,  16,  116,  65,  95,  116,  98,  124,  -4,  49,  22,  31,  104,  90,  70,  77,  23,  23,  56,  83,  108,  94,  19,  29,  0,  -9,  122,  -1,  -10,  114,  -8,  124,  90,  8,  69,  20,  13,  10,  70,  -7,  98,  112,  108,  87,  30,  61,  109,  106,  103,  3,  55,  4,  -7,  104,  -1,  39,  113,  110,  97,  100,  87,  92, 127,  88,  108,  115,  35,  28,  76,  117,  113,  41,  44,  94,  5,  34,  79,  0,  96,  28,  99,  59,  52,  43,  38,  108,  -2,  100,  30,  117,  74,  109,  68,  0,  94,  64,  12,  25,  52,  107,  125,  22,  97,  67,  117,  32,  5,  39,  -1,  94,  99,  1,  90,  95,  51,  122,  -3,  21,  98,  -2,  28,  101,  33,  54,  104,  39,  73,  14,  113,  66,  0,  127,  78,  95,  120,  -1,  40,  17,  -2,  80,  -8,  48,  73,  69,  115,  98,  54,  94,  69,  17,  53,  44,  31,  71,  95,  73,  115,  11,  51,  85,  117,  104,  80,  74,  52,  92,  55,  49,  37,  76,  108,  79,  14,  39,  66,  126,  26,  69,  39,  49,  80,  67,  2,  86,  37,  10,  96,  61,  95,  34,  80,  99,  96,  123,  48,  86,  61,  119,  65,  69,  86,  91,  92,  22,  115,  66, 108,  102,  126,  95,  3,  43,  49,  114,  39,  11,  126,  -4,  78,  26,  91,  118,  29,  45,  13,  76,  104,  110,  31,  93,  57,  116,  100,  83,  116,  5,  1,  68,  127,  -4,  68,84,  80,  119,  107,  92,  89,  63,  41,  30,  32,  23,  71,  50,  89,  37,  71,  48,  121,  59,  97,  19,  22,  93,  126,  36,  73,  -10,  46,  36,  -9,  39,  61,  16,  -8,  6,  89,  64,  81,  101,  117,  113,  92,  79,  7,  69,  96,  121,  8,  66,  37,  95,  77,  -2,  125,  35,  87,  34,  92,  0,  90,  88,  106,  78,  125,  27,  -2,  45,  77,  122,  37,  -6,  -7,  72,  26,  24,  36,  26,  73,  104,  41,  92,  25,  103,  23,  66,  104,  115,  92,  82,  6,  55,  121,  78,  111,  16,  60,  81,  60,  88,  76,  50,  57,  54,  22,  22,  62,  48,  65,  47,  122,  84,  -5,  113,  57,  77,  29,  63,  33,  5,  105,  92,  43,  79,  -5,  23,  62,  95,  29,  105,  10,  123,  114,  116,  107,  -9,  93,  81,  88,  78,  18,  80,  12,  43,  16, 20,  13,  81,  39,  30,  -2,  112,  9,  81,  101,  67,  48,  27,  38,  61,  44,  79,  4,  12,  60,  37,  28,  21,  112,  44,  113,  7,  62,  65,  117,  32,  16,  -5,  59,  53,  91,  127,  2,  77,  127,  123,  -5,  126,  114,  59,  67,  16,  31,  118,  62,  8,  123,  57,  8,  116,  26,  70,  105,  30,  -2,  101,  99,  120,  123,  27,  69,  94,  17,  34,  13,  -7,  114,  -8,  73,  -1,  56,  21,  53,  14,  78,  35,  42,  12,  -2,  -3,  29,  -4,  120,  44,  107,  -9,  71,  5,  67,  74,  11,  6,  101,  1,  126,  39,  42,  63,  90,  78,  22,  62,  92,  96,  91,  67,  79,  20,  109,  55,  124,  53,  89,  10,  102,  66,  62,  92,  -6,  32,  127,  8,  104,  21,  41,  116,  5,  100,  26,  106,  125,  79,  57,  103,  61,  99,  51,  55,  112,  -3,  93,  59,  118,  62,  76,  78,  -3,  70,  105,  33,  13,  84,  19,  75,  63,  82,  -5,  100,  38,  37,  39,  87,  94,  22,  82,  56,  9,  99,  23,  125,  26,  14,  98,  56,  58, 30,  50,  4,  -6,  113,  4,  15,  41,  79,  111,  127,  114,  86,  44,  70,  -2,  99,  79,  35,  81,  47,  12,  105,  76,  16,  40,  7,  75,  100,  26,  63,  54,  113,  13,  117,  66, -9,  114,  70,  108,  50,  -3,  72,  74,  20,  8,  -8,  51,  75,  3,  117,  -10,  -2,  15,  76,  29,  51,  63,  43,  57,  -3,  89,  22,  11,  73,  27,  2,  61,  90,  62,  39,  48,  0, 68,  14,  7,  85,  65,  93,  127,  25,  -4,  97,  78,  25,  108,  10,  83,  108,  18,  34,  38,  94,  95,  4,  74,  53,  81,  17,  6,  43,  -1,  27,  6,  7,  100,  6,  63,  81,  56,  72,  80,  41,  107,  -8,  49,  112,  117,  -9,  64,  13,  -8,  71,  -2,  6,  1,  18,  84,  -8,  52,  109,  98,  118,  25,  60,  102,  -6,  108,  123,  64,  88,  48,  -2,  94,  44,  48,87,  28,  98,  47,  6,  13,  106,  14,  31,  63,  6,  42,  12,  33,  5,  60,  123,  36,  49,  57,  69,  16,  49,  99,  118,  124,  31,  58,  52,  10,  75,  11,  80,  34,  73,  103,  5, 18,  44,  97,  83,  125,  85,  4,  50,  43,  5,  86,  63,  50,  51,  7,  23,  100,  16,  8,  49,  92,  72,  19,  113,  -2,  38,  7,  109,  -7,  46,  8,  22,  69,  68,  -1,  74,  108,32,  -10,  -3,  105,  114,  22,  28,  3,  -6,  26,  15,  113,  116,  71,  20,  35,  113,  47,  17,  42,  24,  -4,  45,  104,  12,  110,  7,  96,  64,  8,  75,  79,  88,  118,  15,  93, 108,  107,  124,  77,  120,  103,  67,  47,  70,  63,  58,  29,  94,  -2,  106,  69,  82,  78,  115,  19,  -2,  55,  3,  82,  -3,  71,  19,  47,  41,  117,  58,  87,  37,  19,  117,  20,  25,  78,  72,  -8,  122,  83,  13,  74,  42,  89,  95,  113,  66,  11,  9,  71,  90,  -6,  20,  23,  0,  48,  3,  10,  93,  98,  65,  107,  53,  3,  48,  45,  15,  38,  25,  18,  48,  59,  93,  12,  40,  44,  47,  108,  91,  50,  10,  61,  94,  73,  78,  83,  56,  94,  93,  110,  14,  -3,  109,  71,  108,  -5,  73,  121,  58,  22,  34,  12,  55,  86,  83,  107,  13,  37,  53,  22,  127,  39,  78,  39,  7,  116,  55,  5,  55,  67,  95,  13,  94,  5,  82,  111,  95,  31,  98,  98,  72,  95,  92,  10,  112,  66,  68,  63,  122,  65,  63,  102,  28,  81,  93,  104,  43,  82,  -6,  46,  2,  43,  16,  115,  85,  5,  109,  93,  34,  77,  65,  -7,  72,  90,  -3,  19,  63,  126,  97,  63,  79,  -10,  124,  18,  58,  73,  94,  99,  85, 127,  47,  0,  38,  83,  11,  105,  126,  28,  70,  42,  16,  33,  126,  0,  51,  103,  61,  59,  -3,  25,  62,  64,  62,  31,  -1,  37,  83,  78,  24,  93,  96,  46,  23,  87,  114,38,  86,  49,  96,  91,  119,  68,  96,  102,  125,  39,  -3,  38,  42,  101,  -9,  65,  86,  68,  83,  43,  60,  109,  85,  63,  12,  -5,  89,  80,  11,  125,  74,  15,  123,  54,  79,  2,  53,  22,  11,  94,  107,  78,  17,  -5,  35,  113,  122,  127,  11,  67,  64,  58,  26,  26,  91,  114,  11,  42,  105,  -4,  101,  98,  40,  7,  69,  36,  100,  82,  18,  54,  -4,  2,  74,  115,  79,  108,  -1,  28,  98,  98,  59,  -4,  64,  109,  16,  12,  125,  47,  51,  28,  117,  126,  55,  39,  120,  13,  73,  36,  86,  13,  49,  13,  4,  2,  58,  100,  -8,  14,  10,  22,  32,  2,  22,  -6,  79,  78,  35,  120,  49,  67,  83,  105,  98,  81,  90,  2,  83,  82,  46,  60,  84,  127,  5,  56,  15,  37,  115,  119,  96,  123,  51,  6,  92,14,  83,  55,  3,  25,  30,  -2,  127,  110,  115,  46,  68,  104,  104,  85,  47,  108,  107,  117,  121,  51,  17,  81,  56,  127,  59,  73,  63,  2,  42,  47,  72,  41,  57,  109,  23,  107,  123,  29,  51,  15,  103,  123,  36,  20,  54,  -1,  81,  49,  4,  104,  125,  90,  37,  81,  45,  -7,  29,  69,  29,  29,  108,  31,  108,  96,  35,  13,  8,  125,  96,  20, 9,  114,  6,  93,  58,  82,  23,  20,  107,  100,  94,  88,  -3,  31,  96,  96,  121,  35,  70,  53,  22,  7,  45,  51,  74,  3,  76,  112,  117,  13,  38,  0,  63,  -8,  65,  70,  117,  -3,  -2,  0,  3,  92,  83,  87,  26,  2,  77,  89,  94,  8,  13,  70,  -8,  33,  101,  31,  39,  114,  21,  103,  43,  81,  67,  31,  9,  17,  93,  79,  -4,  124,  67,  2,  80,  101,  106,  125,  -8,  65,  115,  63,  29,  96,  44,  95,  7,  46,  119,  70,  41,  93,  65,  -1,  4,  122,  58,  48,  58,  94,  45,  119,  87,  27,  -4,  73,  -10,  55,  67,  27,  -5,  61,  113,  37,  -5,  118,  120,  118,  127,  123,  114,  77,  8,  117,  41,  69,  -6,  34,  90,  -1,  39,  22,  56,  51,  34,  5,  10,  1,  -1,  68,  39,  94,  88,  78,  97,  51,  55,  74,  18,  57,  20,  86,  9,  125,  51,  50,  98,  122,  14,  -5,  22,  22,  120,  -3,  31,  114,  38,  -5,  65,  78,  112,  74,  30,  18,  77,  51,  53,  53,  51,  50,  93,  55,  46,  64, 47,  126,  50,  73,  10,  -6,  87,  3,  109,  103,  91,  75,  6,  82,  2,  29,  97,  27,  10,  13,  49,  93,  11,  19,  73,  69,  23,  104,  23,  87,  38,  78,  21,  103,  125,  58,  78,  86,  27,  -9,  99,  -5,  70,  30,  62,  61,  -6,  122,  111,  33,  97,  95,  106,  93,  36,  44,  -1,  -5,  103,  99,  3,  94,  5,  115,  60,  19,  54,  15,  62,  106,  54,  91,  20,  63,  12,  24,  126,  -4,  76,  20,  46,  -6,  52,  32,  60,  49,  9,  3,  74,  43,  68,  17,  8,  8,  124,  37,  -2,  66,  -2,  54,  77,  92,  47,  39,  115,  124,  19,  114,  108,15,  101,  41,  115,  121,  38,  114,  108,  -6,  22,  38,  39,  106,  102,  62,  79,  -7,  -4,  27,  34,  1,  78,  88,  7,  -10,  124,  107,  117,  62,  125,  100,  -8,  42,  10,  16, 80,  126,  73,  2,  124,  123,  123,  86,  127,  -10,  111,  73,  29,  38,  -3,  -2,  111,  110,  73,  -3,  6,  66,  29,  72,  30,  112,  97,  123,  38,  99,  66,  101,  -6,  117,  33,  23,  47,  80,  107,  69,  9,  111,  74,  38,  88,  -4,  62,  -3,  64,  81,  49,  98,  89,  23,  100,  108,  75,  97,  28,  39,  0,  117,  115,  35,  123,  110,  101,  125,  113,  21, 101,  28,  85,  46,  10,  65,  20,  57,  87,  108,  98,  103,  42,  75,  29,  -5,  106,  42,  121,  -10,  47,  37,  31,  83,  96,  92,  85,  48,  91,  101,  -4,  123,  65,  19,  41,  47,  97,  116,  64,  110,  -8,  34,  39,  92,  103,  60,  31,  77,  119,  84,  53,  102,  0,  92,  26,  60,  88,  -1,  15,  119,  73,  27,  23,  117,  79,  -5,  3,  50,  117,  31,  26,39,  50,  59,  89,  12,  33,  23,  62,  18,  -2,  111,  19,  32,  64,  86,  122,  63,  49,  127,  51,  65,  35,  118,  43,  55,  118,  123,  50,  82,  47,  18,  121,  70,  4,  49,  125,  96,  115,  71,  114,  116,  20,  19,  16,  66,  33,  -3,  11,  28,  -9,  32,  107,  -10,  11,  20,  98,  54,  73,  16,  95,  87,  22,  85,  107,  45,  81,  88,  118,  35,  55,  83,  11,  72,  10,  116,  25,  2,  75,  11,  123,  -3,  49,  83,  49,  82,  35,  93,  49,  123,  81,  94,  27,  92,  119,  119,  76,  86,  79,  96,  78,  122,  1,  96,  71,  120,  71,  96,  -7,  33,  2,  -8,  105,  82,  32,  85,  23,  0,  117,  25,  -3,  109,  66,  31,  72,  21,  98,  123,  58,  109,  53,  8,  3,  103,  93,  63,  120,  114,  114,  34,  52,  46,  119,  23,111,  124,  87,  43,  64,  108,  61,  123,  35,  104,  72,  0,  123,  77,  -4,  1,  2,  59,  65,  102,  42,  59,  21,  -8,  56,  63,  126,  -10,  94,  76,  65,  80,  91,  20,  20,  20, 127,  41,  96,  65,  116,  51,  116,  79,  85,  -1,  -8,  108,  102,  105,  16,  75,  88,  90,  108,  100,  -1,  104,  36,  44,  93,  121,  60,  2,  8,  50,  105,  83,  94,  -5,  120, 120,  53,  87,  29,  97,  101,  72,  66,  28,  5,  -10,  86,  69,  102,  34,  98,  72,  50,  28,  105,  75,  -3,  45,  55,  115,  73,  34,  31,  70,  106,  85,  41,  42,  61,  55,  109,  -2,  63,  33,  46,  69,  106,  119,  35,  17,  -5,  72,  108,  98,  99,  74,  38,  50,  96,  98,  10,  76,  53,  33,  73,  76,  115,  63,  65,  59,  6,  72,  92,  17,  127,  59,  88,  8,  54,  4,  43,  26,  82,  77,  99,  11,  21,  59,  25,  71,  14,  89,  92,  -2,  -8,  112,  35,  125,  6,  43,  12,  72,  126,  37,  12,  55,  10,  2,  121,  52,  55,  1,  81,  8,81,  5,  49,  -6,  123,  96,  19,  -3,  47,  19,  60,  83,  88,  102,  93,  55,  81,  32,  36,  1,  115,  26,  103,  3,  32,  36,  64,  115,  105,  36,  61,  115,  113,  64,  66,  57,78,  -1,  63,  108,  98,  3,  48,  83,  53,  47,  74,  71,  43,  88,  -8,  86,  110,  78,  14,  42,  105,  125,  3,  73,  0,  17,  59,  -2,  72,  37,  81,  -1,  85,  28,  59,  7,  43,97,  15,  73,  102,  70,  -8,  97,  51,  71,  17,  63,  -9,  55,  108,  68,  39,  11,  89,  8,  63,  91,  125,  36,  58,  55,  39,  93,  117,  15,  -10,  41,  64,  82,  85,  20,  35,  7,  120,  67,  29,  27,  34,  37,  11,  49,  31,  27,  28,  39,  70,  104,  56,  65,  90,  110,  123,  67,  68,  -9,  42,  29,  105,  109,  91,  52,  71,  117,  -6,  49,  -3,  4,  89,  85,  64,  126,  -3,  1,  34,  21,  21,  1,  99,  48,  71,  86,  16,  52,  92,  4,  67,  98,  103,  123,  99,  -3,  112,  49,  25,  -4,  63,  15,  47,  -6,  88,  24,  35,  101,  8,  17,61,  92,  72,  91,  64,  55,  57,  84,  79,  5,  107,  7,  45,  108,  58,  -4,  29,  77,  86,  54,  55,  15,  96,  32,  63,  68,  51,  116,  113,  96,  57,  107,  97,  13,  9,  123,  -3,  -8,  49,  62,  33,  71,  63,  110,  25,  8,  29,  85,  20,  71,  119,  61,  28,  7,  53,  32,  -6,  52,  120,  53,  21,  16,  100,  94,  47,  104,  8,  10,  15,  107,  58,  48,  67, 57,  79,  94,  34,  49,  20,  45,  5,  88,  116,  73,  54,  70,  41,  68,  75,  49,  30,  113,  38,  114,  63,  23,  68,  81,  61,  41,  52,  84,  15,  43,  127,  106,  54,  22,  76,10,  54,  80,  109,  84,  49,  74,  -1,  31,  15,  116,  106,  4,  111,  76,  -5,  45,  77,  55,  35,  35,  92,  110,  50,  9,  89,  80,  59,  82,  -10,  67,  15,  6,  73,  -4,  16,  97,  14,  29,  74,  37,  45,  8,  100,  89,  0,  21,  118,  104,  115,  6,  58,  -7,  122,  29,  -3,  124,  119,  77,  27,  49,  92,  28,  -8,  115,  65,  21,  108,  33,  98,  111,  14,72,  -3,  -6,  1,  22,  103,  65,  115,  65,  94,  98,  41,  12,  68,  47,  108,  30,  93,  95,  106,  114,  89,  98,  82,  100,  32,  125,  72,  98,  43,  10,  38,  5,  -1,  63,  11,112,  69,  82,  55,  89,  50,  -6,  100,  103,  58,  99,  15,  109,  94,  98,  15,  26,  12,  73,  83,  70,  -2,  60,  19,  112,  7,  48,  47,  41,  127,  113,  71,  71,  29,  60,  39, 78,  14,  20,  127,  84,  -4,  12,  9,  97,  64,  2,  5,  69,  82,  60,  123,  0,  94,  117,  68,  63,  85,  100,  81,  40,  20,  27,  4,  69,  48,  53,  45,  125,  97,  18,  32,  109,  59,  122,  96,  76,  5,  79,  70,  88,  122,  84,  84,  81,  109,  117,  64,  50,  91,  68,  87,  65,  -3,  68,  23,  69,  110,  86,  39,  17,  50,  35,  46,  39,  88,  18,  13,  57, 80,  72,  71,  117,  50,  80,  83,  56,  102,  107,  7,  52,  70,  120,  65,  78,  82,  -3,  50,  114,  64,  103,  39,  61,  -10,  80,  123,  -2,  31,  15,  104,  47,  103,  -7,  70,23,  62,  112,  43,  25,  -5,  116,  72,  116,  80,  80,  41,  -6,  108,  83,  98,  94,  4,  103,  4,  56,  47,  64,  -7,  51,  38,  -2,  63,  -9,  71,  54,  22,  72,  54,  12,  84,  17,  39,  65,  81,  65,  56,  116,  -4,  11,  27,  102,  41,  11,  55,  98,  96,  60,  92,  84,  98,  96,  34,  34,  3,  111,  24,  19,  33,  63,  33,  49,  125,  12,  74,  66,  -1,  62, 39,  43,  71,  74,  3,  127,  109,  54,  91,  38,  12,  83,  109,  93,  125,  80,  50,  117,  114,  34,  3,  66,  104,  56,  76,  121,  33,  -10,  18,  25,  117,  103,  71,  117,  116,  15,  13,  102,  35,  22,  36,  50,  64,  20,  18,  86,  64,  101,  -7,  44,  85,  103,  109,  10,  105,  98,  125,  127,  110,  -2,  29,  106,  60,  39,  59,  33,  14,  1,  15,  73,117,  86,  106,  94,  93,  40,  33,  109,  126,  -4,  87,  125,  90,  95,  119,  4,  93,  70,  7,  23,  123,  89,  83,  79,  80,  20,  81,  72,  123,  84,  91,  31,  78,  117,  123,  26,  6,  121,  11,  65,  87,  38,  120,  99,  17,  21,  85,  103,  65,  70,  6,  115,  103,  102,  118,  27,  65,  10,  64,  93,  5,  73,  48,  86,  124,  4,  106,  107,  6,  47,  -4,  3,  36,  125,  54,  83,  29,  121,  90,  20,  25,  97,  105,  33,  62,  122,  77,  -2,  116,  17,  110,  54,  103,  35,  80,  59,  72,  89,  126,  14,  117,  20,  110,  103,  12,  33,  89,  20,  69,  40,  67,  23,  126,  92,  60,  13,  97,  122,  92,  -1,  118,  4,  3,  14,  7,  37,  95,  114,  58,  98,  -2,  90,  -2,  37,  7,  18,  12,  83,  88,  29,  -3,  1,  60,  69,  97,  16,  16,  12,  27,  -2,  47,  81,  -6,  59,  50,  -8,  94,  73,  69,  88,  29,  52,  54,  58,  3,  21,  112,  0,  73,  20,  13,  94,  92,  20,  89,  88,  91,  92,  74,  57,  92, 45,  123,  64,  110,  125,  104,  51,  109,  98,  10,  117,  19,  83,  2,  54,  122,  82,  33,  -4,  3,  13,  60,  13,  35,  53,  104,  -6,  2,  127,  19,  99,  100,  80,  40,  32,  12,  12,  8,  37,  -2,  36,  50,  123,  121,  74,  82,  36,  -4,  29,  37,  92,  -1,  8,  65,  24,  91,  101,  -6,  127,  17,  87,  -10,  65,  26,  102,  29,  21,  56,  67,  111,  7,  80,  113,  -9,  100,  127,  105,  13,  47,  25,  2,  73,  72,  94,  127,  113,  0,  28,  49,  29,  35,  91,  35,  63,  51,  14,  78,  89,  37,  65,  127,  42,  74,  103,  59,  98,  74,  47,  -7,  48,  7,  58,  -6,  39,  33,  109,  42,  18,  123,  68,  80,  6,  49,  3,  76,  22,  23,  19,  -1,  12,  70,  28,  68,  -5,  32,  81,  121,  119,  55,  46,  6,  -6,  30,  26,  120,  111,  105,  88,  52,  22,  5,  122,  116,  60,  31,  120,  -8,  87,  40,  37,  55,  1,  25,  37,  -4,  18,  27,  37,  -3,  104,  3,  67,  43,  25,  54,  117,  1,  84,  88,  113,  66,  102,  56,  15,  32,  94,  53,  47,  111,  0,  118,  39,  99,  33,  47,  61,  76,  103,  46,  31,  54,  120,  -7,  52,  113,  9,  113,  22,  -9,  64,  74,  65,  126,  -3,  39,  100,-1,  94,  71,  59,  87,  16,  45,  38,  10,  63,  7,  56,  7,  31,  127,  11,  69,  68,  12,  59,  54,  122,  44,  120,  9,  25,  -3,  93,  39,  21,  87,  3,  79,  12,  112,  15,  -2,41,  104,  8,  24,  0,  25,  -4,  35,  28,  103,  2,  95,  91,  57,  44,  25,  46,  93,  53,  124,  34,  40,  18,  55,  17,  12,  121,  -1,  35,  85,  77,  68,  40,  95,  32,  2,  97,42,  50,  41,  99,  117,  126,  54,  77,  49,  68,  48,  40,  76,  88,  2,  76,  111,  97,  64,  120,  110,  -2,  33,  4,  78,  62,  94,  2,  -2,  76,  114,  123,  114,  100,  -4,  69, 33,  57,  95,  -3,  62,  59,  -3,  93,  33,  11,  66,  62,  16,  127,  9,  104,  96,  107,  -7,  34,  60,  30,  78,  89,  -3,  -8,  111,  100,  -8,  75,  30,  50,  32,  12,  41,  -6,-2,  54,  20,  -4,  36,  18,  36,  107,  71,  -10,  122,  125,  101,  -8,  75,  90,  21,  34,  61,  93,  10,  50,  121,  7,  118,  121,  7,  53,  121,  14,  77,  3,  -1,  123,  61,  99,  38,  -2,  4,  74,  75,  45,  65,  22,  1,  33,  4,  49,  60,  25,  14,  24,  44,  33,  11,  78,  21,  -3,  7,  107,  0,  -4,  68,  116,  83,  32,  98,  44,  123,  -2,  54,  38,  72,34,  -9,  5,  51,  109,  81,  13,  102,  79,  25,  53,  99,  64,  127,  124,  122,  77,  101,  36,  45,  62,  99,  84,  54,  -5,  6,  14,  0,  56,  27,  -4,  14,  6,  -3,  70,  108,  125,  57,  92,  -2,  10,  20,  84,  122,  60,  12,  19,  80,  63,  7,  121,  1,  7,  88,  -5,  65,  23,  35,  16,  125,  51,  19,  42,  65,  31,  59,  71,  75,  -6,  60,  59,  106,  12,61,  65,  41,  16,  99,  94,  68,  5,  93,  22,  108,  122,  54,  25,  95,  66,  127,  -4,  6,  14,  3,  49,  78,  58,  52,  58,  5,  16,  103,  -3,  87,  60,  116,  108,  76,  45,  69,  109,  83,  71,  48,  94,  119,  113,  90,  27,  66,  85,  36,  80,  106,  26,  48,  123,  15,  117,  52,  74,  86,  107,  4,  45,  54,  38,  63,  107,  53,  76,  108,  91,  -5,  -2,31,  5,  74,  0,  102,  51,  69,  2,  76,  56,  42,  117,  32,  95,  59,  71,  4,  44,  30,  -8,  11,  91,  82,  107,  27,  62,  125,  124,  30,  126,  106,  108,  54,  7,  120,  67,  25,  50,  3,  14,  108,  64,  70,  8,  106,  49,  0,  24,  75,  15,  3,  -4,  117,  36,  31,  21,  2,  -3,  33,  111,  14,  67,  -9,  17,  -3,  106,  -1,  42,  24,  54,  -4,  32,  -5,  14,  -1,  54,  0,  63,  17,  95,  57,  83,  102,  5,  29,  15,  25,  37,  14,  97,  8,  119,  81,  71,  125,  47,  123,  46,  91,  102,  -4,  46,  109,  78,  57,  34,  53,  3,  20,  13, 19,  1,  19,  59,  28,  67,  65,  26,  115,  30,  59,  109,  48,  85,  14,  -2,  106,  102,  81,  56,  92,  37,  79,  104,  33,  -8,  84,  76,  93,  25,  56,  13,  56,  29,  82,  82,61,  100,  31,  6,  123,  115,  75,  47,  23,  12,  120,  102,  92,  51,  80,  56,  103,  32,  100,  67,  65,  112,  75,  65,  85,  50,  103,  64,  72,  40,  -1,  0,  8,  56,  83,  20, 97,  119,  -3,  107,  47,  92,  69,  102,  46,  94,  5,  122,  111,  25,  113,  32,  54,  92,  44,  73,  88,  -5,  102,  60,  97,  33,  104,  4,  53,  67,  111,  92,  4,  125,  50,  96,  56,  -10,  87,  55,  53,  62,  29,  24,  109,  32,  45,  125,  84,  104,  82,  -8,  111,  3,  8,  111,  49,  42,  83,  82,  122,  71,  47,  17,  -2,  20,  -8,  10,  13,  46,  97,  21,  64,  69,  56,  79,  16,  124,  120,  23,  20,  42,  34,  102,  114,  -9,  -10,  -10,  2,  51,  116,  50,  45,  2,  104,  95,  46,  123,  39,  106,  39,  60,  62,  7,  42,  0,  110,7,  -1,  0,  62,  84,  95,  116,  86,  3,  21,  4,  57,  21,  39,  5,  101,  89,  123,  -6,  51,  80,  123,  14,  16,  7,  30,  23,  29,  100,  21,  47,  42,  116,  69,  20,  51,  110, 46,  86,  -3,  33,  97,  41,  94,  62,  90,  -2,  12,  81,  119,  22,  23,  14,  40,  99,  124,  -4,  125,  65,  -7,  46,  89,  101,  112,  114,  0,  16,  10,  78,  47,  94,  99,  3,50,  111,  16,  38,  89,  60,  7,  100,  8,  53,  105,  73,  109,  120,  26,  75,  9,  116,  69,  18,  101,  101,  20,  101,  -5,  111,  49,  34,  46,  0,  82,  23,  126,  -3,  120,  2,  122,  59,  34,  110,  115,  23,  64,  85,  125,  120,  119,  27,  16,  104,  2,  63,  79,  100,  23,  29,  118,  117,  45,  71,  -9,  9,  57,  -1,  24,  39,  9,  54,  13,  36,  106,79,  96,  48,  120,  -3,  115,  12,  121,  119,  -8,  25,  82,  45,  14,  104,  117,  93,  -2,  11,  103,  6,  -2,  84,  42,  52,  67,  -4,  81,  58,  -4,  124,  34,  103,  103,  46,  119,  65,  21,  69,  87,  3,  25,  121,  39,  19,  127,  51,  105,  17,  -8,  56,  117,  61,  26,  70,  75,  27,  38,  32,  8,  18,  -10,  50,  -2,  21,  65,  47,  84,  -3,  41,  30,  106,  6,  35,  89,  89,  125,  71,  52,  -5,  55,  13,  55,  108,  95,  48,  -7,  55,  120,  25,  27,  -8,  59,  116,  32,  101,  95,  24,  23,  -8,  -4,  87,  98,  10,  114,  34,  -1,  75,  -10,  102,  29,  59,  24,  5,  46,  6,  115,  52,  0,  33,  36,  72,  110,  59,  95,  37,  73,  9,  34,  95,  127,  47,  89,  -7,  79,  63,  82,  97,  60,  31,  122,  34,  26,  84, -8,  76,  113,  87,  91,  2,  119,  41,  -1,  33,  11,  1,  37,  68,  38,  114,  49,  5,  51,  7
