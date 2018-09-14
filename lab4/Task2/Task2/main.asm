;
; Task2.asm
;
; Created: 10/3/2017 7:13:33 PM
; Author : Ethan
;

; MOTOR
; POT ~ MOT
; +5V ~ OpE
; OpO ~ TDX2 (INT2)

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

.macro lcd_set
	sbi LCD_CTRL_PORT, @0
.endmacro

.macro lcd_clr
	cbi LCD_CTRL_PORT, @0
.endmacro

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

.def motorHoleCounter=r17
.def motorRevolutionCounter=r18
.def timerCounter=r19

; 3 routines: RESET~main; INT2addr~motor OpO interrupt; OVF0addr~timer interrupt
.org 0x0000; main
jmp RESET

.org INT2addr; connected to TDX2, Motor's Opo
jmp EXT_INT2

.org OVF0addr; Timer Overflow Address
jmp Timer0OVF

RESET:
	; set OpO interrupt
	ldi r16, (1<<ISC21); set external interrupt 2 for falling-edge
	sts EICRA, r16; external interrupt 0~3 are controlled by A
	in r16, EIMSK; enable INT0
	ori r16, (1<<INT2)
	out EIMSK, r16

	; set timer interrupt
	ldi r16, 0x00; Set-up Timer0 Overflow, 8 bits counter
	out TCCR0A, r16
	ldi r16, 0x04; prescaling value=256, 256*256/16Mhz=4ms
	out TCCR0B, r16
	ldi r16, 1<<TOIE0; enable the Overflow Interrupt
	sts TIMSK0, r16; T/C0 interrupt enable

	sei; Global Interrupt Enable

	; LCD
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

	; others
	clr motorHoleCounter
	clr motorRevolutionCounter
	clr timerCounter

	ser r16
	out DDRC, r16

	loop:; keep running
		rjmp loop

EXT_INT2:
	inc motorHoleCounter
	cpi motorHoleCounter, 4
	breq incRevolutionCounter
	rjmp endOpO
	
	incRevolutionCounter:
		clr motorHoleCounter
		inc motorRevolutionCounter

	endOpO:
		reti

.def digit3=r20
.def digit2=r21
.def digit1=r22

Timer0OVF:
	in r16, SREG
	push r16
	push digit3
	push digit2
	push digit1
	push r0
	push r1

	clr digit3
	clr digit2
	clr digit1

	inc timerCounter
	cpi timerCounter, 250; 1s / 4ms

	brne endTimer
	clr timerCounter
	
	; r/sec
	out PORTC, motorRevolutionCounter
	
	; display revolutions/second
	do_lcd_command 0b00000001 ; Clear display

	hundred:					;Subtract 100 until temp1 is less than 100
		cpi motorRevolutionCounter, 100
		brlo ten
		subi motorRevolutionCounter, 100
		inc digit3
		rjmp hundred
	ten:						;Subtract 10 until temp1 is less than 10
		cpi motorRevolutionCounter, 10
		brlo one
		subi motorRevolutionCounter, 10
		inc digit2
		rjmp ten
	one:						;The remainder is put to digit1
		mov digit1, motorRevolutionCounter

	clr motorRevolutionCounter

	ldi r16, 48; ASCII conversion
	add digit3, r16
	add digit2, r16
	add digit1, r16
	do_lcd_data digit3
	do_lcd_data digit2
	do_lcd_data digit1

	endTimer:
		pop r1
		pop r0
		pop digit1
		pop digit2
		pop digit3
		pop r16
		out SREG, r16
		reti

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