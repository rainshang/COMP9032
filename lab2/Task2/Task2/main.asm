`````````````````4;
; Task2.asm
;
; Created: 8/25/2017 3:35:58 PM
; Author : Ethan
;
;int main(void) {
;	char s[] = "12345";
;	int number; number = atoi(s); return 0;
;}

;int atoi(char *a) {
;	char i;
;	char c;
;	int n;
;	n = 0;
;	c = *a;
;	for (i = 1; ((c >= '0') && (c <= '9') && (n < 65536)); i++) {
;		n = 10 * n + (c -'0');
;		c = *(a+i);
;	}
;	return n;
;}

;aH, aL, 0x0000
.macro cpwi
	cpi @0, high(@2)
	brne endCpw;if there is result for high bits comparision, return
	cpi @1, low(@2);high bit equal, then compare low bits
	endCpw:
.endmacro

;addw aH, aL, b
;result is aH:aL
.macro addw
	add @1, @2
	brcc endAddw;no carry, stop
	inc @0;add with carry
	endAddw:
.endmacro

;a1:a0 * b0
;-->
;		 a1:a0
;X			b0
;-------------
;		 a0*b0
;	  a1*b0
;-------------
;	  c2:c1:c0

;mulw a1, a0, b0
;using x, result is x
.macro mulw
	mul @1, @2;a0*b0
	mov xh, r1
	mov xl, r0;x=a0*b0
	mul @0, @2;a1*b0
	add xh, r0;then xh = c1 = high(a0*b0) + low(a1*b0), so x = c1:c0, c2 is abandoned
.endmacro

;mult a1, a0 with 10
;result is in a1:a0
.macro mulw10
	mulw @0, @1, r16
	mov r18, xh
	mov r19, xl
.endmacro

.dseg
number: .byte 2

.cseg
strings: .db "25", 0

.equ _0='0'
.equ _10='9'+1
.equ max_int=65535
.def ten=r16

main:
    ldi r20, high(strings<<1)
	ldi r21, low(strings<<1);r20:r21 = *s
	rcall atoi
	ldi xh, high(number<<1)
	ldi xl, low(number<<1)
	st x+, r18
	st x, r19; save n in atoi
end:
	rjmp end

atoi:
	;prologue

	push yl;r29:r28 will be used as the frame pointer
	push yh;save r29:r28 in the stack
	push r16;10
	push r17;char c
	push zl
	push zh
	
	in yl, spl
	in yh, sph;initialize the stack frame pointer value
	sbiw y, 4;reserve space for local variables and parameters; (r16, r17, char* is a 2-byte address)
	out sph, yh
	out spl, yl;update the stack pointer to point to the new stack top

	;pass actual parameters
	std y+1, r20
	std y+2, r21;pass *s to *a
	;end prologue

	;function body
	ldi r16, 10
	clr r18;n_h=0
	clr r19;n_l=0
	ldd zh, y+1
	ldd zl, y+2;read address of *a to z
	lpm r17, z+;read z (*a) to c, z++
	loop:
		cpi r17, _0
		brlo endloop;c >= '0'
		cpi r17, _10
		brsh endloop;c <= '9'
		cpwi r18, r19, max_int
		brsh endloop;n < 65535
		mulw10 r18, r19;n = 10 * n
		subi r17, _0;c = (c -'0')
		addw r18, r19, r17
		lpm r17, z+;read z (*c) to c, z++
		rjmp loop
	endloop:
	;end of function body

	;epilogue
	adiw y, 4;de-allocate the reserved space
	out sph, yh
	out spl, yl

	;push-pop is actuallu doing a recovery job!!
	pop zh
	pop zl
	pop r17
	pop r16
	pop yh
	pop yl
	ret
	;end epilogue