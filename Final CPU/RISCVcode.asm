ARRAY: DW 1, 3, 5, 7, 9, 11, 0
SUMORRED: DW 0, 0
START:
	add t5, x0, x0
	add s0, x0, x0
	add a0, x0, x0
LOOP:
	lw t0, 0(t5)
	beq t0, x0, DONE
	add s0, s0, t0
	or a0, a0, t0
	addi t5, t5, 4
	jal x0, LOOP
DONE:
	sw s0, 0x20(x0)
	sw a0, 0x24(x0)