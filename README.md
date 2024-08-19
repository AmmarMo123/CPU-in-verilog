## Project Description
This project was guided by additional resources provided by the Computer Organisation course. This project was not submitted or graded, and I implemented it in my free time outside of class.

Designed a fully pipelined 5-stage processor from the scratch, starting with basic combinational components like adders, multiplexers, and ALUs, followed by sequential components such as registers and memory. The design included circuits for instruction fetch, decode, execute, memory access, and writeback stages. Additionally, a control unit was implemented to generate the necessary signals and coordinate the operation of the processor.

## Interacting with the processor
The processor built interacts with the outside world through four channels: 
- The clock signal (input).
- The interrupt signal (input).
- The entry point signal (input).
- The BIU (Bus Interface Unit in yIF and yDM) (input and output).

## Testing the processor
The processor was tested using a sample RISC-V program (`ram.dat` and `testbed_program.asm`), with register values being read to confirm it operates as expected.
The program in `ram.dat` is used to initialize memory, and the processor executes the instructions from there. When the component is first instantiated, it looks (in the current working directory) for that file, and if present, it reads its content to initialize the memory words. Each record in the file holds an address, content pair:
```
@a c // optional comment
```
The address a must be prefixed with @ and is followed by the content c. Both values must be in hex and are separated by whitespace. The record may end with an in-line comment using the // separator.

`execute.v` simulates the execution of the processor, and prints out the values stored in the registers that hold the final result of the test bench.

## Running the processor
All modules have been consolidated into `cpu.v` so that it can be easily compiled, eliminating the need to manage multiple files during the compilation process.

## Supported Instructions
| Instruction   | Assembly      | Type | Opcode | funct 3 | funct 7 | Description |
| ------------- | ------------- | ---|---|---|---|---|
| and  | and rd rs1 rs2 | R-type | 0110011 | 0x7 | 0x00 | rd = rs1 & rs2 |
| or	| or rd rs1 rs2	| R-type	| 0110011	| 0x6	| 0x00 | rd = rs1 \| rs2 |
| add	| add rd rs1 rs2	| R-type	| 0110011	| 0x0	| 0x00 | rd = rs1 + rs2|
| sub	| sub rd rs1 rs2	| R-type	| 1101111	| 0x0	| 0x20 | rd = rs1 - rs2|
| slt	| slt rd rs1 rs2	| R-type	| 1101111	| 0x2	| 0x00 | rd = (rs1 < rs2)?1:0|
| addi	| addi rd, rs1, imm	| I-type	| 0010011	| 0x0	| imm[5:11]=0x00 | rd = rs1 + imm|
| lw	| lw rd, imm(rs1)	| I-type	| 0000011	| 0x2 | N/A	| rd = M[rs1+imm][0:31] | 
| sw	| sw rs2, imm(rs1)	| S-type	| 0100011	| 0x2	| N/A | M[rs1+imm][0:31] = rs2[0:31]|
| beq	| beq rs1, rs2, label	| SB-type	| 1100011	| 0x0	| N/A | if (rs1 == rs2) PC += imm|
| jal	| jal rd, label	| UJ-type	| 1101111	| 0x0	| N/A | rd = PC+4; PC += imm|

## Visual recources
Diagram used as reference, showcasing the components involved at each stage and their connections:
<img width="456" alt="Screenshot 2024-08-19 at 7 03 54 AM" src="https://github.com/user-attachments/assets/e46dd392-94b3-4b5e-8126-1b67286847e0">

(Credits: Computer Organization and Design by David A. Patterson and John L. Henessy)
