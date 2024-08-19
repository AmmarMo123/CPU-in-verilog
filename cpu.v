// 2-to-1 1 bit Multiplexer Module
// Selects between inputs a and b based on control signal c
module yMux1 (
    output z,    // Output of the multiplexer
    input a,     // Input a
    input b,     // Input b
    input c      // Control signal: 0 selects a, 1 selects b
);

    // Internal signals
    wire notC;
    wire upper;
    wire lower;

    // Generate the negation of control signal c
    not my_not (
        notC,
        c
    );

    // Compute upper as a AND NOT c
    and upperAnd (
        upper,
        a,
        notC
    );

    // Compute lower as b AND c
    and lowerAnd (
        lower,
        b,
        c
    );

    // Compute final output z as (upper OR lower)
    or my_or (
        z,
        upper,
        lower
    );

endmodule

// N-to-1 Multiplexer Module
module yMux #(
    parameter SIZE = 2  // Parameter to define the bit-width of the multiplexer
)(
    output [SIZE-1:0] z,  // Output vector of the multiplexer
    input [SIZE-1:0] a,   // Input vector a
    input [SIZE-1:0] b,   // Input vector b
    input c               // Select signal
);

    // Instantiate a vector of 2-to-1 multiplexers
    // Each bit of the output z is selected based on the corresponding bits of a, b, and the select signal c
    yMux1 mine[SIZE-1:0](
        .z(z),
        .a(a),
        .b(b),
        .c(c)
    );

endmodule

// 4-to-1 Multiplexer Module
// Selects one of four inputs (a0, a1, a2, a3) based on the 2-bit select signal c
module yMux4to1 (
    output [SIZE-1:0] z,   // Select value for one of the inputs
    input [SIZE-1:0] a0,   // First data input
    input [SIZE-1:0] a1,   // Second data input
    input [SIZE-1:0] a2,   // Third data input
    input [SIZE-1:0] a3,   // Fourth data input
    input [1:0] c          // Select input: determines which data input to choose
);

    parameter SIZE = 2;    // Parameter defining the width of the data inputs and output

    // Internal signals
    wire [SIZE-1:0] zLo;
    wire [SIZE-1:0] zHi;

    // 2-to-1 multiplexer for the lower half of the inputs (a0 and a1)
    yMux #(SIZE) lo (
        .z(zLo),
        .a(a0),
        .b(a1),
        .c(c[0])
    );

    // 2-to-1 multiplexer for the upper half of the inputs (a2 and a3)
    yMux #(SIZE) hi (
        .z(zHi),
        .a(a2),
        .b(a3),
        .c(c[0])
    );

    // 2-to-1 multiplexer to select between the results of the lower and upper multiplexers
    yMux #(SIZE) final (
        .z(z),
        .a(zLo),
        .b(zHi),
        .c(c[1])
    );

endmodule// 1-bit Full Adder Module

module yAdder1(
    output z,      // Sum output
    output cout,   // Carry-out output
    input a,       // Input bit a
    input b,       // Input bit b
    input cin      // Carry-in input
);

    // Internal signals
    wire tmp;
    wire outL;
    wire outR;

    // Compute the sum (z) using XOR gates
    xor (tmp, a, b);
    xor (z, cin, tmp);

    // Compute the carry-out (cout) using AND and OR gates
    and (outL, a, b);
    and (outR, tmp, cin);
    or (cout, outR, outL);

endmodule

// 32-bit Ripple Carry Adder Module
module yAdder(
    output [31:0] z,    // 32-bit Sum output
    output cout,        // Carry-out output
    input [31:0] a,     // 32-bit Input operand a
    input [31:0] b,     // 32-bit Input operand b
    input cin           // Carry-in input
);

    // Internal signals
    wire [31:0] in;
    wire [31:0] out;

    // Instantiate 32 1-bit full adders
    yAdder1 mine[31:0](
        .z(z),
        .cout(out),
        .a(a),
        .b(b),
        .cin(in)
    );

    // Connect carry-in to the first adder and propagate carry-out to the next adder
    assign in[0] = cin;           // Initialize carry-in for the least significant bit
    assign in[31:1] = out[30:0]; // Propagate carry-out from each adder to the next adder's carry-in

    // The final carry-out is the carry-out of the most significant bit
    assign cout = out[31];

endmodule

// Arithmetic Unit Module
// Performs addition if ctrl = 0, and subtraction if ctrl = 1
module yArith (
    output [31:0] z,     // 32-bit output result
    output cout,         // Carry-out bit for addition
    input [31:0] a,      // 32-bit input operand a
    input [31:0] b,      // 32-bit input operand b
    input ctrl           // Control signal: 0 for addition, 1 for subtraction
);

    // Internal signals
    wire [31:0] notB;
    wire [31:0] tmp;
    wire cin;

    // Generate the bitwise negation of b
    not b_not[31:0] (
        notB,
        b
    );

    // Multiplexer to select between b and ~b based on ctrl signal
    // If ctrl = 0, tmp = b (for addition)
    // If ctrl = 1, tmp = ~b (for subtraction)
    yMux #(.SIZE(32)) my_mux[31:0] (
        .z(tmp),
        .a(b),
        .b(notB),
        .c(ctrl)
    );

    // Assign carry-in for the adder from ctrl
    assign cin = ctrl;

    // 32-bit adder to compute z = a + tmp
    // cout is the carry-out bit
    yAdder my_add[31:0] (
        .z(z),
        .cout(cout),
        .a(a),
        .b(tmp),
        .cin(cin)
    );

endmodule

// ALU Module
// Performs various arithmetic and logic operations based on the op code
module yAlu (
    output [31:0] z,   // Result of the operation
    output ex,         // Zero flag: set if result z is zero
    input [31:0] a,    // Operand a
    input [31:0] b,    // Operand b
    input [2:0] op     // Operation code - op=000: z=a AND b, op=001: z=a|b, op=010: z=a+b, op=110: z=a-b
);

    // Internal signals
    wire [15:0] z16;
    wire [7:0] z8;
    wire [3:0] z4;
    wire [1:0] z2;
    wire z1;
    wire cout;
    wire [31:0] zAnd, zOr, zArith, slt;
    wire condition;
    wire [31:0] aSubB;

    // Perform bitwise AND operation
    and ab_and[31:0] (
        zAnd,
        a,
        b
    );

    // Perform bitwise OR operation
    or ab_or[31:0] (
        zOr,
        a,
        b
    );

    // Compute the zero flag
    or or16[15:0] (z16, z[15:0], z[31:16]);
    or or8[7:0] (z8, z16[7:0], z16[15:8]);
    or or4[3:0] (z4, z8[3:0], z8[7:4]);
    or or2[1:0] (z2, z4[1:0], z4[3:2]);
    or or1 (z1, z2[1], z2[0]);
    not zero_not(ex, z1);

    // Perform set-less-than operation
    xor slt_xor (
        condition,
        a[31],
        b[31]
    );

    yArith slt_arith (
        .z(aSubB),
        .cout(cout),
        .a(a),
        .b(b),
        .ctrl(1'b1)
    );

    yMux1 my_mux_slt (
        .z(slt[0]),
        .a(aSubB[31]),
        .b(a[31]),
        .c(condition)
    );

    // Perform arithmetic operations (add or subtract)
    yArith ab_arith[31:0] (
        .z(zArith),
        .cout(cout),
        .a(a),
        .b(b),
        .ctrl(op[2])
    );

    // Select between the results based on the op code
    yMux4to1 #(.SIZE(32)) my_mux (
        .z(z),
        .a0(zAnd),
        .a1(zOr),
        .a2(zArith),
        .a3(slt),
        .c(op[1:0])
    );

endmodule

// Parameterized Register Module
// It captures data on the rising edge of the clock when the enable signal is high.
module register #(parameter WIDTH = 32) (
    output reg [WIDTH-1:0] q,      // Output: the current value stored in the register
    input wire [WIDTH-1:0] d,      // Input: data to be stored in the register
    input wire clk,                // Input: clock signal for synchronizing the data capture
    input wire enable              // Input: enable signal to control when the data is captured
);

    always @(posedge clk) begin
        // On each rising edge of the clock
        if (enable)                 // If enable signal is high
            q <= d;                // Update the register value with the input data
    end

endmodule

// Register File Module
module rf (
    output reg [31:0] rd1,   // Read data 1
    output reg [31:0] rd2,   // Read data 2
    input  wire [4:0]  rs1,  // Read register 1 select
    input  wire [4:0]  rs2,  // Read register 2 select
    input  wire [4:0]  wn,   // Write register number
    input  wire [31:0] wd,   // Write data
    input  wire        clk,  // Clock
    input  wire        w     // Write enable
);

    // Register file (32 registers, each 32-bits wide)
    reg [31:0] registers [31:0];

    // Initialize register $0 to 0 (Read-only register)
    initial begin
        registers[0] = 32'b0;
    end

    // Write operation (on positive edge of clk)
    always @(posedge clk) begin
        if (w && wn != 5'b00000)  // Ensure w is enabled and not writing to register 0
            registers[wn] <= wd;
    end

    // Continuous assignment for read operations
    always @(*) begin
        rd1 = registers[rs1];
        rd2 = registers[rs2];
    end
endmodule

// Memory Module
// This module implements a memory with read and write capabilities.
// It loads initial values from ram.dat
module mem (
    output reg [31:0] memOut,  // Output data (read)
    input wire [31:0] address, // Address for read/write
    input wire [31:0] memIn,   // Input data (write)
    input wire clk,            // Clock
    input wire memRead,        // Memory read enable
    input wire memWrite        // Memory write enable
);

    // Memory array (1024 words of 32 bits each, adjust size as needed)
    reg [31:0] memory [1023:0];

    // Word-aligned address check (address should be divisible by 4)
    wire [9:0] wordAddress = address[11:2]; // Extracting word address (ignoring lower 2 bits)
    wire alignedAddress = (address[1:0] == 2'b00);

    // Declare variables at the module level
    integer file;
    integer r;
    reg [31:0] addr, content;
    reg [255:0] line;  // Support for long lines

    // Initialization block to load memory from "ram.dat"
    initial begin
        file = $fopen("ram.dat", "r");
        if (file) begin
            $display("Initializing memory from ram.dat");
            while (!$feof(file)) begin
                r = $fgets(line, file); // Read one line from the file
                    r = $sscanf(line, "@%h %h", addr, content); // Parse the line for address and content
                    if (r == 2 && addr[1:0] == 2'b00) begin // Check word alignment
                        memory[addr[11:2]] = content;
                        $display("Loaded address %h with data %h", addr, content);
                    end else if (r == 2) begin
                        $display("unaligned address in ram.dat: %h", addr);
                    end
            end
            $fclose(file);
        end else begin
            $display("ram.dat not found. Memory initialized to zero.");
            for (integer i = 0; i < 1024; i = i + 1) begin
                memory[i] = 32'b0;
            end
        end
    end

    // Read operation
    always @(*) begin
        if (memRead) begin
            if (alignedAddress)
                memOut = memory[wordAddress];
            else begin
                memOut = 32'b0; // Returning zero if address is unaligned
                $display("unaligned address: %h", address);
            end
        end else begin
            memOut = 32'bz; // High impedance when not reading
        end
    end

    // Write operation
    always @(posedge clk) begin
        if (memWrite) begin
            if (alignedAddress)
                memory[wordAddress] <= memIn;
            else
                $display("unaligned address: %h", address);
        end
    end

endmodule

// Instruction Fetch (IF) Module
// This module handles fetching instructions from memory, updating the program counter (PC), and calculating the next PC.
module yIF (
    output [31:0] ins,   // Fetched instruction from memory
    output [31:0] PC,    // Current program counter value
    output [31:0] PCp4,  // Program counter + 4 (next instruction address)
    input [31:0] PCin,   // Input for updating the program counter
    input clk            // Clock signal
);

    // Internal signals
    wire zero;
    wire read, write;
    wire enable;
    wire [31:0] a;
    wire [31:0] memIn;
    wire [2:0] add;

    // Assign constants
    assign enable = 1'b1;    // Enable the PC register to update
    assign a = 32'h0004;     // Constant value 4 for PC increment
    assign add = 3'b010;     // ALU operation: 010 for addition
    assign read = 1'b1;      // Enable memory read operation
    assign write = 1'b0;     // Disable memory write operation

    // Program Counter Register
    register #(32) pcReg (
        .q(PC),
        .d(PCin),
        .clk(clk),
        .enable(enable)
    );

    // Instruction Memory
    mem insMem (
        .memOut(ins),
        .address(PC),
        .memIn(memIn),
        .clk(clk),
        .memRead(read),
        .memWrite(write)
    );

    // ALU for calculating PC + 4
    yAlu myAlu (
        .z(PCp4),
        .ex(zero),
        .a(a),
        .b(PC),
        .op(add)
    );
endmodule

// Instruction Decode Module
// This module decodes the instruction and generates necessary control signals and outputs.
module yID (
    output [31:0] rd1,      // Data read from the first register
    output [31:0] rd2,      // Data read from the second register
    output [31:0] immOut,   // Immediate value output (based on instruction type)
    output [31:0] jTarget,  // Jump target address (for UJ-Type instructions)
    output [31:0] branch,   // Branch address (for SB-Type instructions)
    input [31:0] ins,       // Instruction input
    input [31:0] wd,        // Write data to register file
    input RegWrite,         // Register write enable
    input clk               // Clock signal
);

    // Internal signals
    wire [19:0] zeros, ones;
    wire [11:0] zerosj, onesj;
    wire [31:0] imm, saveImm;

    // Register file instantiation
    // Reads data from registers based on instruction's rs1, rs2, and rd fields
    rf myRF (
        .rd1(rd1),
        .rd2(rd2),
        .rs1(ins[19:15]),
        .rs2(ins[24:20]),
        .wn(ins[11:7]),
        .wd(wd),
        .clk(clk),
        .w(RegWrite)
    );

    // Immediate value extraction and sign extension
    assign imm[11:0] = ins[31:20];  // Extract immediate from instruction
    assign zeros = 20'h00000;        // Zero constant for sign extension
    assign ones = 20'hFFFFF;         // One constant for sign extension
    yMux #(20) se (
        .z(imm[31:12]),
        .a(zeros),
        .b(ones),
        .c(ins[31])
    );

    // Immediate value for S-Type instructions
    assign saveImm[11:5] = ins[31:25]; // Upper part of immediate
    assign saveImm[4:0] = ins[11:7];   // Lower part of immediate
    yMux #(20) saveImmSe (
        .z(saveImm[31:12]),
        .a(zeros),
        .b(ones),
        .c(ins[31])
    );

    // Select the appropriate immediate value based on instruction type
    yMux #(32) immSelection (
        .z(immOut),
        .a(imm),
        .b(saveImm),
        .c(ins[5])
    );

    // Branch address calculation for SB-Type instructions
    assign branch[11] = ins[31];          // Sign bit for upper part of branch address
    assign branch[10] = ins[7];           // Middle part of branch address
    assign branch[9:4] = ins[30:25];      // Upper part of branch address
    assign branch[3:0] = ins[11:8];       // Lower part of branch address
    yMux #(20) bra (
        .z(branch[31:12]),
        .a(zeros),
        .b(ones),
        .c(ins[31])
    );

    // Jump target address calculation for UJ-Type instructions
    assign zerosj = 12'h000;             // Zero constant for UJ-Type
    assign onesj = 12'hFFF;              // One constant for UJ-Type
    assign jTarget[19] = ins[31];        // Sign bit for upper part of jump target
    assign jTarget[18:11] = ins[19:12];  // Middle part of jump target
    assign jTarget[10] = ins[20];        // Additional part of jump target
    assign jTarget[9:0] = ins[30:21];    // Lower part of jump target
    yMux #(12) jum (
        .z(jTarget[31:20]),
        .a(zerosj),
        .b(onesj),
        .c(jTarget[19])
    );

endmodule

// Execute Stage Module
// This module performs ALU operations based on the inputs and control signals.
module yEX (
    output [31:0] z,     // Result of the ALU operation
    output zero,         // Zero flag indicating if the result is zero
    input [31:0] rd1,    // First operand for ALU operation
    input [31:0] rd2,    // Second operand for ALU operation (or immediate value)
    input [31:0] imm,    // Immediate value used for certain operations
    input [2:0] op,      // ALU operation code
    input ALUSrc         // Control signal to select between rd2 and imm
);

    // Internal signal to hold the result of the mux operation
    wire [31:0] muxOut;

    // Mux to select between rd2 and immediate value based on ALUSrc
    // When ALUSrc is 1, imm is selected; otherwise, rd2 is selected
    yMux #(32) reg_or_imm (
        .z(muxOut),
        .a(rd2),
        .b(imm),
        .c(ALUSrc)
    );

    // ALU to perform the operation based on the opcode and selected operand
    yAlu execAlu (
        .z(z),
        .ex(zero),
        .a(rd1),
        .b(muxOut),
        .op(op)
    );

endmodule

// Data Memory Module
// This module interfaces with the memory module to handle read and write operations.
module yDM (
    output [31:0] memOut,   // Data read from memory
    input [31:0] exeOut,    // Address for read/write operations
    input [31:0] rd2,       // Data to be written to memory
    input clk,              // Clock signal
    input MemRead,          // Memory read enable
    input MemWrite          // Memory write enable
);

    // Instantiate the memory module
    mem data_mem (
        .memOut(memOut),
        .address(exeOut),
        .memIn(rd2),
        .clk(clk),
        .memRead(MemRead),
        .memWrite(MemWrite)
    );
endmodule

// Write-Back Module
// This module selects the appropriate value to be written back to the register file
// based on the Mem2Reg control signal.
module yWB (
    output [31:0] wb,      // Data to be written back to the register file
    input [31:0] exeOut,   // ALU result (potential write-back data)
    input [31:0] memOut,   // Data read from memory (potential write-back data)
    input Mem2Reg          // Control signal to select between exeOut and memOut
);

    // Instantiate a 2-to-1 multiplexer to select the write-back data
    // based on the Mem2Reg signal.
    yMux #(32) writeback (
        .z(wb),
        .a(exeOut),
        .b(memOut),
        .c(Mem2Reg)
    );
endmodule

// Program Counter (PC) Module
// This module computes the next value of the Program Counter (PC) based on different control signals 
// such as interrupts, branching, and jumping.
module yPC (
    output [31:0] PCin,        // Next PC value to be used in the next clock cycle
    input [31:0] PC,           // Current PC value
    input [31:0] PCp4,         // PC + 4 (next sequential instruction address)
    input INT,                 // Interrupt signal
    input [31:0] entryPoint,   // Entry point address (used for interrupt handling)
    input [31:0] branchImm,    // Immediate value for branching
    input [31:0] jImm,         // Immediate value for jumping
    input zero,                // Zero flag (used to determine branch condition)
    input isbranch,            // Branch control signal
    input isjump               // Jump control signal
);

    // Internal signals
    wire [31:0] branchImmX4;
    wire [31:0] jImmX4;
    wire [31:0] jImmX4PPCp4;
    wire [31:0] bTarget;
    wire [31:0] choiceA, choiceB;
    wire doBranch;
    wire zf;
    wire [2:0] add;

    // Assign constants
    assign enable = 1'b1;    // Enable the PC register to update
    assign a = 32'h0004;     // Constant value 4 for PC increment
    assign add = 3'b010;     // ALU operation: 010 for addition

    // Shifting branch immediate left by 2 bits (multiply by 4)
    assign branchImmX4[31:2] = branchImm[29:0];
    assign branchImmX4[1:0] = 2'b00;

    // Shifting jump immediate left by 2 bits (multiply by 4)
    assign jImmX4[31:2] = jImm[29:0];
    assign jImmX4[1:0] = 2'b00;

    // Calculate branch target address by adding PC and branchImmX4
    yAlu bALU (
        .z(bTarget), 
        .ex(zf), 
        .a(PC), 
        .b(branchImmX4), 
        .op(add)
    );

    // Calculate jump target address by adding PC and jImmX4
    yAlu jALU (
        .z(jImmX4PPCp4), 
        .ex(zf), 
        .a(PC), 
        .b(jImmX4), 
        .op(add)
    );

    // Determine whether to branch (isbranch AND zero)
    and decide_doBranch(doBranch, isbranch, zero);

    // First Mux: Choose between sequential PC (PCp4) and branch target (bTarget)
    yMux #(32) mux1 (
        .z(choiceA), 
        .a(PCp4), 
        .b(bTarget), 
        .c(doBranch)
    );

    // Second Mux: Choose between previous choice (choiceA) and jump target (jImmX4PPCp4)
    yMux #(32) mux2 (
        .z(choiceB), 
        .a(choiceA), 
        .b(jImmX4PPCp4), 
        .c(isjump)
    );

    // Third Mux: Choose between previous choice (choiceB) and entry point (entryPoint) based on interrupt
    yMux #(32) mux3 (
        .z(PCin), 
        .a(choiceB), 
        .b(entryPoint), 
        .c(INT)
    );
endmodule

// Control Unit Module (yC1)
// This module decodes the operation code (opCode) to generate control signals
// for different instruction types: S-Type, R-Type, I-Type, LW, JUMP, and BRANCH.
module yC1 (
    output isStype,    // Signal indicating S-Type instruction
    output isRtype,    // Signal indicating R-Type instruction
    output isItype,    // Signal indicating I-Type instruction
    output isLw,       // Signal indicating LW instruction
    output isjump,     // Signal indicating JUMP instruction (UJ-Type)
    output isbranch,   // Signal indicating BRANCH instruction (SB-Type)
    input [6:0] opCode // 7-bit operation code input
);

    // opCode
    // lw      0000011
    // I-Type  0010011
    // R-Type  0110011
    // SB-Type 1100011 beq
    // UJ-Type 1101111 jal
    // S-Type  0100011

    // Detect UJ-type
    assign isjump= opCode[3];

    // Detect lw
    or lw_detect(lwor, opCode[6], opCode[5], opCode[4], opCode[3], opCode[2]); not (isLw, lwor);

    // Select between S-Type and I-Type
    xor s_i_xor(ISselect, opCode[6], opCode[3], opCode[2], opCode[1], opCode[0]);
    and s_and(isStype, ISselect, opCode[5]);
    and i_and(isItype, ISselect, opCode[4]);

    // Detect R-Type
    and r_and(isRtype, opCode[5], opCode[4]);

    // Select between JAL and Branch
    and JB_and(JBselect, opCode[6], opCode[5]); // SB and UJ are the only ones with bits 5 and 6
    not branch_detect(sbz, opCode[3]); // SB has 0 in bits 2 and 3
    and branch_and(isbranch, JBselect, sbz);
endmodule

// Control Unit Module (yC2)
// This module generates control signals for register writing, ALU source selection,
// memory reading, memory writing, and memory-to-register selection based on instruction
// types set from yC1.
module yC2 (
    output RegWrite,   // Signal to enable writing to a register
    output ALUSrc,     // Signal to select ALU source (immediate vs. register)
    output MemRead,    // Signal to enable memory read operation
    output MemWrite,   // Signal to enable memory write operation
    output Mem2Reg,    // Signal to select data from memory to write to register
    input isStype,     // Input: S-Type instruction indicator
    input isRtype,     // Input: R-Type instruction indicator
    input isItype,     // Input: I-Type instruction indicator
    input isLw,        // Input: Load Word (LW) instruction indicator
    input isjump,      // Input: Jump (UJ-Type) instruction indicator
    input isbranch     // Input: Branch (SB-Type) instruction indicator
);

    // ALUSrc is 1 for I-Type and sw and UJ
    // Mem2Reg is 1 for lw
    // RegWrite is 1 for R-format and lw and UJ
    // MemRead is 1 for lw
    // MemWrite is 1 for sw

    nor (ALUSrc, isRtype, isbranch);			//0 - do calculation; 1 - add immediate
    nor (RegWrite, isStype, isbranch);			//need to write to a register

    assign Mem2Reg = isLw;
    assign MemRead = isLw;
    assign MemWrite = isStype;
endmodule

// Control Unit Module (yC3)
// This module generates the ALUop, which determines non-R-type instructions 
module yC3 (
    output [1:0] ALUop, // 2-bit ALU operation code
    input isRtype,      // Input: R-Type instruction indicator
    input isbranch      // Input: Branch instruction indicator
);

    // Generate the ALU operation code based on the instruction type
    assign ALUop[1] = isRtype;
    assign ALUop[0] = isbranch;
endmodule

// ALU Control Unit (yC4)
// This module generates the final ALU operation code (op) based on ALUop and funct3 inputs.
module yC4 (
    output [2:0] op,       // 3-bit ALU operation code
    input [1:0] ALUop,     // 2-bit ALU operation code generated by yC2
    input [2:0] funct3     // 3-bit function code from the instruction
);

    wire f21out, f10out, upperAndOut, notALU, notf3;

    // Bit 2 of the ALU operation code
    xor f21 (f21out, funct3[2], funct3[1]);
    and upperAnd(upperAndOut, ALUop[1], f21out);
    or upperOr(op[2], ALUop[0], upperAndOut);

    // Bit 1 of the ALU operation code
    not ALUop1no(notALU, ALUop[1]);
    not f3no(notf3, funct3[1]);
    or lowerOr(op[1], notALU, notf3);

    // Bit 0 of the ALU operation code
    xor f10 (f10out, funct3[1], funct3[0]);
    and lowerAnd(op[0], ALUop[1], f10out);
endmodule

// Top-level Processor Module (yChip)
// This module integrates all the stages of the processor pipeline: Instruction Fetch (IF), 
// Instruction Decode (ID), Execution (EX), Data Memory (DM), and Write-Back (WB). 
// It controls the flow of instructions through the processor.
module yChip (
    output [31:0] ins,       // Current instruction (Added only for testing purposes)
    output [31:0] rd2,       // Data from the second register operand (Added only for testing purposes)
    output [31:0] wb,        // Data to be written back to the register file (Added only for testing purposes)
    input [31:0] entryPoint, // Starting address for the program
    input INT,               // Interrupt signal
    input clk                // Clock signal
);

    // Internal signals
    wire [31:0] PCin, PC, wd, rd1, imm, PCp4, z, branch, jTarget, memOut;
    wire [2:0] op, funct3;
    wire [6:0] opCode;
    wire zero;
    wire isStype, isRtype, isItype, isLw, isjump, isbranch;
    wire RegWrite, ALUSrc, MemRead, MemWrite, Mem2Reg;
    wire [1:0] ALUop;

    // Instruction Fetch (IF) Stage
    yIF myIF(
        .ins(ins), 
        .PC(PC), 
        .PCp4(PCp4), 
        .PCin(PCin), 
        .clk(clk)
    );

    // Instruction Decode (ID) Stage
    yID myID(
        .rd1(rd1), 
        .rd2(rd2), 
        .immOut(imm), 
        .jTarget(jTarget), 
        .branch(branch), 
        .ins(ins), 
        .wd(wd), 
        .RegWrite(RegWrite), 
        .clk(clk)
    );

    // Execute (EX) Stage
    yEX myEx(
        .z(z), 
        .zero(zero), 
        .rd1(rd1), 
        .rd2(rd2), 
        .imm(imm), 
        .op(op), 
        .ALUSrc(ALUSrc)
    );

    // Data Memory (DM) Stage
    yDM myDM(
        .memOut(memOut), 
        .exeOut(z), 
        .rd2(rd2), 
        .clk(clk), 
        .MemRead(MemRead), 
        .MemWrite(MemWrite)
    );

    // Write-Back (WB) Stage
    yWB myWB(
        .wb(wb), 
        .exeOut(z), 
        .memOut(memOut), 
        .Mem2Reg(Mem2Reg)
    );

    // Program Counter (PC) Module
    yPC myPC(
        .PCin(PCin), 
        .PC(PC), 
        .PCp4(PCp4), 
        .INT(INT), 
        .entryPoint(entryPoint), 
        .branchImm(branch), 
        .jImm(jTarget), 
        .zero(zero), 
        .isbranch(isbranch), 
        .isjump(isjump)
    );

    // Control Unit - Part 1: Decode opCode, generate 
    // ALUSrc, RegWrite, Mem2Reg, MemRead, MemWrite, isjump
    // and isbranch signals
    assign opCode = ins[6:0];
    yC1 myC1(
        .isStype(isStype), 
        .isRtype(isRtype), 
        .isItype(isItype), 
        .isLw(isLw), 
        .isjump(isjump), 
        .isbranch(isbranch), 
        .opCode(opCode)
    );

    // Control Unit - Part 2: Set ALUSrc, RegWrite, Mem2Reg,
    // MemRead and MemWrite based on the output of yC1
    yC2 myC2(
        .RegWrite(RegWrite), 
        .ALUSrc(ALUSrc), 
        .MemRead(MemRead), 
        .MemWrite(MemWrite), 
        .Mem2Reg(Mem2Reg), 
        .isStype(isStype), 
        .isRtype(isRtype), 
        .isItype(isItype), 
        .isLw(isLw), 
        .isjump(isjump), 
        .isbranch(isbranch)
    );

    // ALU Control Unit, yC3 and yC4 generate the opcode collabaratively.
    // yC3, is responsible for non-R-type, and yC4 takes care of R-types.
    yC3 myC3(
        .ALUop(ALUop), 
        .isRtype(isRtype), 
        .isbranch(isbranch)
    );

    assign funct3 = ins[14:12];
    yC4 myC4(
        .op(op), 
        .ALUop(ALUop), 
        .funct3(funct3)
    );

    // Write-back Data Assignment
    assign wd = wb;
endmodule