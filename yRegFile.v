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