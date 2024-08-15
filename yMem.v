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