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