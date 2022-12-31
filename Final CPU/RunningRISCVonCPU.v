module labM;
reg [31:0] PCin;
reg Mem2Reg, RegWrite, clk, ALUSrc, MemRead, MemWrite; 
reg [2:0] op;
wire [31:0] wb, wd, rd1, rd2, imm, ins, PCp4, z, branch;
wire [31:0] jTarget, memOut; //why in lab set as [25:0] - typo?
wire zero;

yIF myIF(ins, PCp4, PCin, clk);
yID myID(rd1, rd2, imm, jTarget, branch, ins, wd, RegWrite, clk);
yEX myEx(z, zero, rd1, rd2, imm, op, ALUSrc);
yDM myDM(memOut, z, rd2, clk, MemRead, MemWrite);
yWB myWB(wb, z, memOut, Mem2Reg);
assign wd = wb;

initial
begin
    //------------------------------------Entry point
    PCin = 16 'h28;
    //------------------------------------Run program
    repeat (43)
    begin
        //---------------------------------Fetch an ins
        clk = 1; #1;
        //---------------------------------Set control signals
        RegWrite = 0; ALUSrc = 1; op = 3'b010;
        MemRead = 0; MemWrite = 0; Mem2Reg = 0;

        //R type: add t5, x0, x0
        if (ins[6:0] == 7'h33)
            begin
                RegWrite = 1; ALUSrc = 0; op = 3'b010; 
                $display("R type: add");
                //MemRead, MemWrite, Mem2Reg r all 0
            end

        //UJ type: jal x0, LOOP
        else if (ins[6:0] == 7'h6F)
            begin //what shl op be??
                RegWrite = 1; ALUSrc = 1; 
                $display("UJ type: jal");
                //MemRead, MemWrite, Mem2Reg r all 0
            end

        //I-Type: lw t0, 0(t5)
        else if (ins[6:0] == 7'h3)
            begin
                RegWrite = 1; ALUSrc = 1; 
                $display("I type: lw");
                MemRead = 1; MemWrite = 0; Mem2Reg = 1;
            end

        //I-Type: addi t5, t5, 4
        else if (ins[6:0] == 7'h13)
            begin
                RegWrite = 1; ALUSrc = 1; op = 3'b010;
                $display("I type: addi");
                //MemRead, MemWrite, Mem2Reg r all 0
            end

        //S-Type: sw s0, 0x20(x0)
        else if (ins[6:0] == 7'h23)
            begin
                RegWrite = 0; ALUSrc = 1; 
                $display("S type: sw");
                MemRead = 0; MemWrite = 1; Mem2Reg = 0;
            end

        //SB type: beq t0, x0, DONE
        else if (ins[6:0] == 7'h63)
            begin
                RegWrite = 0; ALUSrc = 0; op = 3'b110; 
                $display("SB type: beq");
                //MemRead, MemWrite, Mem2Reg r all 0
            end

        //---------------------------------Execute the ins
        clk = 0; #1;
        //---------------------------------View results

        $display("%8h: rd1=%d rd2=%d z=%d zero=%b wb=%d", ins, rd1, rd2, z, zero, wb);

        //---------------------------------Prepare for the next ins
        if (ins[6:0] == 7'h63 && zero == 1) 
            PCin = PCin + (imm<<1);
        else if (ins[6:0] == 7'h6F)
            PCin = PCin + (jTarget<<2); 
        else
        PCin = PCp4;
    end
   
end
endmodule