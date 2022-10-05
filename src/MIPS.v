`timescale 1ns/1ns
module MIPS(clk, rst);
  input clk, rst;
  wire[5:0] opcode, func;
  wire[2:0] ALUoperation;
  wire RegDst, ALUsrc, MemtoReg, MemWrite, MemRead, RegWrite, PCsrc, eq, clr, jalRegSel, jalWriteSel;
  wire[1:0] jsel;
  Controller CU(opcode, func, eq, RegDst, ALUsrc, MemWrite, MemRead, MemtoReg, RegWrite, jsel, PCsrc, clr, jalRegSel, jalWriteSel, ALUoperation);
  Datapath DP(clk, rst, PCsrc, jsel, RegDst, ALUsrc, MemWrite, MemRead, MemtoReg, RegWrite, clr, jalRegSel, jalWriteSel, ALUoperation, func, opcode, eq);
endmodule

module MIPS_TB();
  reg clk = 1'b0, rst; 
  MIPS mips(clk, rst);
  always #100 clk = ~clk;
  initial begin
    #20 rst = 1'b1;
    #120 rst = 1'b0;
    #50000 $stop;
  end
endmodule