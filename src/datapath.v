`timescale 1ns/1ns
module MUX2to1 #(parameter N = 2)(A, B, sel, out);
  input [N-1:0] A, B;
  input sel;
  output [N-1:0] out;
  assign out = sel ? B : A;
endmodule

module MUX3to1 #(parameter N = 2)(A, B, C, sel, out);
  input [N-1:0] A, B, C;
  input [1:0] sel;
  output [N-1:0] out;
  reg [N-1:0] out;
  always @(A, B, C, sel)begin
    case(sel)
      2'b00: out = A;
      2'b01: out = B;
      2'b10: out = C;
    endcase
  end
endmodule

module Adder(A ,B, sum);
  input[31:0] A, B;
  output[31:0] sum;
  assign sum = A + B;
endmodule

module SignExtend(in, out);
  input[15:0] in;
  output[31:0] out;
  assign out = in[15] ? {16'b1111111111111111,in} : {16'b0000000000000000,in};
endmodule

module ShiftLeft26to28(in, out);
  input[25:0] in;
  output[27:0] out;
  assign out = {in, 2'b00};
endmodule

module ShiftLeft32(in, out);
  input[31:0] in;
  output[31:0] out;
  assign out = {in[29:0],2'b00};
endmodule

module PCreg(pc, clk, ld, rst, nextPC);
  input[31:0] pc, clk, ld, rst;
  output[31:0] nextPC;
  reg[31:0] nextPC;
  always@(posedge clk, posedge rst) begin
    if(rst) nextPC <= 32'd0;
    else if(ld) nextPC <= pc; 
  end
endmodule

module DataMemory(Address, WriteData, clk, MemRead, MemWrite, ReadData);
  input [31:0] Address, WriteData;
  input MemWrite, MemRead, clk;
  output [31:0] ReadData;
  reg [31:0] memory [0:2048];
  always @(posedge clk)begin
    if(MemWrite)
      memory[Address] <= WriteData;
  end
  initial begin
    $readmemb("array1.txt",memory);
  end
  //array1 -> 55
  //array2 -> 60
  //array3 -> 100
  //array4 -> 120
  assign ReadData = MemRead ? memory[Address] : 32'd0;
endmodule

module checkEqual(A, B, eq);
  input[31:0] A, B;
  output eq;
  assign eq = (A == B);
endmodule

module ALU(A, B, ALUoperation, result);
  input[31:0] A, B;
  input[2:0] ALUoperation;
  output[31:0] result;
  reg[31:0] result;
  always@(ALUoperation,A,B) begin
    case(ALUoperation)
    3'b000: result = A & B;
    3'b001: result = A | B;
    3'b010: result = A + B;
    3'b110: result = A - B;
    3'b111: result = A < B;
    endcase
  end
endmodule

module InstructionMemory(Address, instruction);
  input [31:0] Address;
  output [31:0] instruction;
  reg [31:0] memory [0:2047];  
  initial begin
    $readmemb("instructions.txt",memory);
  end
  assign instruction = memory[Address];
endmodule

module RegisterFile(clk, RegWrite, ReadRegister1, ReadRegister2, WriteRegister, WriteData,  ReadData1, ReadData2);
  input [4:0] ReadRegister1, ReadRegister2, WriteRegister;
  input [31:0] WriteData;
  input RegWrite, clk;
  output [31:0]  ReadData1, ReadData2;
  reg [31:0] registers [0:31];
  initial begin
    registers[0] = 32'd0;
  end
  assign ReadData1 = (ReadRegister1 == 5'd0) ? 32'd0 : registers[ReadRegister1];
  assign ReadData2 = (ReadRegister2 == 5'd0) ? 32'd0 : registers[ReadRegister2];
  always @(negedge clk)begin
    if(RegWrite)
      registers[WriteRegister] <= WriteData;
    end
endmodule

module IF_ID_reg(nextPCIn, instIn, clk, ld, clr, nextPCOut, instOut);
  input[31:0] nextPCIn, instIn;
  input clk, ld, clr;
  output [31:0] nextPCOut, instOut;
  reg[31:0] nextPCOut, instOut;
  always@(posedge clk) begin
    if(clr) instOut <= 32'b00000000000000000000000000100000;
    else begin
      nextPCOut <= nextPCIn;
      if(ld) instOut <= instIn; 
    end
  end
endmodule

module ID_EX_reg(cntrl, readData1In, readData2In, ExtIn, jalPCDataIn, RtIn, RsIn, RdIn, clk,
  cntrlOut, readData1Out, readData2Out, ExtOut, jalPCDataOut, RtOut, RsOut, RdOut);
  input[10:0] cntrl;
  input[31:0] readData1In, readData2In, ExtIn, jalPCDataIn;
  input[4:0] RtIn, RsIn, RdIn;
  input clk;
  output[10:0] cntrlOut;
  reg[10:0] cntrlOut;
  output[31:0] readData1Out, readData2Out, ExtOut, jalPCDataOut;
  reg[31:0] readData1Out, readData2Out, ExtOut, jalPCDataOut;
  output[4:0] RtOut, RsOut, RdOut;
  reg[4:0] RtOut, RsOut, RdOut;

  always@(posedge clk) begin
    cntrlOut <= cntrl;
    readData1Out <= readData1In;
    readData2Out <= readData2In;
    ExtOut <= ExtIn;
    RtOut <= RtIn;
    RsOut <= RsIn;
    RdOut <= RdIn;
    jalPCDataOut <= jalPCDataIn;
  end
endmodule

module EX_MEM_reg(cntrl, ALUResIn, writeDataIn, jalPCDataIn, DstRegIn, clk,
  cntrlOut, ALUResOut, writeDataOut, jalPCDataOut, DstRegOut);
  input[4:0] cntrl;
  input[31:0] ALUResIn, writeDataIn, jalPCDataIn;
 
input[4:0] DstRegIn;
  input clk;
  output[4:0] cntrlOut;
  reg[4:0] cntrlOut;
  output[31:0] ALUResOut, writeDataOut, jalPCDataOut;
  reg[31:0] ALUResOut, writeDataOut, jalPCDataOut;
  output[4:0] DstRegOut;
  reg[4:0] DstRegOut;
  
  always@(posedge clk) begin
    cntrlOut <= cntrl;
    ALUResOut <= ALUResIn;
    writeDataOut <= writeDataIn;
    DstRegOut <= DstRegIn;
    jalPCDataOut <= jalPCDataIn;
  end
endmodule

module MEM_WB_reg(cntrl, ALUResIn, readDataMemoryIn, jalPCDataIn, DstRegIn, clk,
  cntrlOut, ALUResOut, readDataMemoryOut, jalPCDataOut, DstRegOut);
  input[2:0] cntrl;
  input[31:0] ALUResIn, readDataMemoryIn, jalPCDataIn;
  input[4:0] DstRegIn;
  input clk;
  output[2:0] cntrlOut;
 
reg[2:0] cntrlOut;
  output[31:0] ALUResOut, readDataMemoryOut, jalPCDataOut;
  reg[31:0] ALUResOut, readDataMemoryOut, jalPCDataOut;
  output[4:0] DstRegOut;
  reg[4:0] DstRegOut;
  always@(posedge clk) begin
    cntrlOut <= cntrl;
    ALUResOut <= ALUResIn;
    readDataMemoryOut <= readDataMemoryIn;
    DstRegOut <= DstRegIn;
    jalPCDataOut <= jalPCDataIn;
  end
endmodule

module ForwardUnit(ID_EX_Rs, ID_EX_Rt, EX_MEM_Rd, MEM_WB_Rd, EX_MEM_RegWrite, MEM_WB_RegWrite, FrwA, FrwB);
  input [4:0] ID_EX_Rs, ID_EX_Rt, EX_MEM_Rd, MEM_WB_Rd;
  input EX_MEM_RegWrite, MEM_WB_RegWrite;
  output[1:0] FrwA, FrwB;
  reg[1:0] FrwA, FrwB;
  
  always@(ID_EX_Rs, EX_MEM_Rd, MEM_WB_Rd, EX_MEM_RegWrite, MEM_WB_RegWrite) begin
    FrwA = 2'b00;
    if(EX_MEM_RegWrite == 1 && EX_MEM_Rd == ID_EX_Rs && EX_MEM_Rd != 5'd0)
      FrwA = 2'b01;
    else if(MEM_WB_RegWrite == 1 && MEM_WB_Rd == ID_EX_Rs && MEM_WB_Rd != 5'd0)
      FrwA = 2'b10;
  end

  always@(ID_EX_Rt, EX_MEM_Rd, MEM_WB_Rd, EX_MEM_RegWrite, MEM_WB_RegWrite) begin
    FrwB = 2'b00;
    if(EX_MEM_RegWrite == 1 && EX_MEM_Rd == ID_EX_Rt && EX_MEM_Rd != 5'd0)
      FrwB = 2'b01;
    else if(MEM_WB_RegWrite == 1 && MEM_WB_Rd == ID_EX_Rt && MEM_WB_Rd != 5'd0)
      FrwB = 2'b10;
  end
endmodule

module HazardUnit(PCrst, ID_EX_MemRead, ID_EX_Rt, Rt, Rs, PcWrite, IF_ID_write, SignalSel);
  input PCrst, ID_EX_MemRead;
  input[4:0] ID_EX_Rt, Rt, Rs;
  output PcWrite, IF_ID_write, SignalSel;
  reg PcWrite, IF_ID_write, SignalSel;
  
  always@(ID_EX_MemRead, ID_EX_Rt, Rt, Rs, PCrst)begin
    {SignalSel, IF_ID_write, PcWrite} = 3'b111;
    if(ID_EX_MemRead == 1'b1 && ID_EX_Rt != 5'd0)begin
      if(ID_EX_Rt == Rs || ID_EX_Rt == Rt)begin
        IF_ID_write = 1'b0;
        PcWrite = 1'b0;
        SignalSel = 1'b0;
      end
    end
  end
endmodule

module Datapath(clk, rst, PCsrc, jsel, RegDst, ALUsrc, MemWrite, MemRead, MemtoReg, RegWrite, clr, jalRegSel, jalWriteSel, ALUoperation, 
  func, opcode, eq);
  input clk, rst, PCsrc;
  input[1:0] jsel;
  input RegDst, ALUsrc, MemWrite, MemRead, MemtoReg, RegWrite, clr, jalRegSel, jalWriteSel;
  input [2:0]ALUoperation;
  output[5:0] func, opcode;
  output eq;
  
  wire[10:0] controlSignals;
  assign controlSignals = {ALUoperation, RegDst, ALUsrc, jalRegSel, MemWrite, MemRead, MemtoReg, RegWrite, jalWriteSel};
  wire PCld, IF_ID_Write, bubbleSel;
  wire[31:0] pc, nextPC, pcPlus4, Instruction, IF_ID_pc, IF_ID_inst, writeData, readData1, readData2;
  wire[31:0] extendedData, MEM_WB_readData, shifted32, branchAdr, jumpAdr, ID_EX_jalPC, EX_MEM_jalPC, MEM_WB_jalPC;
  wire[31:0] ID_EX_readData1, ID_EX_readData2, ID_EX_extended, EX_MEM_ALURes, MemtoRegOut, A, B, forwardBOut, ALURes, EX_MEM_writeData;
  wire[31:0] readDataMemory, MEM_WB_ALURes;
  wire[2:0] MEM_WB_cntrl;
  wire[4:0] MEM_WB_rd, ID_EX_Rt, ID_EX_Rs, ID_EX_Rd, EX_MEM_Rd, EX_MEM_cntrl, dstRegOut, jalRegOut;
  wire[27:0] shifted28;
  wire[10:0] signals, ID_EX_cntrl;
  wire[1:0] ForwardA, ForwardB;
  
  PCreg PC(pc, clk, PCld, rst, nextPC);
  Adder pcAdder(nextPC, 32'd4, pcPlus4);
  InstructionMemory InstMem(nextPC,Instruction);
  IF_ID_reg IF_ID(pcPlus4, Instruction, clk, IF_ID_Write,clr, IF_ID_pc, IF_ID_inst);
  MUX2to1 #32 JAL_WRITE_MUX(MemtoRegOut, MEM_WB_jalPC, MEM_WB_cntrl[0], writeData);
  RegisterFile RegFile(clk, MEM_WB_cntrl[1], IF_ID_inst[25:21], IF_ID_inst[20:16], MEM_WB_rd, writeData, readData1, readData2);
  SignExtend SignExt(IF_ID_inst[15:0], extendedData);
  ShiftLeft32 Shifter32(extendedData, shifted32);
  Adder BranchAdder(IF_ID_pc, shifted32, branchAdr);
  ShiftLeft26to28 shifter26to28(IF_ID_inst[25:0],shifted28);
  MUX3to1 #32 JselMUX(branchAdr, {nextPC[31:28], shifted28}, readData1, jsel, jumpAdr);
  MUX2to1 #32 PCSrcMUX(pcPlus4, jumpAdr, PCsrc, pc);
  checkEqual EQ(readData1, readData2, eq);
  MUX2to1 #11 Bubble_MUX(11'd0, controlSignals, bubbleSel, signals);
  ID_EX_reg ID_EX(signals, readData1, readData2, extendedData, IF_ID_pc, IF_ID_inst[20:16], IF_ID_inst[25:21],  IF_ID_inst[15:11], clk, ID_EX_cntrl, ID_EX_readData1, ID_EX_readData2, ID_EX_extended, ID_EX_jalPC, ID_EX_Rt, ID_EX_Rs, ID_EX_Rd);
  HazardUnit Hazard_unit(rst, ID_EX_cntrl[3], ID_EX_Rt, IF_ID_inst[20:16], IF_ID_inst[25:21], PCld, IF_ID_Write, bubbleSel);
  ForwardUnit Forward_unit(ID_EX_Rs, ID_EX_Rt, EX_MEM_Rd, MEM_WB_rd, EX_MEM_cntrl[1], MEM_WB_cntrl[1], ForwardA, ForwardB);
  MUX3to1 #32 ALUMUXA(ID_EX_readData1, EX_MEM_ALURes, MemtoRegOut, ForwardA, A);
  MUX3to1 #32 ALUMUX(ID_EX_readData2, EX_MEM_ALURes, MemtoRegOut, ForwardB, forwardBOut);
  MUX2to1 #32 ALUMUXB(forwardBOut, ID_EX_extended, ID_EX_cntrl[6], B);
  ALU ALU(A, B, ID_EX_cntrl[10:8], ALURes);
  MUX2to1 #5 DstReg(ID_EX_Rd, ID_EX_Rt, ID_EX_cntrl[7], dstRegOut);
  MUX2to1 #5 JAL_REG(dstRegOut, 32'd31, ID_EX_cntrl[5], jalRegOut);
  EX_MEM_reg EX_MEM(ID_EX_cntrl[4:0], ALURes, forwardBOut, ID_EX_jalPC, jalRegOut, clk, EX_MEM_cntrl, EX_MEM_ALURes, EX_MEM_writeData, EX_MEM_jalPC, EX_MEM_Rd);
  DataMemory DataMemory(EX_MEM_ALURes, EX_MEM_writeData, clk, EX_MEM_cntrl[3], EX_MEM_cntrl[4], readDataMemory);
  MEM_WB_reg MEM_WB(EX_MEM_cntrl[2:0], EX_MEM_ALURes, readDataMemory, EX_MEM_jalPC, EX_MEM_Rd, clk, MEM_WB_cntrl, MEM_WB_ALURes, MEM_WB_readData,MEM_WB_jalPC, MEM_WB_rd);
  MUX2to1 #32 memToRegMux(MEM_WB_ALURes, MEM_WB_readData, MEM_WB_cntrl[2], MemtoRegOut);
  assign opcode = IF_ID_inst[31:26];
  assign func = IF_ID_inst[5:0];
endmodule
