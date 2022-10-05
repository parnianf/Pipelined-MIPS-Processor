`timescale 1ns/1ns
module Controller(opcode, func, eq, RegDst, ALUsrc, MemWrite, MemRead, MemtoReg, RegWrite, jsel, PCsrc, clr, jalRegSel,jalWriteSel, ALUoperation);
  input[5:0] opcode, func;
  input eq;
  output RegDst,ALUsrc,MemWrite,MemRead,MemtoReg,RegWrite;
  reg RegDst,ALUsrc,MemWrite,MemRead,MemtoReg,RegWrite;
  output[1:0] jsel;
  reg[1:0] jsel;
  output PCsrc;
  reg PCsrc;
  output clr;
  output jalRegSel, jalWriteSel;
  reg jalRegSel, jalWriteSel;
  output [2:0] ALUoperation;
  reg[2:0] ALUoperation;
  
  reg[1:0] ALUop;
  reg Branch;
  always @(opcode, func, eq)begin
    {RegDst, RegWrite, ALUsrc, MemtoReg, MemWrite, MemRead, Branch, jalRegSel, jalWriteSel, jsel, PCsrc, ALUop} = 15'd0;
    case(opcode)
      6'b000000: begin
        if(func != 6'b001000)begin
          RegWrite = 1'b1;
          ALUop = 2'b10;
        end
        else begin
        jsel = 2'b10;
        PCsrc = 1'b1;
        end
      end
      6'b100011: begin//lw
        {ALUsrc, RegDst, MemRead, MemtoReg, RegWrite} = 5'b11111;
      end
      6'b101011: begin//sw
       {ALUsrc, MemWrite} = 2'b11;
      end
      6'b000010: begin//j
        jsel = 2'b01;
        PCsrc = 1'b1;
      end
      6'b000011: begin//jal
        {RegWrite, jalRegSel, jalWriteSel, PCsrc} = 4'b1111;
        jsel = 2'b01;
      end
      6'b000100: begin//beq
        Branch = 1'b1;
        ALUop = 2'b01;
        PCsrc = eq;
      end
      6'b000101: begin//bne
        Branch = 1'b1;
        ALUop = 2'b01;
        PCsrc = ~eq;
      end
      6'b001000: begin//addi
       {ALUsrc, RegDst, RegWrite} = 3'b111;
      end
      6'b001010: begin//slti
       {ALUsrc, RegDst, RegWrite} = 3'b111;
        ALUop = 2'b11;
      end
    endcase
  end
  
  always@(ALUop,func)begin
    ALUoperation = 3'b101;
    case (ALUop)
      2'b00: begin
        ALUoperation = 3'b010;
      end
      2'b01: begin
        ALUoperation = 3'b110;
      end
      2'b10: begin
        case(func)
        6'b100000: ALUoperation = 3'b010;
        6'b100010: ALUoperation = 3'b110;
        6'b100100: ALUoperation = 3'b000;
        6'b100101: ALUoperation = 3'b001;
        6'b101010: ALUoperation = 3'b111;
        endcase
      end 
      2'b11: begin
        ALUoperation = 3'b111;
      end 
    endcase
  end
  assign clr = ((opcode == 6'b000100) && (eq==1'b1)) || (opcode == 6'b000101 && eq==1'b0) || (jsel != 2'b00);

endmodule

