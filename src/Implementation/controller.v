`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer: John Careaga
//
// Create Date:    05:58:01 11/24/2014
// Design Name:
// Module Name:    controller
// Project Name:
// Target Devices:
// Tool versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////////////

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
// ~~~~~~~~~~~~~~~~~~~ CONTROLLER ~~~~~~~~~~~~~~~~~~~ //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

module controller(opcode, clk, reset, PCWrite, PCWriteCond, DMEMWrite, IRWrite,
                   MemtoReg, PCSource, ALUSel, ALUSrcA, ALUSrcB, RegWrite,
                   RegReadSel);

  // ~~~~~~~~~~~~~~~~~~~ PORTS ~~~~~~~~~~~~~~~~~~~ //

  // opcode, clock, and reset inputs
  input [5:0] opcode;	// from instruction register
  input	clk, reset;

  // control signal outputs
  output reg PCWrite, PCWriteCond, DMEMWrite, IRWrite, ALUSrcA, RegWrite, RegReadSel;
  output reg [1:0] MemtoReg, PCSource, ALUSrcB;
  output reg [3:0] ALUSel;

  // ~~~~~~~~~~~~~~~~~~~ REGISTER ~~~~~~~~~~~~~~~~~~~ //

  // 4-bit state register
  reg [3:0]	state;

  // ~~~~~~~~~~~~~~~~~~~ PARAMETERS ~~~~~~~~~~~~~~~~~~~ //

  // state parameters
  parameter IF  = 4'd0;
  parameter ID  = 4'd1;
  parameter R_type_ex  = 4'd2;
  parameter L_type_sign_ex  = 4'd3;
  parameter L_type_zero_ex  = 4'd4;
  parameter LWI_mem_read  = 4'd5;
  parameter reg_write_back_aluout  = 4'd6;
  parameter reg_write_back_mdr  = 4'd7;
  parameter swi_mem_write  = 4'd8;
  parameter reg_write_back_li  = 4'd9;
  parameter reg_write_back_lui = 4'd10;
  parameter branch = 4'd11;
  parameter jump = 4'd12;
  parameter sR  = 4'd13;	// reset
  parameter R1_output = 4'd14;

  // opcode[5:4] parameters
  parameter J  = 2'b00;	// Jump or NOP
  parameter R  = 2'b01;	// R-type
  parameter BR = 2'b10;	// Branch
  parameter I  = 2'b11;	// I-type

  // I-type opcode[3:0] variations
  parameter ADDI = 4'b0010;
  parameter SUBI = 4'b0011;
  parameter ORI	 = 4'b0100;
  parameter ANDI = 4'b0101;
  parameter XORI = 4'b0110;
  parameter SLTI = 4'b0111;
  parameter LI	 = 4'b1001;
  parameter LUI	 = 4'b1010;
  parameter LWI	 = 4'b1011;
  parameter SWI	 = 4'b1100;

  // ~~~~~~~~~~~~~~~~~~~ STATE MACHINE ~~~~~~~~~~~~~~~~~~~ //

  // control state machine
  always @(posedge clk) begin
    // check for reset signal. If set, write zero to PC and switch to Reset State on next CC.
    if (reset) begin
      PCWrite 		<= 1;
      PCWriteCond <= 0;
      DMEMWrite 	<= 0;
      IRWrite 		<= 0;
      MemtoReg 		<= 0;
      PCSource 		<= 2'b11; // reset
      ALUSel 			<= 0;
      ALUSrcA 		<= 0;
      ALUSrcB 		<= 0;
      RegWrite 		<= 0;

      state <= sR;
    end
    else begin	// if reset signal is not set, check state at pos edge
      case (state)

        // if in reset state (and reset signal not set), go to IF (IF)
        sR: begin
          PCWrite 		<= 1;
          DMEMWrite 	<= 0;
          IRWrite 		<= 1;
          PCSource 		<= 2'b00;
          ALUSel 			<= 4'b0010;
          ALUSrcA 		<= 0;
          ALUSrcB 		<= 2'b01;
          RegWrite 		<= 0;
          PCWriteCond <= 0;

          state <= IF;
        end

        // if in IF, go to ID (ID)
        IF: begin
          PCWrite 		<= 0;
          DMEMWrite 	<= 0;
          IRWrite 		<= 0;
          ALUSel 			<= 4'b0010;
          ALUSrcA 		<= 0;
          ALUSrcB 		<= 2'b10;
          RegWrite 		<= 0;
          RegReadSel	<= 0;

          state <= ID;
        end

        // if in ID (ID) check opcode from instruction register to determine new state
        ID: begin
          case (opcode[5:4])
            // R-type opcode: go to R_type_ex (R-type EX)
            // opcode[5:4] = 01 => Arithmetic/Logical R
            R: begin
              PCWrite 		<= 0;
              DMEMWrite 	<= 0;
              IRWrite 		<= 0;
              ALUSel 			<= opcode[3:0];
              ALUSrcA 		<= 1;
              ALUSrcB 		<= 2'b00;
              RegWrite 		<= 0;

              state <= R_type_ex;
            end

            // J-type or NOP
            J: begin
              // NOP: do nothing and go back to IF (IF) for next instruction
              if (opcode[3:0] == 4'b0000) begin
                PCWrite 		<= 1;
                DMEMWrite 	<= 0;
                IRWrite 		<= 1;
                PCSource 		<= 2'b00;
                ALUSel 			<= 4'b0010;
                ALUSrcA 		<= 0;
                ALUSrcB 		<= 2'b01;
                RegWrite 		<= 0;
                PCWriteCond <= 0;

                state	<= 0;
              end
              // Jump: go to jump (jump completion)
              else begin
                PCWrite 		<= 1;
                DMEMWrite 	<= 0;
                IRWrite 		<= 0;
                PCSource 		<= 2'b10;
                RegWrite 		<= 0;

                state <= jump;
              end
            end

            // Branch: go to R1_output ($R1 read)
            BR: begin
              PCWrite 		<= 0;
              DMEMWrite 	<= 0;
              IRWrite 		<= 0;
              ALUSel 			<= 4'b0010;
              ALUSrcA 		<= 0;
              ALUSrcB 		<= 2'b10;
              RegWrite 		<= 0;
              RegReadSel	<= 1; // for R1

              state <= R1_output;
            end

            // I-type
            I: begin
            // go to L_type_sign_ex (EX for ALU I-type with sign extended immediate)
              if ((opcode[3:0] == ADDI) || (opcode[3:0] == SUBI) || (opcode[3:0] == SLTI)) begin
                PCWrite 		<= 0;
                DMEMWrite 	<= 0;
                IRWrite 		<= 0;
                ALUSel 			<= opcode[3:0];
                ALUSrcA 		<= 1;
                ALUSrcB 		<= 2'b10;
                RegWrite 		<= 0;

                state <= L_type_sign_ex;
              end

              // go to L_type_zero_ex (EX for ALU I-type with zero extended immediate)
              else if ((opcode[3:0] == ORI) || (opcode[3:0] == ANDI) || (opcode[3:0] == XORI)) begin
                PCWrite 		<= 0;
                DMEMWrite 	<= 0;
                IRWrite 		<= 0;
                ALUSel 			<= opcode[3:0];
                ALUSrcA 		<= 1;
                ALUSrcB 		<= 2'b11;
                RegWrite 		<= 0;

                state <= L_type_zero_ex;
              end

                // go to LWI_mem_read (MEM access for LWI)
              else if (opcode[3:0] == LWI) begin
                PCWrite 		<= 0;
                DMEMWrite 	<= 0;
                IRWrite 		<= 0;
                RegWrite 		<= 0;

                state <= LWI_mem_read;
              end
              // go to R1_output for R1 read
              else if ((opcode[3:0] == LI) || (opcode[3:0] == LUI) || (opcode[3:0] == SWI))begin
                PCWrite 		<= 0;
                DMEMWrite 	<= 0;
                IRWrite 		<= 0;
                ALUSel 			<= 4'b0010;
                ALUSrcA 		<= 0;
                ALUSrcB 		<= 2'b10;
                RegWrite 		<= 0;
                RegReadSel	<= 1; // for R1

                state <= R1_output;
              end
            end
          endcase
        end

        // if in R_type_ex (R-type EX) go to reg_write_back_aluout (ALUOp write backs)
        R_type_ex: begin
          PCWrite 		<= 0;
          DMEMWrite 	<= 0;
          IRWrite 		<= 0;
          MemtoReg 		<= 2'b00;
          RegWrite 		<= 1;

          state <= reg_write_back_aluout;
        end

        // if in L_type_sign_ex (EX for Arithmetic I-type with sign extended Imm) go to reg_write_back_aluout (ALUOp WB)
        L_type_sign_ex: begin
          PCWrite 		<= 0;
          DMEMWrite 	<= 0;
          IRWrite 		<= 0;
          MemtoReg 		<= 2'b00;
          RegWrite 		<= 1;

          state <= reg_write_back_aluout;
        end

        // if in L_type_zero_ex (EX for Arithmetic I-type with zero extended Imm) go to reg_write_back_aluout (ALUOp WB)
        L_type_zero_ex: begin
          PCWrite 		<= 0;
          DMEMWrite 	<= 0;
          IRWrite 		<= 0;
          MemtoReg 		<= 2'b00;
          RegWrite 		<= 1;

          state <= reg_write_back_aluout;
        end

        // if in LWI_mem_read (LWI MEM access) go to reg_write_back_mdr (Reg File WB for LWI)
        LWI_mem_read: begin
          PCWrite 		<= 0;
          DMEMWrite 	<= 0;
          IRWrite 		<= 0;
          MemtoReg 		<= 2'b01;
          RegWrite 		<= 1;

          state <= reg_write_back_mdr;
        end

        // if in reg_write_back_aluout (ALUOut WB) go back to IF (IF)
        reg_write_back_aluout: begin
          PCWrite 		<= 1;
          DMEMWrite 	<= 0;
          IRWrite 		<= 1;
          PCSource 		<= 2'b00;
          ALUSel 			<= 4'b0010;
          ALUSrcA 		<= 0;
          ALUSrcB 		<= 2'b01;
          RegWrite 		<= 0;
          PCWriteCond <= 0;

          state <= IF;
        end

        // if in reg_write_back_mdr (Reg Gile WB for LWI) go to IF (IF)
        reg_write_back_mdr: begin
          PCWrite 		<= 1;
          DMEMWrite 	<= 0;
          IRWrite 		<= 1;
          PCSource 		<= 2'b00;
          ALUSel 			<= 4'b0010;
          ALUSrcA 		<= 0;
          ALUSrcB 		<= 2'b01;
          RegWrite 		<= 0;
          PCWriteCond <= 0;

          state <= IF;
        end

        // if in swi_mem_write (MEM write for SWI) go to IF (IF)
        swi_mem_write: begin
          PCWrite 		<= 1;
          DMEMWrite 	<= 0;
          IRWrite 		<= 1;
          PCSource 		<= 2'b00;
          ALUSel 			<= 4'b0010;
          ALUSrcA 		<= 0;
          ALUSrcB 		<= 2'b01;
          RegWrite 		<= 0;
          PCWriteCond <= 0;

          state <= IF;
        end

        // if in reg_write_back_li (Reg WB for LI) go to IF (IF)
        reg_write_back_li: begin
          PCWrite 		<= 1;
          DMEMWrite 	<= 0;
          IRWrite 		<= 1;
          PCSource 		<= 2'b00;
          ALUSel 			<= 4'b0010;
          ALUSrcA 		<= 0;
          ALUSrcB 		<= 2'b01;
          RegWrite 		<= 0;
          PCWriteCond <= 0;

          state <= IF;
        end

        // if in reg_write_back_lui (Reg WB for LUI) go to IF (IF)
        reg_write_back_lui: begin
          PCWrite 		<= 1;
          DMEMWrite 	<= 0;
          IRWrite 		<= 1;
          PCSource 		<= 2'b00;
          ALUSel 			<= 4'b0010;
          ALUSrcA 		<= 0;
          ALUSrcB 		<= 2'b01;
          RegWrite 		<= 0;
          PCWriteCond <= 0;

          state <= IF;
        end

        // if in branch (Branch completion) go to IF (IF)
        branch: begin
          PCWrite 		<= 1;
          DMEMWrite 	<= 0;
          IRWrite 		<= 1;
          PCSource 		<= 2'b00;
          ALUSel 			<= 4'b0010;
          ALUSrcA 		<= 0;
          ALUSrcB 		<= 2'b01;
          RegWrite 		<= 0;
          PCWriteCond <= 0;

          state <= IF;
        end

        // if in jump (Jump completion) go to IF (IF)
        jump: begin
          PCWrite 		<= 1;
          DMEMWrite 	<= 0;
          IRWrite 		<= 1;
          PCSource 		<= 2'b00;
          ALUSel 			<= 4'b0010;
          ALUSrcA 		<= 0;
          ALUSrcB 		<= 2'b01;
          RegWrite 		<= 0;
          PCWriteCond <= 0;

          state <= IF;
        end

        // if in R1 read
        R1_output: begin
          // if Branch, go to branch, branch completion
          if (opcode[5:4] == BR) begin
            PCWrite 		<= 0;
            PCWriteCond <= 1;
            DMEMWrite 	<= 0;
            IRWrite 		<= 0;
            PCSource 		<= 2'b01;
            ALUSel 			<= 4'b0011;
            ALUSrcA 		<= 1;
            ALUSrcB 		<= 2'b00;
            RegWrite 		<= 0;
            RegReadSel	<= 1;

            state <= branch;
          end
          // if LI, go to reg_write_back_li LI WB
          if (opcode[3:0] == LI) begin
            PCWrite 		<= 0;
            DMEMWrite 	<= 0;
            IRWrite 		<= 0;
            MemtoReg 		<= 2'b10;
            RegWrite 		<= 1;

            state <= reg_write_back_li;
          end
          // if LUI, go to reg_write_back_lui LUI WB
          else if (opcode[3:0] == LUI) begin
            PCWrite 		<= 0;
            DMEMWrite 	<= 0;
            IRWrite 		<= 0;
            MemtoReg 		<= 2'b11;
            RegWrite 		<= 1;

            state <= reg_write_back_lui;
          end
          // if SWI, go to swi_mem_write, Mem Access
          else if (opcode[3:0] == SWI) begin
            PCWrite 		<= 0;
            DMEMWrite 	<= 1;
            IRWrite 		<= 0;
            RegWrite 		<= 0;

            state <= swi_mem_write;
          end
        end
        // go to IF
        default: begin
          PCWrite 		<= 1;
          DMEMWrite 	<= 0;
          IRWrite 		<= 1;
          PCSource 		<= 2'b00;
          ALUSel 			<= 4'b0010;
          ALUSrcA 		<= 0;
          ALUSrcB 		<= 2'b01;
          RegWrite 		<= 0;
          PCWriteCond <= 0;

          state <= IF;
        end
      endcase
    end
  end
endmodule