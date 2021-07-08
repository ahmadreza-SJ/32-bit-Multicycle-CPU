`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   23:52:28 11/22/2014
// Design Name:   jumpALU
// Module Name:   /ad/eng/users/j/o/johnc219/EC413/MultiCycleCPU/jumpALU_test.v
// Project Name:  MultiCycleCPU
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: jumpALU
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module Imem_test;

	parameter word_size = 32;

	// inputs
	reg [31:0] PC;
  
	wire [31:0] Instruction;


	// Instantiate the Unit Under Test (UUT)
	IMem imem_unit(
		.PC(PC),          // PC (address) of instruction in IMem
        .Instruction(Instruction)
		);

	initial begin
		// Initialize Inputs
		PC = 0;		
		$display("at time %t , PC = %b, instruction = %b", $time,  PC, Instruction);

		// Wait 100 ns for global reset to finish
		#10;
		PC = 2;		
		$display("at time %t , PC = %b, instruction = %b", $time,  PC, Instruction);

		#10;
    	PC = 4;		
		$display("at time %t , PC = %b, instruction = %b", $time,  PC, Instruction);
		
		#10;
    	PC = 5;		
		$display("at time %t , PC = %b, instruction = %b", $time,  PC, Instruction);

		#10;
    	PC = 6;		
		$display("at time %t , PC = %b, instruction = %b", $time,  PC, Instruction);
	end
      
endmodule

