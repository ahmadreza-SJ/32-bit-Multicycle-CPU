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

module myAlu_test;

	parameter word_size = 32;

	// inputs
	reg [word_size-1:0] sourceA, sourceB;
	reg [3:0] ALUSel;

	// outputs
	wire  [word_size-1:0] output_data;
	wire  zero;


	// Instantiate the Unit Under Test (UUT)
	myALU myalu_unit (
		.sourceA(sourceA), 
		.sourceB(sourceB), 
		.ALUSel(ALUSel),
		.output_data(output_data), 
		.zero(zero)
	);

	initial begin
		// Initialize Inputs
		sourceA = 4;
		sourceB = 5;
		ALUSel = 1;

		$display("at time %t , sourceA = %b, sourceB = %b, alusel = %b, output_data=%b, zero=%b", $time,  sourceA, sourceB, ALUSel,output_data, zero );

		// Wait 100 ns for global reset to finish
		#10;
    	sourceA = 5;
		sourceB = 4;
		ALUSel = 5;

		$display("at time %t , sourceA = %b, sourceB = %b, alusel = %b, output_data=%b, zero=%b", $time,  sourceA, sourceB, ALUSel,output_data, zero );

		#10;
    	sourceA = 12;
		sourceB = 10;
		ALUSel = 5;

		$display("at time %t , sourceA = %b, sourceB = %b, alusel = %b, output_data=%b, zero=%b", $time,  sourceA, sourceB, ALUSel,output_data, zero );

		#10;
    	sourceA = 5;
		sourceB = 4;
		ALUSel = 3;

		$display("at time %t , sourceA = %b, sourceB = %b, alusel = %b, output_data=%b, zero=%b", $time,  sourceA, sourceB, ALUSel,output_data, zero );

		#10;
    	sourceA = 8;
		sourceB = 6;
		ALUSel = 1;

		$display("at time %t , sourceA = %b, sourceB = %b, alusel = %b, output_data=%b, zero=%b", $time,  sourceA, sourceB, ALUSel,output_data, zero );
	end
      
endmodule

