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

module read_mux_test;
	parameter word_size = 5;

	// inputs
	reg select;
	reg [word_size-1:0] input_data0, input_data1;

	// output
	wire [word_size-1:0] output_data;

	// Instantiate the Unit Under Test (UUT)
	read_mux uut (
		.output_data(output_data), 
		.input_data0(input_data0), 
		.input_data1(input_data1), 
		.select(select)
	);

	initial begin
		// Initialize Inputs
		input_data0 = 0;
		input_data1 = 0;
		select = 0;
		$display("at time %t , input_data0 = %b, input_data1 = %b, select = %b", $time,  input_data0, input_data1, select);

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here
		select = 1;		// change selected input
		input_data0 = 32'hFFFF0000;	// input 0
		input_data1 = 32'h0000FFFF;	// input 1
		$display("at time %t , input_data0 = %b, input_data1 = %b, select = %b", $time,  input_data0, input_data1, select);

		
		#20;
		select = 1;		// change selected input
		input_data0 = 32'h88888888;	// change input 0
		input_data1 = 32'hFEFEFEFE;	// change input 1
		$display("at time %t , input_data0 = %b, input_data1 = %b, select = %b", $time,  input_data0, input_data1, select);
	end
      
endmodule

