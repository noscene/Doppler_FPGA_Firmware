
//------------------------------------------------------------------------------
//          VCA , zMors
//------------------------------------------------------------------------------
module VCA( vca_in_a, vca_in_b, vca_out,clk);
	input clk;
	input [15:0] vca_in_a;
	input [15:0] vca_in_b;
	output [15:0] vca_out;

	wire  [31:0] downsample;

	assign vca_out = downsample[31:16] <<< 1;		// TODO: check this

	SB_MAC16 vca_mul (
	    .A(vca_in_a[15:0]),
	    .B(vca_in_b[15:0]),
	    .C(16'b0),
	    .D(16'b0),
	    .CLK(clk),
	    .CE(1'b1),
	    .IRSTTOP(1'b0),	/* reset */
	    .IRSTBOT(1'b0), /* reset */
	    .ORSTTOP(1'b0), /* reset */
	    .ORSTBOT(1'b0), /* reset */
	    .AHOLD(1'b0),
	    .BHOLD(1'b0),
	    .CHOLD(1'b0),
	    .DHOLD(1'b0),
	    .OHOLDTOP(1'b0),
	    .OHOLDBOT(1'b0),
	    .OLOADTOP(1'b0),
	    .OLOADBOT(1'b0),
	    .ADDSUBTOP(1'b0),
	    .ADDSUBBOT(1'b0),
	    .CO(),
	    .CI(1'b0),
	    .O(downsample)
	  );

	//16x16 => 32 unsigned pipelined multiply
	defparam vca_mul.B_SIGNED                  = 1'b1;	// TODO: check this
	defparam vca_mul.A_SIGNED                  = 1'b1;	// TODO: check this
	defparam vca_mul.MODE_8x8                  = 1'b0;

	defparam vca_mul.BOTADDSUB_CARRYSELECT     = 2'b00;
	defparam vca_mul.BOTADDSUB_UPPERINPUT      = 1'b0;
	defparam vca_mul.BOTADDSUB_LOWERINPUT      = 2'b00;
	defparam vca_mul.BOTOUTPUT_SELECT          = 2'b11;

	defparam vca_mul.TOPADDSUB_CARRYSELECT     = 2'b00;
	defparam vca_mul.TOPADDSUB_UPPERINPUT      = 1'b0;
	defparam vca_mul.TOPADDSUB_LOWERINPUT      = 2'b00;
	defparam vca_mul.TOPOUTPUT_SELECT          = 2'b11;

	defparam vca_mul.PIPELINE_16x16_MULT_REG2  = 1'b1;
	defparam vca_mul.PIPELINE_16x16_MULT_REG1  = 1'b1;
	defparam vca_mul.BOT_8x8_MULT_REG          = 1'b1;
	defparam vca_mul.TOP_8x8_MULT_REG          = 1'b1;
	defparam vca_mul.D_REG                     = 1'b0;
	defparam vca_mul.B_REG                     = 1'b1;
	defparam vca_mul.A_REG                     = 1'b1;
	defparam vca_mul.C_REG                     = 1'b0;

endmodule
