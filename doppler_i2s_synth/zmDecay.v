


//------------------------------------------------------------------------------
//          Decay , zMors
//------------------------------------------------------------------------------
module DECAY( decay_time, decayout,clk,trigger);
	input clk;
	input trigger;
	input [15:0] decay_time;
	output [15:0] decayout;

	reg [15:0] ramp_down;
	reg [31:0] dcy_downsample;
	reg [15:0] dcycount;

//	assign decayout = dcy_downsample[31:16];
	assign decayout = ramp_down;

	reg dclk;
	always @(posedge clk) begin
		if(dcycount == 0) begin
			dcycount <= decay_time[15:4];
			dclk <= 1;
		end else begin
			dcycount <= dcycount - 1;
			dclk <= 0;
		end
	end

	always @(posedge dclk or posedge trigger) begin
		if(!trigger ) begin
			if(ramp_down > 1)
			ramp_down <= ramp_down - 1;
		end else begin
			ramp_down <= 16'h7ffc;
		end
	end


	SB_MAC16 decay_mul (
	    .A(ramp_down),
	    .B(ramp_down),
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
	    // .SIGNEXTOUT(dcy_dsp_ready)
	    .O(dcy_downsample)
	  );

	//16x16 => 32 unsigned pipelined multiply
	defparam decay_mul.B_SIGNED                  = 1'b0;
	defparam decay_mul.A_SIGNED                  = 1'b0;
	defparam decay_mul.MODE_8x8                  = 1'b0;

	defparam decay_mul.BOTADDSUB_CARRYSELECT     = 2'b00;
	defparam decay_mul.BOTADDSUB_UPPERINPUT      = 1'b0;
	defparam decay_mul.BOTADDSUB_LOWERINPUT      = 2'b00;
	defparam decay_mul.BOTOUTPUT_SELECT          = 2'b11;

	defparam decay_mul.TOPADDSUB_CARRYSELECT     = 2'b00;
	defparam decay_mul.TOPADDSUB_UPPERINPUT      = 1'b0;
	defparam decay_mul.TOPADDSUB_LOWERINPUT      = 2'b00;
	defparam decay_mul.TOPOUTPUT_SELECT          = 2'b11;

	defparam decay_mul.PIPELINE_16x16_MULT_REG2  = 1'b1;
	defparam decay_mul.PIPELINE_16x16_MULT_REG1  = 1'b1;
	defparam decay_mul.BOT_8x8_MULT_REG          = 1'b1;
	defparam decay_mul.TOP_8x8_MULT_REG          = 1'b1;
	defparam decay_mul.D_REG                     = 1'b0;
	defparam decay_mul.B_REG                     = 1'b1;
	defparam decay_mul.A_REG                     = 1'b1;
	defparam decay_mul.C_REG                     = 1'b0;

endmodule
