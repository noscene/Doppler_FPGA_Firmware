//------------------------------------------------------------------------------
//          SUPERSAW Oscillator
//------------------------------------------------------------------------------
module SUPERSAW( clk,pitch, audio_out);
	input clk;
	input [15:0] pitch;
	wire [31:0]  pitch_int ;
	output [15:0]	audio_out;
	reg [31:0] ramp1,ramp2,ramp3,ramp4,ramp5,ramp6,ramp7,ramp8;
	wire [10:0] s1,s2,s3,s4,s5,s6,s7,s8;
	assign s1 = ramp1[31:17];
	assign s2 = ramp2[31:17];
	assign s3 = ramp3[31:17];
	assign s4 = ramp4[31:17];
	assign s5 = ramp5[31:17];
	assign s6 = ramp6[31:17];
	assign s7 = ramp7[31:17];
	assign s8 = ramp8[31:17];

  // shift pitch 8 bits
	assign pitch_int[31:0] = {8'd0, pitch[15:0], 8'd0};

	always @(posedge clk)
	begin
		ramp1 <= ramp1 + pitch_int +  50000 ;
		ramp2 <= ramp2 + pitch_int +  51333 ;
		ramp3 <= ramp3 + pitch_int +  52777 ;
		ramp4 <= ramp4 + pitch_int +  53077 ;
		ramp5 <= ramp5 + pitch_int +  54577 ;
		ramp6 <= ramp6 + pitch_int +  55336 ;
		ramp7 <= ramp7 + pitch_int +  56789 ;
		ramp8 <= ramp8 + pitch_int +  57612 ;
	end
	assign audio_out = s1 + s2 + s3 + s4 + s5 + s6 + s7 + s8;
endmodule
