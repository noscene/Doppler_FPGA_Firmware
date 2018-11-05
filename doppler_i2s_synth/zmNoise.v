//------------------------------------------------------------------------------
//          LFSR Noise
//------------------------------------------------------------------------------
module NOISE( clk, audio_out);
	input clk;
	output [15:0]	audio_out;
	parameter SEED = 32'b10101011101010111010101110101011; // LFSR starting state
	parameter TAPS = 31'b0000000000000000000000001100010;  // LFSR feedback taps
	reg [31:0] shift_register;
	initial shift_register = SEED;
	always @(posedge clk)
	begin
		if(shift_register[31]) begin
			shift_register[31:1] <= shift_register[30:0]^TAPS;
		end else begin
			shift_register[31:1] <= shift_register[30:0];
		end
		shift_register[0] <= shift_register[31];
	end
	assign audio_out = shift_register[31:16];
endmodule
