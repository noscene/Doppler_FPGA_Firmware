`default_nettype none


//
//		Sample PCM5102 Module
//    output noise and supersaw with pitch control via SPI
/*
// Arduino Sound Demo
#include <ICEClass.h>
#include "/Users/svenbraun/Documents/GitHub/Doppler_FPGA_Firmware/PCM5102/PCM5102.h"
ICEClass ice40;
void setup() {
  ice40.upload(PCM5102_bin,sizeof(PCM5102_bin)); // Upload BitStream Firmware to FPGA -> see variant.h
  ice40.initSPI();  // start SPI runtime Link to FPGA
}
// send random pitches
void loop() {
  ice40.sendSPI16(0x300);   delay(400);
  ice40.sendSPI16(0x600);   delay(200);
  ice40.sendSPI16(0x900);   delay(400);
  ice40.sendSPI16(0x1200);  delay(600);
}
*/


module top( output I2S_BCLK,output I2S_DATA,output I2S_LR,
            input sck,mosi,cs, output miso,
            output  [3:0] kled  , output [3:0]  aled);

  wire clk48;
	SB_HFOSC inthosc ( .CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk48) );



  // SPI Slave
  reg [15:0]  data_to_sam=0;
  wire [15:0] data_from_sam;
  wire [14:0] bit_counter;
  reg [15:0] wavetable [0:256];
  SPI_SLAVE spi(.clk(clk48),
                .sck(sck),.miso(miso),.mosi(mosi),.cs(cs),
                .data_from_sam(data_from_sam),.data_to_sam(data_to_sam),
                .bit_counter(bit_counter));
/*
  reg [23:0] teiler;
  always @ ( posedge clk48 ) begin
    teiler <= teiler +1;
  end

  reg [7:0] RADDR_c ;
  always @ ( posedge teiler[12] ) begin

  end

  wire [15:0] RDATA_c;
  SB_RAM40_4K wtable (
                .RDATA(RDATA_c[15:0]),
                .RADDR(RADDR_c[7:0]),
                .RCLK(clk48),
                .RCLKE(1'b1),
                .RE(1'b1),
                .WADDR(bit_counter[14:8]),
                .WCLK(bit_counter[3]),
                .WCLKE(1'b1),
                .WDATA(data_from_sam),
                .WE(1'b1),    // write enable
                .MASK(16'b0)  // no mask
                );

*/

wire [15:0] RDATA_c;
filter_svf  #(
  .SAMPLE_BITS(16)
) vcf (
  .clk(~I2S_LR),
  .in(mysaw),
  .out_lowpass(RDATA_c),
  .F(decay_out),  /* F1: frequency control; fixed point 1.17  ; F = 2sin(π*Fc/Fs).  At a sample rate of 250kHz, F ranges from 0.00050 (10Hz) -> ~0.55 (22kHz) */
  .Q1(18'h4000)  /* Q1: Q control;         fixed point 2.16  ; Q1 = 1/Q        Q1 ranges from 2 (Q=0.5) to 0 (Q = infinity). */
);




  // 4x4 LED stuff
  wire [3:0]  kled_tri;			// connect katode via SB_IO modules to allow high impadance  or 3.3V
  reg [15:0]  data16	;			// data register for 16 leds
  SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b 0) ) led_io1 ( .PACKAGE_PIN(kled[0]), .OUTPUT_ENABLE(kled_tri[0]), .D_OUT_0(1'b1)  );
  SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b 0) ) led_io2 ( .PACKAGE_PIN(kled[1]), .OUTPUT_ENABLE(kled_tri[1]), .D_OUT_0(1'b1)  );
  SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b 0) ) led_io3 ( .PACKAGE_PIN(kled[2]), .OUTPUT_ENABLE(kled_tri[2]), .D_OUT_0(1'b1)  );
  SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b 0) ) led_io4 ( .PACKAGE_PIN(kled[3]), .OUTPUT_ENABLE(kled_tri[3]), .D_OUT_0(1'b1)  );
  LED16  myleds (.clk(clk48),	.ledbits(data16)	,  .aled(aled), .kled_tri(kled_tri) );
  always @ ( posedge cs ) begin
    data16 <= data_from_sam;
  end

  // the Synth
  wire [15:0] mysaw;
  wire [15:0] noise;
  NOISE mnoise(	.clk(clk48),
					      .audio_out(noise) );

  SUPERSAW ssaw(.clk(I2S_LR),
  					    .pitch(data16),
  					    .audio_out(mysaw) );

  wire  [15:0] decay_out;
  DECAY decay (	.clk(clk48),
      					.trigger(~cs),
      					.decay_time(16'h6000),
      					.decayout(decay_out) );

  wire  [15:0] vca_out;
  VCA vca1( 	.clk(clk48),
      				.vca_in_a(decay_out),
      				.vca_in_b(mysaw),
      				.vca_out(vca_out) );

  PCM5102 dac(.clk(clk48),
      .left(vca_out),
      .right(RDATA_c),
      .din(I2S_DATA),
      .bck(I2S_BCLK),
      .lrck(I2S_LR) );





endmodule



//------------------------------------------------------------------------------
//          PCM5102 2 Channel DAC
//------------------------------------------------------------------------------
// http://www.ti.com/product/PCM5101A-Q1/datasheet/specifications#slase121473
module PCM5102(clk,left,right,din,bck,lrck);
	input 			clk;			// sysclk 48MHz
	input [15:0]	left,right;		// left and right 16bit samples Uint16
	output 	reg		din;			// pin on pcm5102 data
	output 	reg		bck;			// pin on pcm5102 bit clock
	output 	reg		lrck;			// pin on pcm5102 l/r clock can be used outside of this module to create new samples

	parameter DAC_CLK_DIV_BITS = 2;	// 1 = ca 384Khz, 2 = 192Khz, 3 = 96Khz, 4 = 48Khz

	reg [DAC_CLK_DIV_BITS:0]	i2s_clk;			// 2 Bit Counter 48MHz -> 6,0 MHz bck = ca 187,5 Khz SampleRate 4% tolerance ok by datasheet
	always @(posedge clk) begin
		i2s_clk 	<= i2s_clk + 1;
	end

	reg [15:0] l2c;
	reg [15:0] r2c;

	always @(negedge i2sword[5]) begin
		l2c <= left;
		r2c <= right;
	end

	reg [5:0]   i2sword = 0;		// 6 bit = 16 steps for left + right
	always @(negedge i2s_clk[DAC_CLK_DIV_BITS]) begin
		lrck	 	<= i2sword[5];
		din 		<= lrck ? r2c[16 - i2sword[4:1]] : l2c[16 - i2sword[4:1]];	// blit data bits
		bck			<= i2sword[0];
		i2sword		<= i2sword + 1;
	end
endmodule

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


//------------------------------------------------------------------------------
//          SUPERSAW Oscillator
//------------------------------------------------------------------------------
module SPI_SLAVE(clk, sck,miso,mosi,cs,data_from_sam,data_to_sam,bit_counter);
  input sck,mosi,cs,clk;
  output miso;
  input   [15:0] data_to_sam;
  output reg [15:0] data_from_sam;
  output reg [14:0] bit_counter;



  // SPI incoming CLK
  reg spi_clk1,spi_clk2;
  wire spi_clk_negedge = ( ~spi_clk1 &&  spi_clk2)  ;
  wire spi_clk_posedge = (  spi_clk1 && ~spi_clk2)  ;
  always @(posedge clk) begin
    spi_clk1 <= sck;
    spi_clk2 <= spi_clk1;
  end

  // SPI incoming CS
  reg spi_cs1,spi_cs2;
	wire spi_cs_negedge = ( ~spi_cs1 &&  spi_cs2)  ;
	wire spi_cs_posedge = (  spi_cs1 && ~spi_cs2)  ;
	always @(posedge clk) begin
		spi_cs1 <= cs;
		spi_cs2 <= spi_cs1;
	end

  // SPI incoming MOSI
  reg spi_mosi1,spi_mosi2;
	wire spi_mosi_negedge = ( ~spi_mosi1 &&  spi_mosi2)  ;
	wire spi_mosi_posedge = (  spi_mosi1 && ~spi_mosi2)  ;
	always @(posedge clk) begin
		spi_mosi1 <= mosi;
		spi_mosi2 <= spi_mosi1;
	end
  reg synced_mosi;
	always @(posedge clk) begin
		if(spi_mosi_posedge)				synced_mosi<= 1'b1;
		else if(spi_mosi_negedge)		synced_mosi<= 1'b0;
	end

  // Spi Shifter
	reg [15:0]		spi_in;
	reg [15:0]		miso_shift;
	assign miso = miso_shift[15];
	always @(posedge clk) begin
		if(spi_cs_posedge) begin

      bit_counter <= 0;
		end else if(spi_cs_negedge) begin
			miso_shift 	<= data_to_sam;
      bit_counter <= 0;
		end else begin
      bit_counter <= bit_counter + 1;
			if(spi_clk_posedge)		spi_in[15:0] 			<= {spi_in[14:0] , 			synced_mosi};
			if(spi_clk_posedge)		miso_shift[15:0] 	<= {miso_shift[14:0] , 	1'b1};
		end
	end

  always @(posedge bit_counter[4]) begin
    data_from_sam	<= spi_in;
  end

endmodule

// LED 4x4 Matrix
// we need to use tri state outputs to avoid bad polarity for LED´s
// just set Pins to static 1 and control by output_enable wire
module LED16 (input wire clk, input  [15:0] ledbits , output reg  [3:0] aled ,  output reg  [3:0] kled_tri );

	reg [31:0] counter;  // = 32'h00000000;
	always @(posedge clk)	begin
			counter<=counter+1 ;
	end

	// Show 16bit values
	always @(posedge counter[4])	begin // do the logic
		case ( counter[8:5] )
			4'b0000:		begin   kled_tri[3:0]  <= ledbits[0]   ? 4'b0001 :  4'd0 ;	 	   aled[3:0] <=	 4'b1110; 	end
			4'b0001:   	begin   kled_tri[3:0]  <= ledbits[1]   ? 4'b0001 :  4'd0;	 	   aled[3:0] <=	 4'b1101; 	end
			4'b0010:		begin   kled_tri[3:0]  <= ledbits[2]   ? 4'b0001 :  4'd0;	 	   aled[3:0] <=	 4'b1011; 	end
			4'b0011:   	begin   kled_tri[3:0]  <= ledbits[3]   ? 4'b0001 :  4'd0;	 	   aled[3:0] <=	 4'b0111; 	end
			4'b0100:		begin   kled_tri[3:0]  <= ledbits[4]   ? 4'b0010 :  4'd0;	 	   aled[3:0] <=	 4'b1110; 	end
			4'b0101:   	begin   kled_tri[3:0]  <= ledbits[5]   ? 4'b0010 :  4'd0;	 	   aled[3:0] <=	 4'b1101; 	end
			4'b0110:		begin   kled_tri[3:0]  <= ledbits[6]   ? 4'b0010 :  4'd0;	 	   aled[3:0] <=	 4'b1011; 	end
			4'b0111:   	begin   kled_tri[3:0]  <= ledbits[7]   ? 4'b0010 :  4'd0;	 	   aled[3:0] <=	 4'b0111; 	end
			4'b1000:		begin   kled_tri[3:0]  <= ledbits[8]   ? 4'b0100 :  4'd0;	 	   aled[3:0] <=	 4'b1110; 	end
			4'b1001:   	begin   kled_tri[3:0]  <= ledbits[9]   ? 4'b0100 :  4'd0;	 	   aled[3:0] <=	 4'b1101; 	end
			4'b1010:		begin   kled_tri[3:0]  <= ledbits[10] ? 4'b0100 :  4'd0;	 	   aled[3:0] <=	 4'b1011; 	end
			4'b1011:   	begin   kled_tri[3:0]  <= ledbits[11] ? 4'b0100 :  4'd0;	 	   aled[3:0] <=	 4'b0111; 	end
			4'b1100:		begin   kled_tri[3:0]  <= ledbits[12] ? 4'b1000 :  4'd0;	 	   aled[3:0] <=	 4'b1110; 	end
			4'b1101:   	begin   kled_tri[3:0]  <= ledbits[13] ? 4'b1000 :  4'd0;	 	   aled[3:0] <=	 4'b1101; 	end
			4'b1110:		begin   kled_tri[3:0]  <= ledbits[14] ? 4'b1000 :  4'd0;	 	   aled[3:0] <=	 4'b1011; 	end
			4'b1111:   	begin   kled_tri[3:0]  <= ledbits[15] ? 4'b1000 :  4'd0;	 	   aled[3:0] <=	 4'b0111;	end
		endcase
	end
endmodule


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

// from
// https://github.com/gundy/tiny-synth/blob/develop/hdl/filter_svf.vh
module filter_svf #(
  parameter SAMPLE_BITS = 12
)(
  input  clk,
  input  signed [SAMPLE_BITS-1:0] in,
  output signed [SAMPLE_BITS-1:0] out_highpass,
  output signed [SAMPLE_BITS-1:0] out_lowpass,
  output signed [SAMPLE_BITS-1:0] out_bandpass,
  output signed [SAMPLE_BITS-1:0] out_notch,
  input  signed [17:0] F,  /* F1: frequency control; fixed point 1.17  ; F = 2sin(π*Fc/Fs).  At a sample rate of 250kHz, F ranges from 0.00050 (10Hz) -> ~0.55 (22kHz) */
  input  signed [17:0] Q1  /* Q1: Q control;         fixed point 2.16  ; Q1 = 1/Q        Q1 ranges from 2 (Q=0.5) to 0 (Q = infinity). */
);


  reg signed[SAMPLE_BITS+2:0] highpass;
  reg signed[SAMPLE_BITS+2:0] lowpass;
  reg signed[SAMPLE_BITS+2:0] bandpass;
  reg signed[SAMPLE_BITS+2:0] notch;
  reg signed[SAMPLE_BITS+2:0] in_sign_extended;

  localparam signed [SAMPLE_BITS+2:0] MAX = (2**(SAMPLE_BITS-1))-1;
  localparam signed [SAMPLE_BITS+2:0] MIN = -(2**(SAMPLE_BITS-1));

  `define CLAMP(x) ((x>MAX)?MAX:((x<MIN)?MIN:x[SAMPLE_BITS-1:0]))

  assign out_highpass = `CLAMP(highpass);
  assign out_lowpass = `CLAMP(lowpass);
  assign out_bandpass = `CLAMP(bandpass);
  assign out_notch = `CLAMP(notch);

  // intermediate values from multipliers
  reg signed [34:0] Q1_scaled_delayed_bandpass;
  reg signed [34:0] F_scaled_delayed_bandpass;
  reg signed [34:0] F_scaled_highpass;

  initial begin
    highpass  = 0;
    lowpass   = 0;
    bandpass  = 0;
    notch     = 0;
  end

  // it can make prduce audioglitches
  // so TODO: check port this into SB_MAC16
  always @(posedge clk) begin
    in_sign_extended            = { in[SAMPLE_BITS-1], in[SAMPLE_BITS-1], in[SAMPLE_BITS-1], in};  /* sign-extend the input value to a wider precision */
    Q1_scaled_delayed_bandpass  = (bandpass * Q1) >>> 16;
    F_scaled_delayed_bandpass   = (bandpass * F) >>> 17;
    lowpass                     = lowpass + F_scaled_delayed_bandpass[SAMPLE_BITS+2:0];
    highpass                    = in_sign_extended - lowpass - Q1_scaled_delayed_bandpass[SAMPLE_BITS+2:0];
    F_scaled_highpass           = (highpass * F) >>> 17;
    bandpass                    = F_scaled_highpass[SAMPLE_BITS+2:0] + bandpass;
    notch                       = highpass + lowpass;
  end

endmodule
