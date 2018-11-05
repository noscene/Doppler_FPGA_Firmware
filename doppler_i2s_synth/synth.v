`default_nettype none
`include "zmDecay.v"
`include "zmSuperSaw.v"
`include "zmVCA.v"
`include "zmNoise.v"
`include "zmLED4x4.v"
`include "zmSPISlave.v"
`include "zmPCM5102.v"

//
//		Sample PCM5102 Module
//    output noise and supersaw with pitch control via SPI
/*
// Arduino Sound Demo
#include <ICEClass.h>
#include "/Users/svenbraun/Documents/GitHub/Doppler_FPGA_Firmware/doppler_i2s_synth/synth.h"
ICEClass ice40;
void setup() {
  ice40.upload(synth_bin,sizeof(synth_bin)); // Upload BitStream Firmware to FPGA -> see variant.h
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
            input button1, input button2,
            output  [3:0] kled  , output [3:0]  aled);

  wire clk48;
	SB_HFOSC inthosc ( .CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk48) );


  // Button handler
  SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_20 	( .PACKAGE_PIN(button1), 		.OUTPUT_ENABLE(1'b0), 				 .D_IN_0(bt1) );
  SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_21 	( .PACKAGE_PIN(button2), 		.OUTPUT_ENABLE(1'b0), 				 .D_IN_0(bt2) );
  reg [6:0] counter;
  reg signed [15:0] modValue=16'd6000;
  wire bt1,bt2;
  always @ ( posedge  I2S_LR ) begin
    counter <= counter+1;
  end
  reg button_reset;
  always @ ( posedge  counter[2] ) begin
    if(bt1 == 0)  begin
        modValue <= modValue + 1;
        button_reset <= 1;
    end else if(bt2 == 0) begin
        modValue <= modValue - 1;
        button_reset <= 1;
    end
    else begin
      button_reset <= 0;
    end
    ledValue4x4 <= modValue;
  end




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







  // 4x4 LED stuff
  wire [3:0]  kled_tri;			// connect katode via SB_IO modules to allow high impadance  or 3.3V
  reg [15:0]  ledValue4x4	;			// data register for 16 leds
  SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b 0) ) led_io1 ( .PACKAGE_PIN(kled[0]), .OUTPUT_ENABLE(kled_tri[0]), .D_OUT_0(1'b1)  );
  SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b 0) ) led_io2 ( .PACKAGE_PIN(kled[1]), .OUTPUT_ENABLE(kled_tri[1]), .D_OUT_0(1'b1)  );
  SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b 0) ) led_io3 ( .PACKAGE_PIN(kled[2]), .OUTPUT_ENABLE(kled_tri[2]), .D_OUT_0(1'b1)  );
  SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b 0) ) led_io4 ( .PACKAGE_PIN(kled[3]), .OUTPUT_ENABLE(kled_tri[3]), .D_OUT_0(1'b1)  );
  LED4x4  myleds (.clk(clk48),	.ledbits(ledValue4x4)	,  .aled(aled), .kled_tri(kled_tri) );



  wire [15:0] RDATA_c;
  filter_svf_pipelined  #(
    .SAMPLE_BITS(14)
  ) vcf (
    .clk(clk48),
    .sample_clk(~I2S_LR),
    .in(mysaw - 2000),
    .rst(button_reset),
    .out_lowpass(RDATA_c),
    .F(decay_out),  /* F1: frequency control; fixed point 1.15  ; F = 2sin(π*Fc/Fs).  At a sample rate of 250kHz, F ranges from 0.00050 (10Hz) -> ~0.55 (22kHz) */
    .Q1(modValue)  /* Q1: Q control;         fixed point 2.14  ; Q1 = 1/Q        Q1 ranges from 2 (Q=0.5) to 0 (Q = infinity). */
  );



  reg [15:0]  pitch	;			// data register for 16 leds
  always @ ( posedge cs ) begin
    pitch <= data_from_sam;
  end


  // the Synth
  wire [15:0] noise;
  NOISE mnoise(	.clk(clk48),
					      .audio_out(noise) );

  wire [15:0] mysaw;
  SUPERSAW ssaw(.clk(I2S_LR),
  					    .pitch(pitch),
  					    .audio_out(mysaw) );

  wire  [15:0] decay_out;
  DECAY decay (	.clk(clk48),
      					.trigger(~cs),
      					.decay_time(16'h3000),
      					.decayout(decay_out) );

  wire  [15:0] vca_out;
  VCA vca1( 	  .clk(clk48),
        				.vca_in_a(decay_out),
        				.vca_in_b(mysaw),
        				.vca_out(vca_out) );

  PCM5102 dac(  .clk(clk48),
                .left(vca_out),
                .right(RDATA_c),
                .din(I2S_DATA),
                .bck(I2S_BCLK),
                .lrck(I2S_LR) );





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



/*
 * State variable filter:
 *  Ref: Musical applications of microprocessors - Chamberlain: pp489+
 *
 * NOTE: this filter should be functionally equivalent to filter_svh, except
 *       that it is pipelined, and requires a higher frequency clock as well
 *       as the sample clock.
 *
 *       This implementation uses only a single 18*18 multiplier, rather than
 *       3, so should save a lot in terms of gate count.
 *
 *       The downside is that the filter takes 4 clock cycles for each sample.
 *
 * This filter provides high-pass, low-pass, band-pass and notch-pass outputs.
 *
 * Tuning parameters are F (frequency), and Q1 of the filter.
 *
 * The relation between F, the cut-off frequency Fc,
 * and the sampling rate, Fs, is approximated by the formula:
 *
 * F = 2π*Fc/Fs
 *
 * F is a 1.17 fixed-point value, and at a sample rate of 250kHz,
 * F ranges from approximately 0.00050 (10Hz) -> ~0.55 (22kHz).
 *
 * Q1 controls the Q (resonance) of the filter.  Q1 is equivalent to 1/Q.
 * Q1 ranges from 2 (corresponding to a Q value of 0.5) down to 0 (Q = infinity)
 */

 /* multiplier module */
 module smul_18x18 (
   input signed [17:0] a,
   input signed [17:0] b,
   output reg signed [35:0] o
 );
   always @(*) begin
    o = a * b;
   end
 endmodule

 module smul_16x16 (
   input clk,
   input signed [15:0] a,
   input signed [15:0] b,
   output reg signed [31:0] o
 );
 SB_MAC16 decay_mul (
     .A(a),
     .B(b),
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
     .O(o)
   );

 //16x16 => 32 unsigned pipelined multiply
 defparam decay_mul.B_SIGNED                  = 1'b1;
 defparam decay_mul.A_SIGNED                  = 1'b1;
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


/* from http://www.earlevel.com/main/2003/03/02/the-digital-state-variable-filter/
q= 1/Q ... normally ranges from 0.5 to inifinity
f= 2 * sin( pi * f / samplerate)
digital state variable filter
Hal Chamberlin’s Musical Applications of Microprocessors.
low = 0;
high = 0;
band = 0;
y = zeros(length(x),1);

for i = 1:length(x)
  low = low + f * band;
  high = x(i) - low - q * band;
  band = f * high + band;
  y(i) = high;
end

// http://billauer.co.il/blog/2012/10/signed-arithmetics-verilog/

*/



module filter_svf_pipelined #(
  parameter SAMPLE_BITS = 12
)(
  input  clk,rst,
  input  sample_clk,
  input  signed [SAMPLE_BITS-1:0] in,
  output reg signed [SAMPLE_BITS-1:0] out_highpass,
  output reg signed [SAMPLE_BITS-1:0] out_lowpass,
  output reg signed [SAMPLE_BITS-1:0] out_bandpass,
  output reg signed [SAMPLE_BITS-1:0] out_notch,
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




    // intermediate values from multipliers
    reg signed [35:0] Q1_scaled_delayed_bandpass;
    reg signed [35:0] F_scaled_delayed_bandpass;
    reg signed [35:0] F_scaled_highpass;




    reg signed [17:0] mul_a, mul_b;
    wire signed [35:0] mul_out;

  smul_18x18 multiplier(.a(mul_a), .b(mul_b), .o(mul_out));
  //smul_16x16 multiplier(.clk(clk), .a(mul_a), .b(mul_b), .o(mul_out));

  reg prev_sample_clk;
  reg [2:0] state;

  initial begin
    state = 3'd3;
    in_sign_extended = 0;
    prev_sample_clk = 0;
    highpass = 0;
    lowpass = 0;
    bandpass = 0;
    notch = 0;
  end

  always @(posedge clk) begin
    prev_sample_clk <= sample_clk;
    if (!prev_sample_clk && sample_clk) begin
      // sample clock has gone high, send out previously computed sample
      out_highpass <= `CLAMP(highpass);
      out_lowpass <= `CLAMP(lowpass);
      out_bandpass <= `CLAMP(bandpass);
      out_notch <= `CLAMP(notch);

      // also clock in new sample, and kick off state machine
       in_sign_extended <= { in[SAMPLE_BITS-1], in[SAMPLE_BITS-1], in[SAMPLE_BITS-1], in};
      mul_a <= bandpass;
      mul_b <= Q1;
      state <= 3'd0;

      if(rst) begin
        highpass <= 0;
        lowpass <= 0;
        bandpass <= 0;
        notch <= 0;
      end
    end

    case (state)
         3'd0: begin
                 // Q1_scaled_delayed_bandpass = (bandpass * Q1) >>> 16;
                 Q1_scaled_delayed_bandpass <= (mul_out >> 16);
                 mul_b <= F;
                 state <= 3'd1;
               end
         3'd1: begin
                 // F_scaled_delayed_bandpass = (bandpass * F) >>> 17;
                 F_scaled_delayed_bandpass = (mul_out >> 17);
                 lowpass = lowpass + F_scaled_delayed_bandpass[SAMPLE_BITS+2:0];
                 highpass = in_sign_extended - lowpass - Q1_scaled_delayed_bandpass[SAMPLE_BITS+2:0];
                 mul_a <= highpass;
                 state <= 3'd2;
               end
         3'd2: begin
                 F_scaled_highpass = mul_out >> 17;
                 bandpass <= F_scaled_highpass[SAMPLE_BITS+2:0] + bandpass;
                 notch <= highpass + lowpass;
                 state <= 3'd3;
               end
       endcase

    // in_sign_extended = { in[SAMPLE_BITS-1], in[SAMPLE_BITS-1], in[SAMPLE_BITS-1], in};  /* sign-extend the input value to a wider precision */
    // Q1_scaled_delayed_bandpass = (bandpass * Q1) >>> 16;
    // F_scaled_delayed_bandpass = (bandpass * F) >>> 17;
    // lowpass = lowpass + F_scaled_delayed_bandpass[SAMPLE_BITS+2:0];
    // highpass = in_sign_extended - lowpass - Q1_scaled_delayed_bandpass[SAMPLE_BITS+2:0];
    // F_scaled_highpass = (highpass * F) >>> 17;
    // bandpass = F_scaled_highpass[SAMPLE_BITS+2:0] + bandpass;
    // notch = highpass + lowpass;
  end

endmodule
