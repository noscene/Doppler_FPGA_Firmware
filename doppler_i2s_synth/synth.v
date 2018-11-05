`default_nettype none
`include "zmDecay.v"
`include "zmSuperSaw.v"
`include "zmVCA.v"
`include "zmNoise.v"
`include "zmFilter.v"
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
  reg [15:0] modValue=16'h0600;
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

  // good Q1 Values: 3fff .. 00ff
  // good F Values:  7fff .. 00ff

  filter_svf16  vcf (
    .clk(I2S_LR),
    .in(mysaw  ),
    // .rst(button_reset),
    .out_lowpass(RDATA_c),
    //.out_highpass(RDATA_c),
    //.out_bandpass(RDATA_c),
    .F(modValue),  // F1: frequency control; fixed point 1.13  ; F = 2sin(π*Fc/Fs).  At a sample rate of 250kHz, F ranges from 0.00050 (10Hz) -> ~0.55 (22kHz)
    .Q1(16'h05ff)   // Q1: Q control;         fixed point 2.12  ; Q1 = 1/Q        Q1 ranges from 2 (Q=0.5) to 0 (Q = infinity).
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

  reg [15:0] clk48div = 0;
  always @ ( posedge clk48 ) begin
    clk48div = clk48div + 1;
  end

  PCM5102 dac(  .clk(clk48),
                .left(vca_out),
                .right(RDATA_c),
                .din(I2S_DATA),
                .bck(I2S_BCLK),
                .lrck(I2S_LR) );
endmodule
