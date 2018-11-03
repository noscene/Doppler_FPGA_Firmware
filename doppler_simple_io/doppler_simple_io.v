`default_nettype none


//
//		This is the default firmware for ice40up5k doppler board
//		
//
module top ( inout  [7:0] pinbank1,													// breakout io pins F11,  F12 , F13, F18, F19, F20, F21, F23
					inout  [7:0] pinbank2,													// breakout io pins F41,  F40 , F39, F38, F37, F36, F35, F34
					output  [3:0] kled  , output [3:0]  aled,  						// led matrix  see the .pcf file in projectfolder for physical pins
					input button1,  	input	button2, 									// 2 Buttons 
					input cfg_cs,  	input  cfg_si,  input cfg_sck,				// SPI:     samd51 <-> ice40  for bitstream and user cases
					output cfg_so,															// SPI Out
					inout pa19, inout pa21, inout pa22,							// alternat SPI Port
					inout pa20,
					inout F25, F32
					 );												
	
	// use ice40up5k 48Mhz internal oscillator
	wire clk; 
	//wire clk48; 
	SB_HFOSC inthosc ( .CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk) );
	// pll my100mhz(.clock_in(clk48), .clock_out(clk) /* , output locked */ );

	// configure/connect IO Pins for LED driver logic
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b 0) ) led_io1 ( .PACKAGE_PIN(kled[0]), .OUTPUT_ENABLE(kled_tri[0]), .D_OUT_0(1'b1)  );  // .D_IN_0(dummy2)
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b 0) ) led_io2 ( .PACKAGE_PIN(kled[1]), .OUTPUT_ENABLE(kled_tri[1]), .D_OUT_0(1'b1)  ); 
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b 0) ) led_io3 ( .PACKAGE_PIN(kled[2]), .OUTPUT_ENABLE(kled_tri[2]), .D_OUT_0(1'b1)  );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b 0) ) led_io4 ( .PACKAGE_PIN(kled[3]), .OUTPUT_ENABLE(kled_tri[3]), .D_OUT_0(1'b1)  );
	
	//	reg [23:0] pullup_cfg;		// TUT NICHT weil nur Parm!


	// Make connections from Pins in top tu Registers and support INPUT/ OUTPUT arrays
	reg [23:0] pin_state_out ;
	wire [23:0] pin_state_in;
	reg  [23:0] out_eneable_cfg;
	
	
	assign  pin_state_out = 24'h0;
	assign  out_eneable_cfg = 24'h0;
	
	// sorry but sometime i need to read src like a excelsheet ;-)
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_0 	( .PACKAGE_PIN(pa19), 				.OUTPUT_ENABLE(out_eneable_cfg[0]), 	.D_OUT_0(pin_state_out[0]) , 	.D_IN_0(pin_state_in[20]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_1 	( .PACKAGE_PIN(pa20), 				.OUTPUT_ENABLE(out_eneable_cfg[1]), 	.D_OUT_0(pin_state_out[1]) , 	.D_IN_0(pin_state_in[21]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_2 	( .PACKAGE_PIN(pa21), 				.OUTPUT_ENABLE(out_eneable_cfg[2]), 	.D_OUT_0(pin_state_out[2]) , 	.D_IN_0(pin_state_in[22]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_3 	( .PACKAGE_PIN(pa22), 				.OUTPUT_ENABLE(out_eneable_cfg[3]), 	.D_OUT_0(pin_state_out[3]) , 	.D_IN_0(pin_state_in[23]) );
 	
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b0) ) upin_4 	( .PACKAGE_PIN(pinbank1[0]), 	.OUTPUT_ENABLE(out_eneable_cfg[4]), 	.D_OUT_0(pin_state_out[4]) , 	.D_IN_0(pin_state_in[4]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b0) ) upin_5 	( .PACKAGE_PIN(pinbank1[1]), 	.OUTPUT_ENABLE(out_eneable_cfg[5]), 	.D_OUT_0(pin_state_out[5]) , 	.D_IN_0(pin_state_in[5]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_6 	( .PACKAGE_PIN(pinbank1[2]), 	.OUTPUT_ENABLE(out_eneable_cfg[6]), 	.D_OUT_0(pin_state_out[6]) , 	.D_IN_0(pin_state_in[6]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_7 	( .PACKAGE_PIN(pinbank1[3]), 	.OUTPUT_ENABLE(out_eneable_cfg[7]), 	.D_OUT_0(pin_state_out[7]) , 	.D_IN_0(pin_state_in[7]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_8 	( .PACKAGE_PIN(pinbank1[4]), 	.OUTPUT_ENABLE(out_eneable_cfg[8]), 	.D_OUT_0(pin_state_out[8]) , 	.D_IN_0(pin_state_in[8]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_9 	( .PACKAGE_PIN(pinbank1[5]), 	.OUTPUT_ENABLE(out_eneable_cfg[9]), 	.D_OUT_0(pin_state_out[9]) , 	.D_IN_0(pin_state_in[9]) ); 
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_10 	( .PACKAGE_PIN(pinbank1[6]), 	.OUTPUT_ENABLE(out_eneable_cfg[10]),	.D_OUT_0(pin_state_out[10]) , .D_IN_0(pin_state_in[10]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_11 	( .PACKAGE_PIN(pinbank1[7]), 	.OUTPUT_ENABLE(out_eneable_cfg[11]),	.D_OUT_0(pin_state_out[11]) ,	.D_IN_0(pin_state_in[11]) );

    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_12	( .PACKAGE_PIN(pinbank2[0]), 	.OUTPUT_ENABLE(out_eneable_cfg[12]),	.D_OUT_0(pin_state_out[12]) , .D_IN_0(pin_state_in[12]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_13	( .PACKAGE_PIN(pinbank2[1]), 	.OUTPUT_ENABLE(out_eneable_cfg[13]), 	.D_OUT_0(pin_state_out[13]) , .D_IN_0(pin_state_in[13]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_14	( .PACKAGE_PIN(pinbank2[2]), 	.OUTPUT_ENABLE(out_eneable_cfg[14]), 	.D_OUT_0(pin_state_out[14]) , .D_IN_0(pin_state_in[14]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_15 	( .PACKAGE_PIN(pinbank2[3]), 	.OUTPUT_ENABLE(out_eneable_cfg[15]), 	.D_OUT_0(pin_state_out[15]) , .D_IN_0(pin_state_in[15]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_16 	( .PACKAGE_PIN(pinbank2[4]), 	.OUTPUT_ENABLE(out_eneable_cfg[16]), 	.D_OUT_0(pin_state_out[16]) , .D_IN_0(pin_state_in[16]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_17 	( .PACKAGE_PIN(pinbank2[5]), 	.OUTPUT_ENABLE(out_eneable_cfg[17]), 	.D_OUT_0(pin_state_out[17]) , .D_IN_0(pin_state_in[17]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_18 	( .PACKAGE_PIN(pinbank2[6]), 	.OUTPUT_ENABLE(out_eneable_cfg[18]),	.D_OUT_0(pin_state_out[18]) , .D_IN_0(pin_state_in[18]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_19 	( .PACKAGE_PIN(pinbank2[7]), 	.OUTPUT_ENABLE(out_eneable_cfg[19]),	.D_OUT_0(pin_state_out[19]) , .D_IN_0(pin_state_in[19]) );
	
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_20 	( .PACKAGE_PIN(button1), 		.OUTPUT_ENABLE(1'b0), 							.D_OUT_0(pin_state_out[20]) , .D_IN_0(pin_state_in[0]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_21 	( .PACKAGE_PIN(button2), 		.OUTPUT_ENABLE(1'b0), 							.D_OUT_0(pin_state_out[21]) , .D_IN_0(pin_state_in[1]) ); 
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_22 	( .PACKAGE_PIN(F25), 				.OUTPUT_ENABLE(out_eneable_cfg[22]), 	.D_OUT_0(pin_state_out[22]) , .D_IN_0(pin_state_in[2]) );
    SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_23 	( .PACKAGE_PIN(F32), 				.OUTPUT_ENABLE(out_eneable_cfg[23]), 	.D_OUT_0(pin_state_out[23]) , .D_IN_0(pin_state_in[3]) );


	
	// see this https://youtu.be/IOmG5y7VMrg?t=49m32s (german only)
	// Cross Domain Clock Syncing! SPI_INCOMING_CLK
	reg spi_clk1,spi_clk2;
	wire spi_clk_negedge = ( ~spi_clk1 &&  spi_clk2)  ;
	wire spi_clk_posedge = (  spi_clk1 && ~spi_clk2)  ;
	always @(posedge clk) begin
		spi_clk1 <= cfg_sck;
		spi_clk2 <= spi_clk1;
	end

	// Cross Domain Clock Syncing! SPI_INCOMING_CS + register Set
	reg spi_cs1,spi_cs2;
	wire spi_cs_negedge = ( ~spi_cs1 &&  spi_cs2)  ;
	wire spi_cs_posedge = (  spi_cs1 && ~spi_cs2)  ;
	always @(posedge clk) begin
		spi_cs1 <= cfg_cs;
		spi_cs2 <= spi_cs1;
	end /*
	reg cs;
	always @(posedge clk) begin
		if(spi_cs_posedge)				cs<= 1'b1;
		else if(spi_cs_negedge)		cs<= 1'b0;
	end
	*/
	
	// Cross Domain Clock Syncing! SPI_INCOMING_MOSI + register Set
	reg spi_mosi1,spi_mosi2;
	wire spi_mosi_negedge = ( ~spi_mosi1 &&  spi_mosi2)  ;
	wire spi_mosi_posedge = (  spi_mosi1 && ~spi_mosi2)  ;
	always @(posedge clk) begin
		spi_mosi1 <= cfg_si;
		spi_mosi2 <= spi_mosi1;
	end
	reg mosi;
	always @(posedge clk) begin
		if(spi_mosi_posedge)				mosi<= 1'b1;
		else if(spi_mosi_negedge)		mosi<= 1'b0;
	end

	// Spi Shifter
	reg [15:0]		spi_in;
	reg [15:0]		miso_shift;
	assign cfg_so = miso_shift[15];
	always @(posedge clk) begin
		if(spi_cs_posedge) begin
			data16 			<= spi_in;							// Just Write data 2 LED
		end else if(spi_cs_negedge) begin
			// miso_shift 	<= data16; 					// loopbackTest
			miso_shift 	<= pin_state_in[15:0];	// PinRead
			//miso_shift 	<= 16'h53f0;					// constValues answer just for tesing
		end else begin
			if(spi_clk_posedge)		spi_in[15:0] 			<= {spi_in[14:0] , 			mosi};
			if(spi_clk_posedge)		miso_shift[15:0] 	<= {miso_shift[14:0] , 	1'b1};
		end
	end
	

	// Led
	wire [3:0]  kled_tri;			// connect katode via SB_IO modules to allow high impadance  or 3.3V
	reg [15:0] data16	;			// data register for 16 leds
 	LED16  myleds (.clk(clk),	.ledbits(data16)	,  .aled(aled), .kled_tri(kled_tri) );
 		
		
endmodule		// end top module


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

/**
 * PLL configuration
 *
 * This Verilog module was generated automatically
 * using the icepll tool from the IceStorm project.
 * Use at your own risk.
 *
 * icepll -i 48 -o 100 -m -f pll.v
 *
 * Given input frequency:        60.000 MHz
 * Requested output frequency:  100.000 MHz
 * Achieved output frequency:   100.000 MHz
 */

module pll(
	input  clock_in,
	output clock_out,
	output locked
	);


SB_PLL40_CORE #(
		.FEEDBACK_PATH("SIMPLE"),
		.DIVR(4'b0010),		// DIVR =  2
		.DIVF(7'b0111111),	// DIVF = 63
		.DIVQ(3'b110),		// DIVQ =  6
		.FILTER_RANGE(3'b001)	// FILTER_RANGE = 1
	) uut (
		.LOCK(locked),
		.RESETB(1'b1),
		.BYPASS(1'b0),
		.REFERENCECLK(clock_in),
		.PLLOUTCORE(clock_out)
		);

endmodule

