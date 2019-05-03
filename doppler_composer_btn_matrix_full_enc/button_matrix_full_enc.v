`default_nettype none


//
//
//
module top ( output [12:0] row,
					   input  [17:0] col,
						 input cfg_cs,  	input  cfg_si,  input cfg_sck,				// SPI:     samd51 <-> ice40  for bitstream and user cases
						 output cfg_so,																			  	// SPI: miso
						input encoderPinA,input encoderPinB, input encoderPinBtn
					 );

	// use ice40up5k 48Mhz internal oscillator
	wire clk;
	SB_HFOSC inthosc ( .CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk) );


	// inputs pins from Matrix with PULLUP
	wire encoderA,encoderB;
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_ena 	( .PACKAGE_PIN(encoderPinA), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(encoderA) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_enb 	( .PACKAGE_PIN(encoderPinB), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(encoderB) );



	wire encoderBtn;
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_eb 	( .PACKAGE_PIN(encoderPinBtn), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(encoderBtn) );

	wire [17:0] col_in;
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_0 	( .PACKAGE_PIN(col[0]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[0]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_1 	( .PACKAGE_PIN(col[1]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[1]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_2 	( .PACKAGE_PIN(col[2]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[2]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_3 	( .PACKAGE_PIN(col[3]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[3]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_4 	( .PACKAGE_PIN(col[4]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[4]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_5 	( .PACKAGE_PIN(col[5]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[5]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_6 	( .PACKAGE_PIN(col[6]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[6]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_7 	( .PACKAGE_PIN(col[7]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[7]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_8 	( .PACKAGE_PIN(col[8]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[8]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_9 	( .PACKAGE_PIN(col[9]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[9]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_10 	( .PACKAGE_PIN(col[10]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[10]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_11 	( .PACKAGE_PIN(col[11]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[11]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_12 	( .PACKAGE_PIN(col[12]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[12]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_13 	( .PACKAGE_PIN(col[13]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[13]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_14 	( .PACKAGE_PIN(col[14]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[14]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_15 	( .PACKAGE_PIN(col[15]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[15]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_16 	( .PACKAGE_PIN(col[16]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[16]) );
	SB_IO #( .PIN_TYPE(6'b 1010_01), .PULLUP(1'b1) ) upin_17 	( .PACKAGE_PIN(col[17]), 	.OUTPUT_ENABLE(1'b0),	.D_OUT_0(1'b0) , .D_IN_0(col_in[17]) );


	wire [7:0] enc_data;
	reg encoderReset;
	ENCODER myEncoder ( .clk(clk), .quadA(encoderA), .quadB(encoderB) , .count(enc_data), .reset(encoderReset));	 		
 	

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
	//reg [15:0]		spi_in;
	//reg [15:0]		miso_shift;
	//assign cfg_so = miso_shift[15];
	//always @(posedge clk) begin
	//	if(spi_cs_posedge) begin
	//		row 			<= spi_in;							// Just Write data 2 LED
	//	end else if(spi_cs_negedge) begin
	//		miso_shift 	<= col_in[15:0];	// PinRead
	//	end else begin
	//		if(spi_clk_posedge)		spi_in[15:0] 			<= {spi_in[14:0] , 			mosi};
	//		if(spi_clk_posedge)		miso_shift[15:0] 	<= {miso_shift[14:0] , 	1'b1};
	//	end
	//end

	// Spi Shifter
	reg [31:0]		spi_in;
	reg [31:0]		miso_shift;
	assign cfg_so = miso_shift[31];
	always @(posedge clk) begin
		if(spi_cs_posedge) begin
			row 			<= spi_in;							// Just Write data 2 LED
			encoderReset <= 1'b0;
		end else if(spi_cs_negedge) begin
			miso_shift 	<= {enc_data[7:0] , encoderBtn,  5'b11111 , col_in[17:0] };	// alles wird zusammengebaut
			encoderReset <= 1'b1;
		end else begin
			if(spi_clk_posedge)		spi_in[31:0] 			<= {spi_in[30:0] , 			mosi};
			if(spi_clk_posedge)		miso_shift[31:0] 	<= {miso_shift[30:0] , 	1'b1};
			encoderReset <= 1'b0;
		end
	end
	

endmodule		// end top module



// Encoder Module from https://www.fpga4fun.com/QuadratureDecoder.html
module ENCODER(clk, quadA, quadB, count , reset);	

	input clk, quadA, quadB, reset;
	output [7:0] count;



	reg [2:0] quadA_delayed, quadB_delayed;


    reg [7:0]	encoder_clk_div;
	always @(posedge clk) begin
		// data16 <= enc_data;
		encoder_clk_div <= encoder_clk_div +1;
	end		



	always @(posedge encoder_clk_div[7]) quadA_delayed <= {quadA_delayed[1:0], quadA};
	always @(posedge encoder_clk_div[7]) quadB_delayed <= {quadB_delayed[1:0], quadB};

	wire count_enable = quadA_delayed[1] ^ quadA_delayed[2] ^ quadB_delayed[1] ^ quadB_delayed[2];
	wire count_direction = quadA_delayed[1] ^ quadB_delayed[2];





	reg [7:0] count  = 8'h80;
	always @(posedge clk)
	begin
	if(reset)	begin
		count  = 8'h80;
	end else begin
	  if(count_enable)
	  begin
	    if(count_direction) count<=count+1; else count<=count-1;
	  end
	end
	end
endmodule