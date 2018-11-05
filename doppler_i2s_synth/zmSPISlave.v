

//------------------------------------------------------------------------------
//          SPI_SLAVE
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
