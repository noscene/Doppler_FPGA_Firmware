
// LED 4x4 Matrix
// we need to use tri state outputs to avoid bad polarity for LED´s
// just set Pins to static 1 and control by output_enable wire
module LED4x4 (input wire clk, input  [15:0] ledbits , output reg  [3:0] aled ,  output reg  [3:0] kled_tri );

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
