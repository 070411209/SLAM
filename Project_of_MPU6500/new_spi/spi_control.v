`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2016/03/21 14:56:30
// Design Name: 
// Module Name: spi_control
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module spi_control(
                input    clk,
                input    rstn,
                output   spi_csn,
                output   spi_clk,
                output   spi_mosi,
                input    spi_miso,	
                input    GO,
                output   reg END,
                input    [15:0] mSPI_DATA,	
                output   [7:0]  led_8bit
);

	parameter SPI_FREQ = 12;
	parameter SPI_HALF_FREQ = 6;

	reg    	CLOCK;
	reg	    SPI_EN;
	reg    [9:0]	 CLK_DIV;
	reg    [7:0] 	 SPI_DATAIN;
    reg    [6:0]    SD_COUNTER;
    reg    [15:0]   SD;
    reg    SDO;
    reg    [7:0]   led_r0;

	assign	spi_clk = CLOCK;
	assign	spi_csn = SPI_EN;
	assign	spi_mosi = SDO;
	assign  led_8bit = led_r0;

    
always @(posedge clk) begin
	if(!rstn)
	   led_r0 <= 0;
	else if(END)
	   led_r0 <= SPI_DATAIN;
	else;
end

always @(posedge clk) begin
	if(!rstn)
		SPI_EN <= 1'b1;
	else if(GO)
		SPI_EN <= 1'b0;
	else if(END)
		SPI_EN <= 1'b1;
	else;
end
// 6500 SDO out at falling  ---	FPGA read at rising
always @(posedge clk) begin
	if(!rstn)
		SPI_DATAIN <= 0;
	else if( CLK_DIV == SPI_FREQ )                // 下降沿采集数据 change by lsq, 20160401
		SPI_DATAIN <= {SPI_DATAIN[6:0],spi_miso};
	else;
end
// SCLK	maximum frequency 1MHz
always @(posedge clk) begin
	if(!rstn) begin
		CLK_DIV <= 0; 
		CLOCK <= 1; 
	end
	else if(!SPI_EN) begin
        // count		
		if(CLK_DIV < SPI_FREQ)
		    CLK_DIV <= CLK_DIV + 1;
		else
			CLK_DIV <= 0;
		// 
		if(CLK_DIV < SPI_HALF_FREQ)
			CLOCK <= 1;
		else
		    CLOCK <= 0;		
	end	
	else begin
		CLOCK <= 1; CLK_DIV <= 0; 
	end
end
//--------------------------------------
//------------- SPI Write --------------

// 6500 SDI read at rising and FPGA write at falling
always @(posedge clk) begin
	if(!rstn)
		SD_COUNTER <=6'd0;
	else begin
		if (!SPI_EN) begin
			if(CLK_DIV == SPI_HALF_FREQ)
				SD_COUNTER <= SD_COUNTER + 1;	
			else;
		end
		else SD_COUNTER <= 0;
	end
end
	
always @(posedge clk) begin
	if(!rstn) begin
		END <= 0;
		SDO <= 0;
		SD  <= 0;
	end
	else begin
		case(SD_COUNTER)
		0:	begin SD <= mSPI_DATA; END <= 0; SDO <= 0; end
		// Address R/W 
		1:  SDO <= mSPI_DATA[15];		
		2:  SDO <= mSPI_DATA[14];
		3:  SDO <= mSPI_DATA[13];
		4:  SDO <= mSPI_DATA[12];		
		5:  SDO <= mSPI_DATA[11];		
		6:  SDO <= mSPI_DATA[10];
		7:  SDO <= mSPI_DATA[9];
		8:  SDO <= mSPI_DATA[8];		
		// DATA
		9:   SDO <= mSPI_DATA[7];		
		10:  SDO <= mSPI_DATA[6];
		11:  SDO <= mSPI_DATA[5];
		12:  SDO <= mSPI_DATA[4];		
		13:  SDO <= mSPI_DATA[3];		
		14:  SDO <= mSPI_DATA[2];
		15:  SDO <= mSPI_DATA[1];
		16:  SDO <= mSPI_DATA[0];
		17:  END <= 1;
		
		default: begin SDO <= 0; END <= 0; SD <= 0; end
		endcase
	end	
end	
    
    
endmodule
