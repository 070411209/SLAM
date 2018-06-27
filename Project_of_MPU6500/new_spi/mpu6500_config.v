`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/01/05 11:47:06
// Design Name: 
// Module Name: mpu6500_config
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


module mpu6500_config(
input    sys_clk,
input    rstn,
input    manual_en,
input    [15:0]  manual_data,
output   spi_csn,
output   spi_clk,
output   spi_mosi,
input    spi_miso 
);
    
wire        mSPI_START;
wire        mSPI_END;
wire [15:0] mSPI_DATA;
wire [7:0]  DATA_OUT;

// ila
ila_spi i0 (
	.clk   (sys_clk), // input wire clk
	.probe0({manual_en,mSPI_START,spi_csn,mSPI_END}), // input wire [0:0]  probe0  
	.probe1(DATA_OUT) // input wire [15:0]  probe1
);  



spi_control u1(
          .clk          (sys_clk),
          .rstn         (rstn),
          .spi_csn      (spi_csn),
          .spi_clk      (spi_clk),
          .spi_mosi     (spi_mosi),
          .spi_miso     (spi_miso),
          .GO           (mSPI_START), // input
          .END          (mSPI_END), // output
          .mSPI_DATA    (mSPI_DATA),   // input
          .led_8bit     (DATA_OUT)
    );     


spi_config  u2(
        .clk            (sys_clk),
        .rstn           (rstn),
        .manual_en      (manual_en),
        .manual_data    (manual_data),
        .mSPI_START     (mSPI_START),
        .mSPI_DATA      (mSPI_DATA)
);    
        
endmodule
