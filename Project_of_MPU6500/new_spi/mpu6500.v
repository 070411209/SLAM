`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/01/05 10:58:17
// Design Name: 
// Module Name: mpu6500
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


module mpu6500(
                input   clk_in,       
                // spi config
                output   spi_csn,
                output   spi_clk,
                output   spi_mosi,
                input    spi_miso,
                output   fsync,
                input    spi_int
    );


wire    manual_en;
wire    [15:0]      manual_data;

sysClock c0 (
    .clk_out1   (sys_clk),     // output clk_out1
    .locked     (rstn),       // output locked
    .clk_in1    (clk_in)             // input clk_in1
);

// vio interface
vio_config v0 (
  .clk       (sys_clk),                // input wire clk
  .probe_out0(manual_en),  // output wire [0 : 0] probe_out0
  .probe_out1(manual_data)  // output wire [15 : 0] probe_out1
);    
// mpu top    
mpu6500_config  u0(
                .sys_clk,
                .rstn,
                .manual_en,
                .manual_data,
                .spi_csn,
                .spi_clk,
                .spi_mosi,
                .spi_miso
);    
    
// ila 

//ila_spi i0 (
//	.clk   (sys_clk), // input wire clk
//	.probe0(manual_en), // input wire [0:0]  probe0  
//	.probe1(manual_data) // input wire [15:0]  probe1
//);    
    
endmodule
