`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/01/08 14:11:37
// Design Name: 
// Module Name: mpu6500_top
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


module mpu6500_top(
 DDR_addr,
 DDR_ba,
 DDR_cas_n,
 DDR_ck_n,
 DDR_ck_p,
 DDR_cke,
 DDR_cs_n,
 DDR_dm,
 DDR_dq,
 DDR_dqs_n,
 DDR_dqs_p,
 DDR_odt,
 DDR_ras_n,
 DDR_reset_n,
 DDR_we_n,
 FIXED_IO_ddr_vrn,
 FIXED_IO_ddr_vrp,
 FIXED_IO_mio,
 FIXED_IO_ps_clk,
 FIXED_IO_ps_porb,
 FIXED_IO_ps_srstb,
 gpio_0_tri_o,
 iic_0_scl_io,
 iic_0_sda_io,
 iic_ad0);
    
 inout [14:0]DDR_addr;
 inout [2:0]DDR_ba;
 inout DDR_cas_n;
 inout DDR_ck_n;
 inout DDR_ck_p;
 inout DDR_cke;
 inout DDR_cs_n;
 inout [3:0]DDR_dm;
 inout [31:0]DDR_dq;
 inout [3:0]DDR_dqs_n;
 inout [3:0]DDR_dqs_p;
 inout DDR_odt;
 inout DDR_ras_n;
 inout DDR_reset_n;
 inout DDR_we_n;
 inout FIXED_IO_ddr_vrn;
 inout FIXED_IO_ddr_vrp;
 inout [53:0]FIXED_IO_mio;
 inout FIXED_IO_ps_clk;
 inout FIXED_IO_ps_porb;
 inout FIXED_IO_ps_srstb;
 input [0:0]gpio_0_tri_o;
 inout iic_0_scl_io;
 inout iic_0_sda_io;    
 output iic_ad0;
 
 assign iic_ad0 = 0;
 
 ila_0 i0 (
     .clk(DDR_ck_n), // input wire clk
     .probe0(gpio_0_tri_o) // input wire [0:0] probe0
 );
    
 design_1_wrapper design_1_i
         (.DDR_addr(DDR_addr),
          .DDR_ba(DDR_ba),
          .DDR_cas_n(DDR_cas_n),
          .DDR_ck_n(DDR_ck_n),
          .DDR_ck_p(DDR_ck_p),
          .DDR_cke(DDR_cke),
          .DDR_cs_n(DDR_cs_n),
          .DDR_dm(DDR_dm),
          .DDR_dq(DDR_dq),
          .DDR_dqs_n(DDR_dqs_n),
          .DDR_dqs_p(DDR_dqs_p),
          .DDR_odt(DDR_odt),
          .DDR_ras_n(DDR_ras_n),
          .DDR_reset_n(DDR_reset_n),
          .DDR_we_n(DDR_we_n),
          .FIXED_IO_ddr_vrn(FIXED_IO_ddr_vrn),
          .FIXED_IO_ddr_vrp(FIXED_IO_ddr_vrp),
          .FIXED_IO_mio(FIXED_IO_mio),
          .FIXED_IO_ps_clk(FIXED_IO_ps_clk),
          .FIXED_IO_ps_porb(FIXED_IO_ps_porb),
          .FIXED_IO_ps_srstb(FIXED_IO_ps_srstb),
          .iic_0_scl_io(iic_0_scl_io),
          .iic_0_sda_io(iic_0_sda_io),
          .gpio_0_tri_o(gpio_0_tri_o));       
          
              
endmodule
