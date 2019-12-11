
`ifndef __SRAM__
`define __SRAM__

module sram (
  input clock,
  input auto_clock,
  input la_active,
  input spi_mode,
  input qpi_mode,
  input qpi_direction,
  input [3:0] qpi_input,
  //latch
  input [3:0] lat,
  //pin controls
  output sram_cs,
  output sram_clock,
  input [3:0] sram_sio_tdi,
  output [3:0] sram_sio_tdo,
  output [3:0] sram_sio_oe,
  //master spi signals
  input mcu_sclk,
  input mcu_mosi,
  output mcu_miso,
  input mcu_cs
  );

assign sram_clock = (la_active)? clock : (qpi_mode)? auto_clock : (spi_mode)? mcu_sclk : 1'b0; //idle low
assign sram_cs = (la_active)? 1'b0 : (qpi_mode || spi_mode)? mcu_cs : 1'b1; //idle high
assign sram_sio_tdo[0] = (la_active)? lat[0] : (qpi_mode)? qpi_input[0] : mcu_mosi;
assign sram_sio_tdo[3:1] = (la_active)? lat[3:1] : qpi_input[3:1];
assign miso = sram_sio_tdi[1];

assign sram_sio_oe[0] = (qpi_mode)? qpi_direction : 1'b1;
assign sram_sio_oe[1] = (qpi_mode)? qpi_direction : 1'b0;
assign sram_sio_oe[2] = qpi_mode && qpi_direction;
assign sram_sio_oe[3] = qpi_mode && qpi_direction;

endmodule
`endif
