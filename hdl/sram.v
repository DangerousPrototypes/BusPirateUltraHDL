
//Logic analyzer endmodule

//srams endmodule
module sram (
  input clock,
  input auto_clock,
  input la_active,
  input spi_mode,
  input quad_mode,
  input quad_direction,
  input quad_output,
  //pin controls
  output sram_cs,
  output sram_clock,
  input sram_sio_tdi,
  output sram_sio_tdo,
  output sram_sio_oe,
  //master spi signals
  input mcu_sclk,
  input mcu_mosi,
  output mcu_miso,
  input mcu_cs
  );

wire [3:0] sram_sio_tdi;
wire [3:0] sram_sio_tdo;
wire [3:0] sram_sio_oe;

reg [3:0] sram_out_d;

assign sram_clock = (la_active)? clock : (quad_mode)? auto_clock : (spi_mode)? mcu_clock : 1'b0; //idle low
assign sram_cs = (la_active)? 1'b0 : (quad_mode || spi_mode)? mcu_cs : 1'b1; //idle high
assign sram_sio_tdo[0] = (la_active)? lat[0] : (quad_mode)? sram_out_d[0] : mcu_mosi;
assign sram_sio_tdo[3:1] = (la_active)? lat[3:1] : sram_out_d[3:1];
assign miso = sram_sio_tdi[1];

assign sram_sio_oe[0] = (quad_mode)? quad_direction : 1'b1;
assign sram_sio_oe[1] = (quad_mode)? quad_direction : 1'b0;
assign sram_sio_oe[2] = quad_mode && quad_direction;
assign sram_sio_oe[3] = quad_mode && quad_direction;
