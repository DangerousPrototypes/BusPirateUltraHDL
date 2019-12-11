`include "registers.v"
`timescale 1ns/1ps
`define DUMPSTR(x) `"x.vcd`"

`define WRITE(address,data) mc_add <= address;mc_data_reg <= data;repeat(3)@(posedge clk);mc_we<=0;repeat(6)@(posedge clk);mc_we<=1;repeat(3)@(posedge clk)
`define READ(address,data) mc_add <= address;mc_data_reg <= data;repeat(3)@(posedge clk);mc_oe<=0;repeat(6)@(posedge clk);mc_oe<=1;repeat(3)@(posedge clk)
`define WC(command) mc_add <= 6'h01;mc_data_reg <= command;repeat(3)@(posedge clk);mc_we<=0;repeat(6)@(posedge clk);mc_we<=1;repeat(3)@(posedge clk)
`define WD(data) mc_add <= 6'h00;mc_data_reg <= data;repeat(3)@(posedge clk);mc_we<=0;repeat(6)@(posedge clk);mc_we<=1;repeat(3)@(posedge clk)
`define PULSE(signal) signal<=1'b1;repeat(5)@(posedge clk);signal<=1'b0;
module buspirate_tb();

  parameter DURATION = 10;
  parameter MC_DATA_WIDTH = 16;
  parameter MC_ADD_WIDTH = 6;
  parameter LA_WIDTH = 8;
  parameter LA_CHIPS = 2;
  parameter LA_SAMPLES = 24'hF42400;
  parameter BP_PINS = 8;
  parameter FIFO_WIDTH = 16;
  parameter FIFO_DEPTH = 256;

  reg clk, rst;

  wire [BP_PINS-1:0] bpio_buffer_io;
  wire [BP_PINS-1:0] bpio_buffer_dir, bpio_buffer_od,bpio_contention, bpio_state;
  reg [BP_PINS-1:0] bpio_test_input; //load test input values here
  wire [LA_CHIPS-1:0] sram_clock;
  wire [LA_CHIPS-1:0] sram_cs;
  wire [LA_WIDTH-1:0] sram_sio;
  reg [LA_WIDTH-1:0] sram_sio_d;
  wire lat_oe, lat_dir;
  reg [LA_WIDTH-1:0] lat;
  reg mcu_clock;
  reg mcu_cs;
  reg mcu_mosi; //sio0
  wire mcu_miso; //sio1
  reg mc_oe, mc_ce, mc_we;
  reg [MC_ADD_WIDTH-1:0] mc_add;
  wire [MC_DATA_WIDTH-1:0] mc_data;
  reg [MC_DATA_WIDTH-1:0] mc_data_reg;
  wire bp_active, bp_fifo_in_full,bp_fifo_out_nempty;
  reg bp_fifo_clear;

  wire adc_mux_en;
  wire [3:0] adc_mux_s;
  wire adc_cs, adc_clock;
  reg adc_data;
  wire pullup_enable;
  reg bpsm_halt_resume_d;


  assign mc_data=(mc_oe)?mc_data_reg:16'hzzzz;

  assign sram_sio=(!mc_oe)?sram_sio_d:8'hzz;


  top #(
    .MC_DATA_WIDTH(MC_DATA_WIDTH),
    .MC_ADD_WIDTH(MC_ADD_WIDTH),
    .LA_WIDTH(LA_WIDTH),
    .LA_CHIPS(LA_CHIPS),
    .LA_SAMPLES(LA_SAMPLES),
    .BP_PINS(BP_PINS),
    .FIFO_WIDTH(FIFO_WIDTH),
    .FIFO_DEPTH(FIFO_DEPTH)
    )buspirate(
    .clock(clk),
    .reset(rst),
    .bpio_buffer_io(bpio_buffer_io),
    .bpio_buffer_dir(bpio_buffer_dir),
    .bpio_buffer_od(bpio_buffer_od),
    .sram_clock(sram_clock),
    .sram_cs(sram_cs),
    .sram_sio(sram_sio),
    .lat_oe(lat_oe),
    .lat_dir(lat_dir),
    .lat(lat),
    .mcu_clock(mcu_clock),
    .mcu_mosi(mcu_mosi),
    .mcu_miso(mcu_miso),
    .mc_oe(mc_oe),
    .mc_ce(mc_ce),
    .mc_we(mc_we),
    .mc_add(mc_add),
    .mc_data(mc_data),
    .bp_active(bp_active),
    .bp_fifo_in_full(bp_fifo_in_full),
    .bp_fifo_out_nempty(bp_fifo_out_nempty),
    .bp_fifo_clear(bp_fifo_clear),
    .adc_mux_en(adc_mux_en),
    .adc_mux_s(adc_mux_s),
    .adc_cs(adc_cs),
    .adc_clock(adc_clock),
    .adc_data(adc_data),
    .pullup_enable(pullup_enable),
    .mcu_aux5(bpsm_halt_resume_d)

    );

    //this simulates the 74LVC logic buffers so we can see the results in simulation
    //the output from the iobuff "hardware driver" goes into here instead of physical hardware
    iobufphy BP_BUF[BP_PINS-1:0](
        .iopin_state(bpio_state),
        .iopin_contention(bpio_contention),
        .iopin_input(bpio_test_input),
      //hardware driver (reversed input/outputs from above)
        .bufdir(bpio_buffer_dir),
        .bufod(bpio_buffer_od),
        .bufio(bpio_buffer_io)
      );

    initial begin
      clk = 1'b0;
      rst = 1'b1;
      repeat(4) #10 clk = ~clk;
      rst = 1'b0;
      forever #10 clk = ~clk; // generate a clock
    end

    initial begin
      $dumpfile(`DUMPSTR(`VCD_OUTPUT));
      $dumpvars(0, buspirate_tb);
      bpio_test_input<=5'b11111;
      mc_we<=1;
      mc_oe<=1;
      mc_ce<=0;
      bp_fifo_clear<=0;
      adc_data<=1'b1;
      bpsm_halt_resume_d<=1'b0;
      mcu_cs<=1'b0;
      @(negedge rst); // wait for reset
      repeat(10) @(posedge clk);
      //test halt
      `WC(`CMD_SM_HALT);
      //logic analyzer in SPI mode
      `WRITE(6'h03,16'b001);
      `WRITE(6'h03,16'b1001);
      `WRITE(6'h03,16'b10001);
      //test SPI mode pins
      //`PULSE(mcu_cs);
      `PULSE(mcu_mosi);
      `PULSE(mcu_clock);
      //logic analyzer to QPI mode
      `WRITE(6'h03,16'b010);
      //test QPI input from SRAM (read SRAM)
      //`PULSE(sram_sio[0]);
      //test QPI output to SRAM (write SRAM)
      `WRITE(6'h03,16'b110);
      `WRITE(6'h02,16'h55AA);
      //logic analyzer reset
      `WC(`CMD_LA_RESET);
      //logic analyzer start
      `WC(`CMD_LA_START);
      //IO pins setup
      `WC(`CMD_REGISTER_SET_POINTER);//CMD_REGISTER_SET_POINTER
      `WD(16'h0000);
      `WC(`CMD_REGISTER_WRITE);//CMD_REGISTER_WRITE
      `WD(16'h00FF);//oe
      `WD(16'h0000);//od
      `WD(16'h0000);//hw config
      `WD(16'h0000); //`WD(16'b10001000);//pause BPSM (la config)
      `WD(16'h0000); //la sample counter
      `WD(16'h0000);
      `WD(16'h0000);
      `WD(16'h0000);
      `WD(16'h0000);
      `WD(16'h0000);
      `WD(16'h0003); //ADC calibrate|2:0 adc clock divider
      `WD(16'h0000);
      `WD(16'h0001); //REG_PERIPHERAL_0
      `WD(16'h0003); //REG_PERIPHERAL_1
      `WD(16'h0000); //REG_PERIPHERAL_2
      `WD(16'h0000); //REG_PERIPHERAL_3
      `WC(`CMD_REGISTER_SET_POINTER);//CMD_REGISTER_SET_POINTER
      `WD(16'h0000);
      `WC(`CMD_REGISTER_READ);
      `WD(16'h0005); //five words
      `WC(`CMD_REGISTER_SET_POINTER);//CMD_REGISTER_SET_POINTER
      `WD(16'h0002); //pullups configuration
      `WC(`CMD_REGISTER_WRITE);//CMD_REGISTER_WRITE
      `WD(16'h0001);//enable pullups
      bpio_test_input<=5'b11111;
      `WC(`CMD_DIO_TRIS); //CMD_DIO_TRIS
      `WD(16'h00FF); //all input
      `WC(`CMD_DIO_READ);
      `WD(16'h0000);
      bpio_test_input<=5'b00000;
      `WD(16'h0000);
      `WC(`CMD_DIO_TRIS); //CMD_DIO_TRIS
      `WD(16'h0000); //all output
      `WC(`CMD_DIO_WRITE); //CMD_DIO_WRITE
      `WD(16'h00FF);//IO pins high
      `WD(16'h0000); //IO pins low
      `WD(16'h00FF);//IO pins high
      `WC(`CMD_PERIPHERAL_WRITE);
      `WD(16'h08AA);
      `WC(`CMD_DELAY);
      `WD(16'h0010);
      `WC(`CMD_PWM_ON_PERIOD); //PWM ON
      `WD(16'h000F);
      `WC(`CMD_PWM_OFF_PERIOD); //PWM OFF
      `WD(16'h000F);
      `WC(`CMD_ADC_READ); //ADC measurement
      `WD(16'h0001); //MUX 0001
      //logic analyzer stop
      `WC(`CMD_LA_STOP);
      //release the state machine
      `PULSE(bpsm_halt_resume_d);

      //logic analyzer read


      /*`WRITE(6'h07,16'h08aa); //write SPI data
      `WRITE(6'h07,16'h08ff);
      //`WRITE(6'h07,16'hFD00);//halt command
      `WRITE(6'h07,16'h0800);
      `WRITE(6'h07,16'h840F); //delay 0x0f
      `WRITE(6'h07,16'h81FF); //IO pins high
      `WRITE(6'h07,16'hFF00); //stop logic analyzer
      `WRITE(6'h03,16'b00000000);//trigger BP SM
      */
      /*repeat(30)@(posedge clk);
      bp_fifo_clear<=1;
      repeat(4)@(posedge clk);
      bp_fifo_clear<=0;*/
      repeat(500)@(posedge clk);
      `WC(`CMD_LA_RESET);
      repeat(200)@(posedge clk);
      $finish;
    end

endmodule
