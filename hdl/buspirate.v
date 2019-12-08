//------------------------------------------------------------------
//-- Bus Pirate peripheral tests
//--
//------------------------------------------------------------------
`include "iobuf.v"
`include "iobufphy.v"
`include "synchronizer.v"
`include "pwm.v"
`include "pll.v"
`include "registers.v"
`include "spimaster.v"
`include "fifo.v"
`include "adc.v"
//`include "ram.v"
`define SIMULATION

module top #(
  parameter MC_DATA_WIDTH = 16,
  parameter MC_ADD_WIDTH = 6,
  parameter LA_WIDTH = 8,
  parameter LA_CHIPS = 2,
  parameter BP_PINS = 8,
  parameter FIFO_WIDTH = 16,
  parameter FIFO_DEPTH = 512
) (
  input clock,
  //input reset,
  inout wire [BP_PINS-1:0] bpio_buffer_io,
  output wire [BP_PINS-1:0] bpio_buffer_dir, bpio_buffer_od,
  output wire[LA_CHIPS-1:0] sram_clock,
  output wire[LA_CHIPS-1:0] sram_cs,
  inout[LA_WIDTH-1:0] sram_sio,
  output wire lat_oe, lat_dir,
  input wire [LA_WIDTH-1:0] lat,
  input wire mcu_clock,
  input wire mcu_cs,
  input wire mcu_mosi, //sio0
  output wire mcu_miso, //sio1
  input wire mc_oe, mc_ce, mc_we,
  input wire [MC_ADD_WIDTH-1:0] mc_add,
  inout [MC_DATA_WIDTH-1:0] mc_data,
  output bp_active,
  output bp_fifo_in_full,
  output bp_fifo_out_nempty,
  input bp_fifo_clear,
  output adc_mux_en,
  output [3:0] adc_mux_s,
  output adc_cs, adc_clock,
  input adc_data,
  output pullup_enable
  );
    //pll PLL_2F(clock,pll_clk,locked);

    // CONFIGURATION REGISTERS
    reg [MC_DATA_WIDTH-1:0] config_register [15:0];
    reg [3:0] reg_count, reg_index;

    //wires tied to the memory controller WE and OE signals
    wire mc_we_sync,mc_oe_sync,mc_ce_sync;
    sync MC_WE_SYNC(clock, mc_we, mc_we_sync);
    sync MC_OE_SYNC(clock, mc_oe, mc_oe_sync);
    sync MC_CE_SYNC(clock, mc_ce, mc_ce_sync);

    // Tristate pin handling
    // Bus Pirate IO pins
    wire [BP_PINS-1:0] bpio_toe, bpio_tdo, bpio_tdi;
    // BP IO pin control wires
    wire [BP_PINS-1:0] bpio_oe,bpio_di,bpio_do,bpio_dir;
    // DIO control regiter for pins not used by the peripheral
    reg [BP_PINS-1:0] bpio_dio_tris_d, bpio_dio_port_d;

    reg bp_busy;
    assign bp_active=in_fifo_out_nempty || peripheral_busy || bp_busy;

    //TODO: register for alt or io mode like STM...
    iobuf BPIO_BUF[BP_PINS-1:0] (
      //interface
      .oe(`REG_BPIO_OE), //bp_oe//output enable 1=true
      .od(`REG_BPIO_OD), //open drain 1=true
      .dir(bpio_dio_tris_d),//direction 1=input
      .din(bpio_do),//data in (value when buffer is output)
      .dout(bpio_di),//data out (value when buffer is input)
      //hardware driver
      .bufdir(bpio_buffer_dir), //74LVC1T45 DIR pin LOW for Hi-Z
      .bufod(bpio_buffer_od), //74LVC1G07 OD pin HIGH for Hi-Z
      .bufdat_tristate_oe(bpio_toe), //tristate data pin output enable
      .bufdat_tristate_dout(bpio_tdo), //tristate data pin data out
      .bufdat_tristate_din(bpio_tdi)  //tristate data pin data in
      );
      `define BP_PERIPHERAL_PINS 3
      assign bpio_do[BP_PINS-1:`BP_PERIPHERAL_PINS]=bpio_dio_port_d[BP_PINS-1:`BP_PERIPHERAL_PINS];
      //assign bpio_dir[BP_PINS-1:`BP_PERIPHERAL_PINS]=bpio_dio_tris_d[BP_PINS-1:`BP_PERIPHERAL_PINS];
      //assign bpio_dir[`BP_PERIPHERAL_PINS-1:0]=wreg[6'h01][`BP_PERIPHERAL_PINS-1+8:8];//`reg_bpio_dir[`BP_PERIPHERAL_PINS-1:0];
    // PWM
    //TODO: N:1 mux freq measure and PWM on IO pins?
    wire pwm_out;
    reg pwm_reset;
    reg [15:0] pwm_on,pwm_off;
    pwm PWM_OUT(pwm_reset,clock,pwm_out,pwm_on,pwm_off);
    //assign bpio_do[4]=pwm_out;

  	// Memory controller interface
  	wire [MC_DATA_WIDTH-1:0] mc_din;
    reg [MC_DATA_WIDTH-1:0] mc_dout_d;
    wire [MC_DATA_WIDTH-1:0] mc_dout;
    assign mc_dout=mc_dout_d;

    //LATCH OE
    assign lat_oe=1'b0; //open latch
    assign lat_dir=1'b0; //input to FPGA

    // PULLUP RESISTORS
    assign pullup_enable=`REG_HW_CONFIG_PULLUPS_EN; //pullups disable

    reg reset;
    reg [2:0] reset_count;
    //1111 0100 0010 0100 0000 0000


    wire [LA_WIDTH-1:0] sram_sio_tdi;
    wire [LA_WIDTH-1:0] sram_sio_tdo;

    reg [7:0] sram_out_d;
    reg la_start;
    wire sram_clock_source;
    reg sram_auto_clock, sram_auto_clock_delay;
    assign sram_clock_source=(la_start&&`reg_la_active)?clock:(`reg_la_io_quad)?sram_auto_clock:(`reg_la_io_spi)?mcu_clock:1'b0;
    assign sram_clock={sram_clock_source,sram_clock_source};
    assign sram_cs=(la_start&&`reg_la_active)?2'b00:{`reg_la_io_cs1,`reg_la_io_cs0}; //TODO: hold CS low during active?
    assign sram_sio_tdo[0]=(la_start&&`reg_la_active)?lat[0]:`reg_la_io_quad?sram_out_d[0]:mcu_mosi;
    assign sram_sio_tdo[4]=(la_start&&`reg_la_active)?lat[4]:`reg_la_io_quad?sram_out_d[4]:mcu_mosi;
    assign {sram_sio_tdo[7:5],sram_sio_tdo[3:1]}=(la_start&&`reg_la_active)?{lat[7:5],lat[3:1]}:{sram_out_d[7:5],sram_out_d[3:1]};
    assign mcu_miso=!`reg_la_io_cs0?sram_sio_tdi[1]:sram_sio_tdi[5]; //very hack dont like

    //FIFO
    wire bp_fifo_clear_sync;
    sync FIFO_CLEAR_SYNC(clock, bp_fifo_clear, bp_fifo_clear_sync);
    //IN FIFO
    wire [MC_DATA_WIDTH:0] in_fifo_out_data;
    wire in_fifo_in_nempty,in_fifo_out_nempty;
    reg in_fifo_in_shift,in_fifo_out_pop;
    //OUT FIFO
    reg [MC_DATA_WIDTH-1:0] out_fifo_in_data_d;
    wire [MC_DATA_WIDTH-1:0] out_fifo_out_data;
    wire out_fifo_in_nempty, out_fifo_in_full;
    reg out_fifo_in_shift,out_fifo_out_pop;
    //PERIPHERAL
    reg peripheral_trigger;
    wire peripheral_busy;
    wire[7:0] peripheral_data_out;
    reg [15:0] peripheral_data_in_d;

    fifo #(
      .WIDTH(FIFO_WIDTH+1),
      .DEPTH(FIFO_DEPTH)
    ) FIFO_IN (
      .reset(bp_fifo_clear_sync),
      .in_clock(clock),
      .in_shift(in_fifo_in_shift),
      .in_data({mc_add[0],mc_din}), //bit 0 of MC address is command/data indicator
      .in_full(bp_fifo_in_full),
      .in_nempty(in_fifo_in_nempty),
      .out_clock(clock),
      .out_pop(in_fifo_out_pop),
      .out_data(in_fifo_out_data),
      .out_nempty(in_fifo_out_nempty)
    );
    fifo #(
      .WIDTH(FIFO_WIDTH),
      .DEPTH(FIFO_DEPTH)
    ) FIFO_OUT (
      .reset(bp_fifo_clear_sync),
      .in_clock(clock),
    	.in_shift(out_fifo_in_shift), //???
    	.in_data(out_fifo_in_data_d), // in data
    	.in_full(out_fifo_in_full), //output
    	.in_nempty(out_fifo_in_nempty), //output
      .out_clock(clock),
    	.out_pop(out_fifo_out_pop), //input out_pop,
    	.out_data(out_fifo_out_data), //out data,
    	.out_nempty(bp_fifo_out_nempty) //output reg out_nempty
    );

    // ADC multiplexer
    reg adc_mux_en_d;
    reg [3:0] adc_mux_s_d;
    assign adc_mux_en=adc_mux_en_d;
    assign adc_mux_s=adc_mux_s_d;
    // ADC
    wire adc_busy;
    reg adc_trigger;
    wire [13:0] adc_data_out;
    // ADC serial input module
    adc ADC_SI (
      // general control
      	.rst(reset),				// resets module to known state
      	.clkin(clock),				// clock that makes everyhting tick
      // sync signals
      	.go(adc_trigger),					// starts a SPI transmission
      	.state(adc_busy),				// state of module (0=idle, 1=busy/transmitting)
        .calibrate(`REG_ADC_CALIBRATE),
      // data in/out
      	.data_o(adc_data_out),				// data out (will get received)
      // spi signals
      	.sclk(adc_clock),				// SPI clock (= clkin/2)
      	.miso(adc_data),				// master in slave out
      	.cs(adc_cs)					// chip select
      );

    // SPI master
     spimaster SPI_MASTER(
     // general control
       .rst(reset),				// resets module to known state
       .clkin(clock),				// clock that makes everyhting tick
     // spi configuration
       .cpol(1'b1), //cpol,				// clock polarity
       .cpha(1'b0), //cpha,				// clock phase
       .cspol(1'b1), //cspol,				// CS polarity
       //.autocs(1'b0), //autocs,				// assert CS automatically
     // sync signals
       .go(peripheral_trigger),					// starts a SPI transmission
       .state(peripheral_busy),				// state of module (0=idle, 1=busy/transmitting)
     // data in/out
       .data_i(peripheral_data_in_d), 			// data in (will get transmitted)
       .data_o(peripheral_data_out),				// data out (will get received)
     // spi signals
       .mosi(bpio_do[0]),          // master out slave in
       .sclk(bpio_do[1]),          // SPI clock (= clkin/2)
       .miso(bpio_di[2])          // master in slave out
       //.cs(bpio_do[3])				     // chip select
       );

    reg [15:0] delay_counter;

    `define STATE_IDLE   0
    `define STATE_WAIT    1
    `define STATE_DELAY 2
    `define STATE_PERIPHERAL_WAIT 3
    `define STATE_POP_FIFO 4
    `define STATE_CLEANUP 5
    `define STATE_HALT 6
    `define STATE_ADC_WAIT 7
    `define STATE_LA_READ_STATUS 8
    `define STATE_READ_REGISTER 9

    `define CMD_DIO_WRITE 8'h00
    `define CMD_DIO_READ 8'h01
    `define CMD_DIO_TRIS 8'h02

    `define CMD_PERIPHERAL_WRITE 8'h03
    `define CMD_PERIPHERAL_READ 8'h04

    `define CMD_DELAY 8'h05

    `define CMD_PWM_ON_PERIOD 8'h06
    `define CMD_PWM_OFF_PERIOD 8'h07

    `define CMD_ADC_READ 8'h08

    `define CMD_LA_START 8'h09
    `define CMD_LA_STOP 8'h0A

    `define CMD_REGISTER_SET_POINTER 8'h0B
    `define CMD_REGISTER_WRITE 8'h0C
    `define CMD_REGISTER_READ 8'h0D

    `define CMD_SM_HALT 8'h0F

    reg [$clog2(`STATE_HALT):0] bpsm_state; //,bpsm_next_state, bpsm_next_next_state;
    reg [$clog2(`CMD_SM_HALT):0] bpsm_command;

    always @(posedge clock)
      begin
      //some manual reset crap...need global reset pin, but this still may be needed accorting to various posts I've read
      if(reset_count<3) begin
        reset_count<=reset_count+1;
        reset<=1'b1;
      end
      else
      begin
        reset<=1'b0;
      end

      if(`reg_bpsm_reset||reset) begin
           bpsm_state <= `STATE_IDLE;
           peripheral_trigger <= 1'b0;
           out_fifo_in_shift<=1'b0;
           in_fifo_out_pop<=1'b0;
           adc_mux_en_d<=1'b1;
       end
       else
       begin

           in_fifo_in_shift<=1'b0;
           out_fifo_out_pop<=1'b0;
           if(!mc_ce)
           begin

             if (mc_we_sync)			// write
             begin
               case(mc_add)
                 6'h00:in_fifo_in_shift<=1'b1;
                 6'h01:in_fifo_in_shift<=1'b1;
                 6'h02:sram_auto_clock_delay<=1'b1;
               endcase
             end

             else if (mc_oe_sync)		// read
             begin
               case(mc_add)
                 6'h00: begin
                   mc_dout_d<=out_fifo_out_data;//remember if there is data in FIFO the first byte is already available
                   out_fifo_out_pop<=1'b1;//pop when done with this word
                 end
                 6'h01: begin
                   mc_dout_d<=out_fifo_out_data;//remember if there is data in FIFO the first byte is already available
                   out_fifo_out_pop<=1'b1;//pop when done with this word
                 end
                 6'h02:begin
                   sram_auto_clock_delay<=1'b1; //is this really stable? don't we need delay betwee the clock and the reading of the results?
                   mc_dout_d <= {8'h00,sram_sio_tdi}; //should move to if clock=1 in a clock delay?
                 end
                 6'h03:mc_dout_d[$clog2(`STATE_HALT):0]<=bpsm_state;
               endcase
             end
           end// if we or oe
         end //if ce

         //this can be done with assign sram_auto_clock=we_last/mc_oe_sync && !current?
         /*if(sram_auto_clock_delay)
         begin
           sram_auto_clock_delay<=1'b0;
           sram_auto_clock<=1'b1;
         end
         else if(sram_auto_clock)
         begin
           sram_auto_clock<=1'b0;
         end*/

         //main bus pirate state machine
         case(bpsm_state)

             `STATE_IDLE: begin

                 if(in_fifo_out_nempty&&!out_fifo_in_full) begin //check out_fifo not full because we slam the command back into the queue
                    bp_busy <= 1'b1;

                    if(in_fifo_out_data[16]===1'b1) begin //D/C bit high = command
                      bpsm_command <= in_fifo_out_data[$clog2(`CMD_SM_HALT):0];
                      //return the command so we can track progress from MCU
                      out_fifo_in_data_d<=in_fifo_out_data;
                      out_fifo_in_shift<=1'b1;
                      bpsm_state <= `STATE_POP_FIFO;
                    end //end if command

                    else begin //begin else data

                    bpsm_state<=`STATE_POP_FIFO; //default pop, otherwise handed to the next state forced below...

                     case(bpsm_command)
                       `CMD_DIO_WRITE: begin//todo:change to set/clear/write...
                        bpio_dio_port_d<= in_fifo_out_data[BP_PINS-1:0];
                       end
                       `CMD_DIO_READ:
                          begin
                          out_fifo_in_data_d<=bpio_di;//this may need a delay!!!
                          out_fifo_in_shift<=1'b1;
                          end
                       `CMD_DIO_TRIS:
                           bpio_dio_tris_d <= in_fifo_out_data[BP_PINS-1:0];
                       `CMD_PERIPHERAL_WRITE: begin
                          peripheral_data_in_d <= in_fifo_out_data; //use extra register so we can pop the FIFO on this loop
                          peripheral_trigger <= 1'b1;
                          bpsm_state<=`STATE_PERIPHERAL_WAIT;
                          end
                       //`CMD_PERIPHERAL_READ:
                       `CMD_DELAY:begin
                           delay_counter<=in_fifo_out_data;
                           bpsm_state <= `STATE_DELAY;
                           end
                       `CMD_PWM_ON_PERIOD:
                          pwm_on<=in_fifo_out_data;
                       `CMD_PWM_OFF_PERIOD:begin
                          pwm_off<=in_fifo_out_data;
                          pwm_reset<=1'b1;
                          end
                       `CMD_ADC_READ: begin
                          adc_mux_en_d<=1'b0;
                          adc_mux_s_d<=in_fifo_out_data[3:0];
                          adc_trigger <= 1'b1;
                          bpsm_state<=`STATE_ADC_WAIT;
                          end
                        `CMD_LA_START:
                           la_start<=1'b1;
                        `CMD_LA_STOP:
                           la_start<=1'b0;
                        `CMD_SM_HALT:
                          bpsm_state <= `STATE_HALT;
                        `CMD_REGISTER_SET_POINTER:
                          reg_index <= in_fifo_out_data[3:0];
                        `CMD_REGISTER_WRITE: begin
                          config_register[reg_index]<=in_fifo_out_data;
                          reg_index<=reg_index+1;
                        end
                        `CMD_REGISTER_READ: begin
                          reg_count<=in_fifo_out_data[3:0];
                          bpsm_state<= `STATE_READ_REGISTER;
                          end
                       default: begin
                           //$display("ERROR: unknown command!");
                           //$display(bpsm_state);
                           //$stop;
                           //error<=1'b1;
                       end
                     endcase
                    end //end else data

                 end else begin //end if in_fifo_out_nempty, else set not busy
                    bp_busy <= 1'b0;
                 end

             end //end STATE_IDLE

           `STATE_DELAY: begin
                if (delay_counter === 0) begin
                    bpsm_state <= `STATE_POP_FIFO;
                end else begin
                    delay_counter <= delay_counter - 1;
                end
            end

            `STATE_PERIPHERAL_WAIT: begin
                peripheral_trigger <= 1'b0;
                if (!peripheral_trigger && !peripheral_busy && !out_fifo_in_full) begin
                    out_fifo_in_data_d<={8'h00,peripheral_data_out};
                    out_fifo_in_shift<=1'b1;
                    bpsm_state <= `STATE_POP_FIFO;
                end
            end

            `STATE_ADC_WAIT: begin
                out_fifo_in_shift<=1'b0;
                adc_trigger <= 1'b0;
                if (!adc_trigger && !adc_busy && !out_fifo_in_full) begin
                    out_fifo_in_data_d<={4'h0,adc_data_out};
                    out_fifo_in_shift<=1'b1;
                    bpsm_state <= `STATE_POP_FIFO;
                end
            end

           `STATE_READ_REGISTER: begin
              if(out_fifo_in_shift===1'b1) begin
                out_fifo_in_shift<=1'b0;
                reg_index<=reg_index+1;
                reg_count<=reg_count-1;
                if(reg_count===0)
                   bpsm_state <= `STATE_IDLE;
              end
              else if (!out_fifo_in_full) begin
                  out_fifo_in_data_d<=config_register[reg_index];
                  out_fifo_in_shift<=1'b1;
              end
            end

            //when a word enters the FIFO and nempty goes high
            //that first word is already in the output
            //poping the fifo removes that word to empty the FIFO or load the next word if nempty is HIGH
            //so we need to pop at the end of acting on the command
            //there are several ways to save one clock but I'll worry about that later
            `STATE_POP_FIFO: begin
              pwm_reset<=1'b0;
              out_fifo_in_shift<=1'b0;
              in_fifo_out_pop<=1'b1;
              bpsm_state <= `STATE_CLEANUP;
             end

             `STATE_CLEANUP: begin
               in_fifo_out_pop<=1'b0;
               bpsm_state <= `STATE_IDLE;
              end

            `STATE_HALT: begin
              `reg_bpsm_reset<=1'b1; //self reset
              out_fifo_in_shift<=1'b0;
              in_fifo_out_pop<=1'b1;
             end

          endcase //bpsm_state
        end //if reset else bpsm_state



/*

        //TODO:
        //prescale to 0xff and capture to nearest 0xff
        if(`reg_la_clear_sample_counter)
        begin
          `reg_la_sample_count<=16'h0000;
          `reg_la_clear_sample_counter<=1'b0;
          `reg_la_max_samples_reached<=1'b0;
        end
        else if(la_start)
        begin
          if(`reg_la_sample_count<16'hF424) //F424
          begin
            `reg_la_sample_count<=`reg_la_sample_count+1;
            `reg_la_active<=1'b1;
          end
          else
            begin
              `reg_la_active<=1'b0;
              `reg_la_max_samples_reached<=1'b1;
            end
        end
        else
        begin
          `reg_la_active<=1'b0;
        end

*/



    //define the tristate data pin explicitly in the top module
    //register MOSI
    SB_IO #(
      .PIN_TYPE(6'b1010_01), //tristate
      .PULLUP(1'b0)          //no pullup
    ) sram_mosi_sbio[LA_CHIPS-1:0] (
      .PACKAGE_PIN({sram_sio[4],sram_sio[0]}),//which pin
      .OUTPUT_ENABLE(`reg_la_io_quad?`reg_la_io_quad_direction:1'b1),   //output enable wire mcu_quadmode?mcu_direction:1'b1
      .D_OUT_0({sram_sio_tdo[4],sram_sio_tdo[0]}),        //data out wire
      .D_IN_0({sram_sio_tdi[4],sram_sio_tdi[0]})           //data in wire
    );
    //register MISO
    SB_IO #(
			.PIN_TYPE(6'b1010_01), //tristate
			.PULLUP(1'b0)          //no pullup
		) sram_miso_sbio[LA_CHIPS-1:0](
			.PACKAGE_PIN({sram_sio[5],sram_sio[1]}),//which pin
			.OUTPUT_ENABLE(`reg_la_io_quad?`reg_la_io_quad_direction:1'b0),   //output enable wire mcu_quadmode?mcu_direction:1'b0
			.D_OUT_0({sram_sio_tdo[5],sram_sio_tdo[1]}),        //data out wire
			.D_IN_0({sram_sio_tdi[5],sram_sio_tdi[1]})           //data in wire
		);
    //register SIO2/3
    SB_IO #(
			.PIN_TYPE(6'b1010_01), //tristate
			.PULLUP(1'b0)          //no pullup
		) sram_sio_sbio[3:0] (
			.PACKAGE_PIN({sram_sio[7:6],sram_sio[3:2]}),//which pin
			.OUTPUT_ENABLE(`reg_la_io_quad&&`reg_la_io_quad_direction),   //quadmode = 1 and direction = 1
			.D_OUT_0({sram_sio_tdo[7:6],sram_sio_tdo[3:2]}),        //data out wire
			.D_IN_0({sram_sio_tdi[7:6],sram_sio_tdi[3:2]})           //data in wire
		);
    // Memory controller data pins
    SB_IO #(
      .PIN_TYPE(6'b1010_01),
      .PULLUP(1'b0)
    ) mc_io [MC_DATA_WIDTH-1:0] (
      .PACKAGE_PIN(mc_data),
      .OUTPUT_ENABLE(!mc_oe),
      .D_OUT_0(mc_dout),
      .D_IN_0(mc_din)
    );
    // Bus Pirate IO pins
    SB_IO #(
      .PIN_TYPE(6'b1010_01),
      .PULLUP(1'b0)
    ) bpio_tio [BP_PINS-1:0] (
      .PACKAGE_PIN(bpio_buffer_io),
      .OUTPUT_ENABLE(bpio_toe),
      .D_OUT_0(bpio_tdo),
      .D_IN_0(bpio_tdi)
    );

`ifdef SIMULATION
    initial begin
      reset<=1'b0;
      reset_count<=3'b000;
      bp_busy <= 1'b0;
      la_start<=1'b0;

      //LA sample counter debugging
      //wire [15:0] reg_la_sample_count;
      //assign reg_la_sample_count=`reg_la_sample_count;
    end
  `endif

endmodule
