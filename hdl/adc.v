// ---------------------------------------------------------------------------
//
// SPI master verilog implementation
//
// Written by Chris van Dongen/SMDprutser for the buspirate NextGen Ultra
// SPI info here: https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
//
// ---------------------------------------------------------------------------
`ifndef __ADC__
`define __ADC__
`define ADC_BITS 14
`define ADC_CALIBRATION_BITS 32
module adc (
// general control
	rst,				// resets module to known state
	clkin,				// clock that makes everyhting tick
	clk_divider,
// sync signals
	go,					// starts a SPI transmission
	state,				// state of module (0=idle, 1=busy/transmitting)
	calibrate,
// data in/out
	data_o,				// data out (will get received)
// spi signals
	sclk,				// SPI clock (= clkin/2)
	miso,				// master in slave out
	cs					// chip select
	);

//
input rst;
input clkin;
input [2:0] clk_divider;

input go;
output reg state;
input calibrate;

output reg [`ADC_BITS-1:0] data_o;
reg [4:0] bitcount;		// number of bits to transmit

reg [2:0] delay;

input miso;
output reg sclk;
output reg cs;

// internal vars
reg clockphase;			// where are we in our clockcycle (0= 1st half, 1= 2nd half)

always @ (posedge clkin or posedge rst)

if(rst)	begin
		data_o <= 16'h00;
		sclk <= 1'b0;
		state <= 1'b0;
		cs <= 1'b1;
	end
else begin

		if(state === 1'b0) begin // only accept go when we are idle

			if(go === 1'b1) begin		//start sending bits
					state <= 1'b1;
					//sclk <= 1'b0;
					cs <= 1'b0;			// cs line (auto?????)
					data_o <= 16'h00;
					delay<=clk_divider;
					if(calibrate===1'b1)
						bitcount <= `ADC_CALIBRATION_BITS-1;
					else
						bitcount <= `ADC_BITS-1;
					end
			else begin //go === 1'b0 idle everything
					cs <= 1'b1;			// cs line
				end
			end

		else begin //state === 1'b1					// transmit the bits and receive them

			if(delay!==3'b000) begin
				delay<=delay-1;
				end
			else begin
				delay<=clk_divider;
				if(sclk === 1'b0)	begin //clock low, setup data and tick the clock line
					sclk <= 1'b1;
					end
		  	else begin //clock high, read data and clear clock line
					if(calibrate===1'b0)
						data_o[bitcount] <= miso;
					sclk <= 1'b0;
					bitcount <= bitcount - 1;
					if(bitcount===0)
							state<=1'b0;
					end
				end

		end //end state === 1'b1

	end //end reset

endmodule
`endif
