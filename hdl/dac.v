// ---------------------------------------------------------------------------
//
// SPI master verilog implementation
//
// Written by Chris van Dongen/SMDprutser for the buspirate NextGen Ultra
// SPI info here: https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
//
// ---------------------------------------------------------------------------
`ifndef __DAC__
`define __DAC__

module dac (
// general control
	rst,				// resets module to known state
	clkin,				// clock that makes everyhting tick
// sync signals
	go,					// starts a SPI transmission
	state,				// state of module (0=idle, 1=busy/transmitting)
	calibrate,
// data in/out
	data_i,				// data in (will get transmitted)
// spi signals
	sclk,				// SPI clock (= clkin/2)
	mosi,				// master in slave out
	sync					// chip select
	);

//
input rst;
input clkin;

input go;
output reg state;

input [15:0] data_i;

reg [2:0] delay;

output reg mosi;
output reg sclk;
output reg sync;

// internal vars
reg clockphase;			// where are we in our clockcycle (0= 1st half, 1= 2nd half)
reg [3:0] bitcount;

always @ (posedge clkin or posedge rst)

if(rst)
	begin
		data_i <= 16'h00;
		sclk <= 1'b0;
		state <= 1'b0;
		cs <= 1'b1;
	end
else
	begin
		if(state === 1'b0)
			begin
			if(go === 1'b1)		// only accept go when we are idle
				begin	//start sending bits
					state <= 1'b1;
					sync <= 1'b0;			// cs line
					delay<=3'b111;
					bitcount<=4'hF;
				end
			else //go === 1'b0
				begin	//idle everything
					cs <= 1'b1;			// cs line
				end
			end

		else //state === 1'b1					// transmit the bits and receive them
		begin
			if(sclk === 1'b0)
				begin //clock low, setup data and tick the clock line
					delay<=delay-1;
					if(delay===3'b000)
						begin
							mosi<=data_i[bitcount];
							sclk <= 1'b1;
							delay<=3'b111;
						end
				end
		  else
				begin //clock high, read data and clear clock line
					delay<=delay-1;
					if(delay===3'b000)
						begin
							sclk <= 1'b0;
							bitcount <= bitcount - 1;
							delay<=3'b111;
							if(bitcount===0)
								begin
									state<=1'b0;
								end
							end
				end
		end
	end

endmodule
`endif
