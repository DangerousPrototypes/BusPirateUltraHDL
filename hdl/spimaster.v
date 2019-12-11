// ---------------------------------------------------------------------------
//
// SPI master verilog implementation
//
// Written by Chris van Dongen/SMDprutser for the buspirate NextGen Ultra
// SPI info here: https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
//
// ---------------------------------------------------------------------------
`ifndef __SPIMASTER__
`define __SPIMASTER__
module spimaster (
// general control
	rst,				// resets module to known state
	clkin,				// clock that makes everyhting tick
// spi configuration
	cpol,				// clock polarity
	cpha,				// clock phase
	cspol,				// CS polarity
	//autocs,				// assert CS automatically

// sync signals
	go,					// starts a SPI transmission
	state,				// state of module (0=idle, 1=busy/transmitting)
// data in/out
	data_i,				// data in (will get transmitted)
	data_o,				// data out (will get received)
// spi signals
	mosi,				// master out slave in
	sclk,				// SPI clock (= clkin/2)
	miso				// master in slave out
	//cs					// chip select
	);

//
input rst;
input clkin;
input cpol;
input cpha;
input cspol;
input clk_divider;
//input autocs;
input go;
output reg state;
input [15:0] data_i;
output reg [7:0] data_o;
input miso;
output reg mosi;
output reg sclk;
//output reg cs;

// internal vars
reg [4:0] bitcount;		// number of bits to transmit
reg clockphase;			// where are we in our clockcycle (0= 1st half, 1= 2nd half)

always @ (posedge clkin or posedge rst)

if(rst) begin
	data_o <= 8'b0;
	mosi <= 1'b0;
	sclk <= 1'b0;
	state <= 1'b0;
	//cs <= 1'b0;
	bitcount <=5'b00000;
	end

else begin

	if(state === 1'b0) begin // only accept go when we are idle
		if(go === 1'b1)	begin
			bitcount <= data_i[11:8]; //adjust the number of bits to send
			state <= 1'b1;
			clockphase <= 1'b0;
			end
		else begin	// put lines into their idle state
			sclk <= cpol;						// clock line
			//if (autocs) cs <= cspol;			// cs line
			end
		end

	else begin //if(state === 1'b1)						// transmit the bits and receive them

		if(bitcount === 5'b00000) begin
			state<=1'b0;
			end
		else if (clockphase === 1'b0)	begin

			if (cpha === 1'b0) begin
				mosi <= data_i[bitcount-1];
				end
			else begin
				data_o[bitcount] <= miso;
				end

			sclk <= cpol;
			clockphase <= 1'b1;
			end

		else begin //clock phase === 1'b1
			if (cpha === 1'b0) begin
				data_o[bitcount-1] <= miso;
				end
			else begin
				mosi <= data_i[bitcount-1];
				end

			sclk <= ~cpol;
			bitcount <= bitcount - 1;
			clockphase <= 1'b0;
			end //end bitcount and clock phase

		if(bitcount === 0) begin
			state <= 1'b0;
			mosi <= data_i[0];
			end
		//if (autocs) cs <= ~cspol;
	end //end state === 1
end //end reset === 0
endmodule
`endif
