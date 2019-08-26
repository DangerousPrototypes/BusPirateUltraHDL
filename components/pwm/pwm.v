// ---------------------------------------------------------------------------
//
// pwm module
//
//	
//
// Written by Chris van Dongen/SMDprutser for the buspirate NextGen Ultra
// SPI info here: https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
//
// ---------------------------------------------------------------------------



module pwm (
rst,					// reset
clkin,					// clock in
clkout,					// clock out
onperiod,				// #ticks period ontime
offperiod				// #ticks period offtime
);

input rst;
input clkin;
output reg clkout;
input [16:0] onperiod;
input [16:0] offperiod;

reg [16:0] count;
reg [16:0] lastonperiod;
reg [16:0] lastoffperiod;

always @ (posedge clkin or posedge rst)
if (rst)
begin
	count = 16'b0000000000000000;
	clkout = 1'b0;
end
else
begin
	if (onperiod != lastonperiod)		// reset count if onperiod has changed
		count = 16'b0;
	if (offperiod != lastoffperiod)		// reset count if offperiod has changed
		count = 16'b0;
		
	lastonperiod = onperiod;
	lastoffperiod = offperiod;

	if (clkout == 0)
	begin
		if (count == offperiod)				// flip clock after halfperiod clocks
		begin
			count = 16'b0000000000000000;
			clkout = 1;
		end
		else
		begin
			count = count + 1'b1;
		end
	end
	else
	begin
		if (count == onperiod)				// flip clock after halfperiod clocks
		begin
			count = 16'b0000000000000000;
			clkout = 0;
		end
		else
		begin
			count = count + 1'b1;
		end
	end

end

endmodule