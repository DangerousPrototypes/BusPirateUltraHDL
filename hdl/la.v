`ifndef __LA__
`define __LA__

module logic_analyzer #(
  parameter SAMPLES = 24'hF42400
)(
  input clock,
  input reset,
  input start,
  output active,
  output max_samples_reached,
  output [$clog2(SAMPLES):0] sample_count
);
//1111 0100 0010 0100 0000 0000 = 24'hF42400
reg [$clog2(SAMPLES):0] sample_count_d;
reg max_samples_reached, active;
assign sample_count = sample_count_d;

always @ (posedge clock ) begin

  if(reset) begin
    sample_count_d<=0;
    max_samples_reached<=1'b0;
    active<=1'b0;
    end

  else if(start) begin

    if(sample_count_d<SAMPLES) begin
      sample_count_d<=sample_count_d+1;
      active<=1'b1;
      end
    else begin
      active<=1'b0;
      max_samples_reached<=1'b1;
      end

  end else begin
    active<=1'b0;
    end

end//end always


endmodule
`endif
