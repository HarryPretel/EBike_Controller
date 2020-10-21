// Connor's Reset Synch
module rst_synch(RST_n, clk, rst_n);
	input logic RST_n, clk;
	logic ftof;
	output logic rst_n;

	always@(negedge clk, negedge RST_n) begin
	  if(!RST_n) begin
		ftof <= 1'b0;
		rst_n <= 1'b0;
	  end
	  else begin
		ftof <= 1'b1;
		rst_n <= ftof;
	  end
	end

endmodule
