// Connor and Jacob
module pushButton(tgglMd, setting, rst_n, clk);
	input tgglMd, clk, rst_n;
	logic q1, q2, rise_edge;
	output logic [1:0] setting;

	// Setting counter
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			setting <= 2'b10;
		else if(rise_edge) begin
			setting <= setting + 1;
		end
	end

	// Rising edge detector
	always @(posedge clk) begin
		q1 <= tgglMd;
		q2 <= q1;
		rise_edge <= (!q2 & q1);
	end

endmodule
