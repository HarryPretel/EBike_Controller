module cadence_filt(clk, rst_n, cadence, cadence_filt);
	input clk;
	input rst_n;
	input cadence;
	output cadence_filt;
	parameter FAST_SIM = 0;

	logic stbl_cnt_fs;
	reg[15:0] stbl_cnt;
	logic pre_cadence_filt, d1, chngd_n;
	reg q1, q2, q3, stable, cadence_filt;

	// Triple Flop w/ first two for metastability
	always @(posedge clk) begin
		q1 <= cadence;
		q2 <= q1;
		q3 <= q2;
	end

	// Set up the input for the rst_n flip flop
	assign chngd_n = ~(q2 ^ q3);
	assign d1 = chngd_n & (stbl_cnt + 1'b1);
	
	// Rst-n flip flop
	always @(posedge clk, negedge rst_n) begin
		if(~rst_n)
		   stbl_cnt <= 16'd0;
		else
		   stbl_cnt <= d1;
	end

	// Wire going into mux that outputs cadence_filt
	//assign pre_cadence_filt = &stbl_cnt_fs ? (q3) : (cadence_filt);

	// Flip flop that outputs cadence_filt
	always @(posedge clk, negedge rst_n) begin
		if(~rst_n)
			cadence_filt <= 1'h0;
		else if(&stbl_cnt_fs)
			cadence_filt <= q3;
	end
	
	generate if(FAST_SIM)
		assign stbl_cnt_fs = stbl_cnt[8:0];
	else
		assign stbl_cnt_fs = stbl_cnt;
	endgenerate

endmodule
