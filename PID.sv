// Connor and Roshan PID
module PID(clk, rst_n, error, not_pedaling, drv_mag);
	input clk, rst_n, not_pedaling;
	input [12:0] error;
	output logic[11:0] drv_mag;
	logic [13:0] p_term; // 14-bit sign extended version of error
	logic [17:0] sign_extended; // Sign extends error to 18 bits
	// Stuff for I Term
	logic [17:0] error_plus_integrator, integrator, check_neg, overflow_out, clock_check, check_pedaling;
	logic pos_ov;
	logic [11:0] i_term;
	// Stuff for D Term
	logic [9:0] d_term;
	logic [12:0] mux1, mux2, mux3, ff1, ff2, prev_err;
	logic [12:0] D_diff;
	logic [8:0] D_diff_sat;
	// Stuff for PID
	logic [13:0] PID;
	logic [11:0] intermediate_drive;
	// Stuff for Counter
	logic [19:0] counter;
	// Stuff for FAST SIM
	logic decimator_full;
	parameter FAST_SIM = 0;

	// Create the p_term as a sign extended version of error // 
	assign p_term = {error[12], error};	

	///// Create the i_term /////
	assign sign_extended = {{5{error[12]}}, error};
	assign error_plus_integrator = sign_extended + integrator;
	assign check_neg = error_plus_integrator[17] ? 18'h00000 : error_plus_integrator;
	assign pos_ov = integrator[16] & error_plus_integrator[17];
	assign overflow_out = pos_ov ? 18'h1FFFF : check_neg;
	assign clock_check = decimator_full ? overflow_out : integrator;
	assign check_pedaling = not_pedaling ? 18'h00000 : clock_check;
	// Flop the integrator value
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n) 
			integrator <= 18'h00000;
		else
			integrator <= check_pedaling;
	end

	assign i_term = integrator[16:5];

	///// Create the d_term /////
	// Assign the muxes
	assign mux1 = decimator_full ? error : ff1;
	assign mux2 = decimator_full ? ff1 : ff2;
	assign mux3 = decimator_full ? ff2: prev_err;
	// Define the flip flops
	always@(posedge clk, negedge rst_n) begin
		if(!rst_n) begin
			ff1 <= 13'h0000;
			ff2 <= 13'h0000;
			prev_err <= 13'h0000;
		end
		else begin
			ff1 <= mux1;
			ff2 <= mux2;
			prev_err <= mux3;
		end
	end
	// Define the difference
	assign D_diff = error - prev_err;
	// Saturate the difference to 9-bits
	assign D_diff_sat = ~D_diff[12] ? (|D_diff[11:8] ? (9'h0FF) : {1'b0, D_diff[7:0]}) : (~(&D_diff[11:8]) ? (9'h100) : (D_diff[8:0]));
	// Signed Multiply by 2
	assign d_term = {D_diff_sat, 1'b0};

	///// Sum all of the terms to create PID /////
	assign PID = {{4{d_term[9]}}, d_term} + p_term + {2'b00, i_term};
	assign intermediate_drive = PID[12] ? 12'hFFF : PID[11:0];

	// Flop drive mag to achieve better timing
	always @(posedge clk) 
		drv_mag <= PID[13] ? 12'h000 : intermediate_drive;

	// Create a counter to count 1/48th of a second //
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n) 
			counter <= 20'h00000;
		else 
			counter <= counter + 1;
	end

	// Create the fast simulation possibilities
	generate if(FAST_SIM)
		assign decimator_full = &counter[14:0];
	else
		assign decimator_full = &counter;
	endgenerate

endmodule
