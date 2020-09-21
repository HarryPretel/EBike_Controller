// Connor and Roshan Sensor Condition
module SensorCondition(clk, rst_n, torque, cadence, curr, incline, setting, batt, error, not_pedaling, TX);
	input clk, rst_n, cadence;
	input [11:0] torque, curr, batt;
	input [12:0] incline;
	input [1:0] setting;
	output signed [12:0] error;
	output logic not_pedaling, TX;
	logic [24:0] cadence_per;
	logic cadence_filt, prev_cadence; //holds output from cadence_filt block
	logic [4:0] cadence_cnt, cadence_vec; // 5-bit signal to accumulate cadence_vec
	logic [21:0] include_smpl; // Counter for exponential average
	logic [13:0] curr_accum; // Accumulates average current
	logic [11:0] avg_curr;
	logic [11:0] target_curr;
	logic prev_not_pedal, not_pedaling_edge;
	logic [16:0] torque_accum; // Accumulates average torque
	logic [11:0] avg_torque;
	// Stuff for FAST SIM
	logic cadence_per_fs;
	logic include_smpl_fs;
	parameter FAST_SIM = 0;

	// Set the localparam
	localparam LOW_BATT_THRES = 12'hA98;

	// Instantiate DesiredDrive
	desiredDrive DDrive(.avg_torque(avg_torque), .cadence_vec(cadence_vec), .incline(incline), .setting(setting), .target_curr(target_curr), .clk(clk));

	// Instantiate Cadence Filter
	cadence_filt #(FAST_SIM) Cadence(.clk(clk), .rst_n(rst_n), .cadence(cadence), .cadence_filt(cadence_filt));

	// Instantiate telemetry (check if batt is correct batt_v input)
	telemetry Telem(.clk(clk), .rst_n(rst_n), .batt_v(batt), .avg_curr(avg_curr), .avg_torque(avg_torque), .TX(TX));
	
	////////// Error Section //////////////
	assign error = (batt < LOW_BATT_THRES) ? 0 : (not_pedaling ? 0 : target_curr - avg_curr);

	////////// Cadence Section //////////// (May have to saturate)

	// Flop for cadence_vec, double flopped for counting rising edges
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n) begin
			prev_cadence <= 1'h0;
			cadence_cnt <= 5'h00;
		end
		else if(cadence_per_fs) begin
			prev_cadence <= 1'h0;
			cadence_cnt <= 5'h00;
		end
		else if(~(&cadence_cnt)) begin
			prev_cadence <= cadence_filt;
			cadence_cnt <= (!prev_cadence & cadence_filt) ? cadence_cnt + 1 : cadence_cnt;
		end
	end

	// Update cadence_vec after a full cycle
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n) 
			cadence_vec <= 5'h00;
		else if(cadence_per_fs)
			cadence_vec <= cadence_cnt;
	end

	// Assign not pedaling if cadence vec is too low
	//assign not_pedaling = (cadence_vec < 5'h02) ? 1 : 0;

	// 0.67 sec counter for Cadence
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			cadence_per <= 25'h0000000;
		else
			cadence_per <= cadence_per + 1;
	end

	// Falling edge detector for not_pedaling (look here if error)
	always @(posedge clk) begin
		prev_not_pedal <= not_pedaling;
		not_pedaling <= (cadence_vec < 5'h02) ? 1 : 0;
	end
		
	// Edge detector (look here if error)
	assign not_pedaling_edge = prev_not_pedal & ~not_pedaling;

	///////// Current Exponential Average ////////////

	// Counter for including the sample
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			include_smpl <= 22'h000000;
		else
			include_smpl <= include_smpl + 1;
	end

	// Average the Current
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			curr_accum <= 14'h0000;
		else if(include_smpl_fs) begin
			curr_accum <= ((curr_accum*3)/4) + curr;
		end
	end

	// Make average curr the accumulator divided by 4
	assign avg_curr = curr_accum[13:2];

	////////// Torque Exponential Average ///////////

	// Flop the torque exponential average
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			torque_accum <= 17'h00000;
		// If not_pedaling_edge we seed the value
		else if(not_pedaling_edge)
			torque_accum <= {1'b0, torque, 4'h0};
		// Check for rising edge of cadence filt
		else if(!prev_cadence & cadence_filt)
			torque_accum <= ((torque_accum * 31)/32) + torque;
	end

	// Make average torque the accumulated torque divided by 32
	assign avg_torque = torque_accum[16:5];

	// Create the fast simulation possiblities
	generate if(FAST_SIM) begin
		assign cadence_per_fs = &cadence_per[15:0];
		assign include_smpl_fs = &include_smpl[15:0];
	end
	else begin
		assign cadence_per_fs = &cadence_per;
		assign include_smpl_fs = &include_smpl;
	end
	endgenerate

endmodule
