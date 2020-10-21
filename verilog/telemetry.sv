// Connors telemetry
module telemetry(clk, rst_n, batt_v, avg_curr, avg_torque, TX);
	input clk, rst_n;
	output TX;
	input[11:0] batt_v, avg_curr, avg_torque;

	reg[19:0] counter;
	logic[7:0] tx_data;
	logic trmt, tx_done;
	wire cnt_done;

	// Instantiate UART TX
	UART_tx UART(.clk(clk), .rst_n(rst_n), .TX(TX), .tx_done(tx_done), .tx_data(tx_data), .trmt(trmt));

	// Define the states
	typedef enum reg[3:0]{idle, delim1, delim2, payload1, payload2, payload3, payload4, payload5, payload6} state_t;
	state_t state, next_state;

	// 20-bit counter to wait 1/48th a second between cycles
	always @(posedge clk) begin
		// Default to a full counter so data transmission starts immediately
		if(!rst_n)	
			counter <= 20'hFFFFF;
		// Increment Counter
		else 
			counter <= counter + 1;
	end

	// cnt_done is asserted when the counter is full
	assign cnt_done = &counter;
	
	// Updating States
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			state <= idle;
		else
			state <= next_state;
	end

	always_comb begin
		// Default to 00 data, trnt to 0 and next_state to the current state
		tx_data = 8'h00;
        trmt = 0;
        next_state = state;
		case(state)
			// If it is in idle and the cnt is full, transition into the first state w/ data and 
			// set trmt so transmitter knows data is ready
			idle: begin
			   if(cnt_done) begin
				next_state = delim1;
				tx_data = 8'hAA;
                trmt = 1;
			   end
			end
			// If it is in delim1 and tx_done is asserted the previous read is completed, 
			// transition into the second state w/ data and set trmt so transmitter knows 
			// next set of data is ready
			delim1: begin
			   if(tx_done) begin
				next_state = delim2;
				tx_data = 8'h55;
				trmt = 1;
			   end
			end
			// If it is in delim2 and tx_done is asserted the previous read is completed, 
			// transition into the third state w/ tx_data set and set trmt so transmitter knows 
			// next set of data is ready
			delim2: begin
			   if(tx_done) begin
				next_state = payload1;
				tx_data = {4'h0, batt_v[11:8]};
				trmt = 1;
			   end
			end
			// If it is in payload1 and tx_done is asserted the previous read is completed, 
			// transition into the fourth state w/ tx_data set and set trmt so transmitter knows 
			// next set of data is ready
			payload1: begin
			   if(tx_done) begin
				next_state = payload2;
				tx_data = batt_v[7:0];
				trmt = 1;
			   end
			end
			// If it is in payload2 and tx_done is asserted the previous read is completed, 
			// transition into the fifth state w/ tx_data set and set trmt so transmitter knows 
			// next set of data is ready
			payload2: begin
			   if(tx_done) begin
				next_state = payload3;
				tx_data = {4'h0, avg_curr[11:8]};
				trmt = 1;
			   end
			end
			// If it is in payload3 and tx_done is asserted the previous read is completed, 
			// transition into the sixth state w/ tx_data set and set trmt so transmitter knows 
			// next set of data is ready
			payload3: begin
			   if(tx_done) begin
				next_state = payload4;
				tx_data = {avg_curr[7:0]};
				trmt = 1;
			   end
			end
			// If it is in payload4 and tx_done is asserted the previous read is completed, 
			// transition into the seventh state w/ tx_data set and set trmt so transmitter knows 
			// next set of data is ready
			payload4: begin
			   if(tx_done) begin
				next_state = payload5;
				tx_data = {4'h0, avg_torque[11:8]};
				trmt = 1;
			   end
			end
			// If it is in payload5 and tx_done is asserted the previous read is completed, 
			// transition into the final state w/ tx_data set and set trmt so transmitter knows 
			// next set of data is ready
			payload5: begin
			   if(tx_done) begin
				next_state = payload6;
				tx_data = {avg_torque[7:0]};
				trmt = 1;
			   end
			end
			// If it is in payload6 and tx_done is asserted the previous read is completed, 
			// transition back into idle and wait for the counter to fill before sampling the
			// data again and restarting the state machine
			payload6: begin
			   if(tx_done) begin
				next_state = idle;
				tx_data = 8'h00;
			   end
			end
			default: next_state = idle;
		endcase
	end

endmodule
