// Connors PWM11
module PWM11(clk, rst_n, duty, PWM_sig);
	input clk, rst_n;
	input[10:0] duty;
	output reg PWM_sig;
	reg[10:0] cnt;
	wire PWM_input;
	
	// While the cnt is less then the duty cycle, we want
	// PWM input to be 1 so it's high for the right # of clock cycles
	assign PWM_input = cnt < duty;
	
	// This flip flop controls the PWM and acts as a counter
	always @(posedge clk, negedge rst_n) begin
		// On a reset, set count to 0 and the PWM_sig to low
		if(~rst_n) begin
			cnt <= 11'h000;
			PWM_sig <= 1'b0;
		end
		// Increment counter and set the PWM signal to the
		// value of cnt < duty calculated above
		else begin
			cnt <= cnt + 1'b1;
			PWM_sig <= PWM_input;
		end
	end
	
endmodule
