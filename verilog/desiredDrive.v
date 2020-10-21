// Connors Desired Drive
module desiredDrive(avg_torque, cadence_vec, incline, setting, target_curr, clk);
	input clk;
	input[12:0] incline;
	input[11:0] avg_torque;
	input[4:0] cadence_vec;
	input[1:0] setting;
	output reg[11:0] target_curr;
	wire[9:0] incline_sat;
	wire[10:0] incline_factor;
	wire[8:0] incline_lim;
	wire[5:0] cadence_factor; //cadence_factor should be 6 bits
	wire[12:0] torque_off;
	wire[11:0] torque_pos;
	reg[13:0] a_1;
	reg[14:0] a_2;
	wire[28:0] assist_prod;
	wire MSB;
	localparam TORQUE_MIN = 12'h380;

	// Handle the Saturation
	assign MSB = incline[12];
	assign incline_sat = ~MSB ? (|incline[11:9] ? (10'b0111111111) : {1'b0, incline[8:0]}) : (~(&incline[11:9]) ? (10'b1000000000) : {incline[9:0]});

	assign incline_factor = incline_sat + 9'd256;
	// assign incline_factor = ~incline_sat[9] ? {1'b0, (incline_sat[8:0] + 9'd256)} : incline_sat + 9'd256;
	// assign incline_lim = incline_factor[10] ? (9'd0) : ((incline_factor[9]) ? (9'd511) : (incline_factor[8:0]));
	assign incline_lim = incline_factor[9] ? (incline_factor[8] ? 9'd0 : 9'd511) : incline_factor[8:0];
	assign cadence_factor = (cadence_vec <= 5'b00001) ? (6'b000000) : (cadence_vec + 6'd32);
	assign torque_off = avg_torque - TORQUE_MIN;
	assign torque_pos = torque_off[12] ? (12'h000) : (torque_off[11:0]);
	assign assist_prod = a_1 * a_2;
	//assign target_curr = |assist_prod[28:26] ? (12'hFFF) : (assist_prod[25:14]);

	always @(posedge clk) begin
		a_1 <= torque_pos * setting;
		a_2 <= incline_lim * cadence_factor;
		target_curr <= |assist_prod[28:26] ? (12'hFFF) : (assist_prod[25:14]);
	end

endmodule
