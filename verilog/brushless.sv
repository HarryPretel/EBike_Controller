module brushless(clk, drv_mag, hallGrn, hallBlu, hallYlw, brake_n, duty, selGrn, selYlw, selBlu);
//INPUTS/OUTPUTS
input logic clk;
input logic [11:0] drv_mag;
input logic hallGrn, hallYlw, hallBlu, brake_n;
output logic [10:0] duty;
output logic [1:0] selGrn, selYlw, selBlu;
//intermetiate
reg [2:0] rotation_state;

//define local params for readability
localparam HI_Z = 2'b00;
localparam R_CURR = 2'b01;
localparam F_CURR = 2'b10;
localparam R_BRK = 2'b11;

always@(posedge clk) begin
	rotation_state <= {hallGrn, hallYlw, hallBlu}; //rotation_state is concatination of hall sensor signals
end
//STATE MACHINE
always_comb begin
	if(!brake_n)begin //if breaking, set each select to break
		selGrn = R_BRK;
		selYlw = R_BRK;
		selBlu = R_BRK;
		end
	else begin
		case(rotation_state) //set each select signal to correct value for that state:
			3'b001: begin
				selGrn = HI_Z;
				selYlw = R_CURR;
				selBlu = F_CURR;
				end
			3'b011: begin
				selGrn = R_CURR;
				selYlw = HI_Z;
				selBlu = F_CURR;
				end
			3'b010: begin
				selGrn = R_CURR;
				selYlw = F_CURR;
				selBlu = HI_Z;
				end
			3'b110: begin
				selGrn = HI_Z;
				selYlw = F_CURR;
				selBlu = R_CURR;
				end
			3'b100: begin
				selGrn = F_CURR;
				selYlw = HI_Z;
				selBlu = R_CURR;
				end
			3'b101: begin
				selGrn = F_CURR;
				selYlw = R_CURR;
				selBlu = HI_Z;
				end
			default: begin //default each to be disconnected (HI_Z)
				selGrn = HI_Z;
				selYlw =HI_Z;
				selBlu = HI_Z;
				end
		endcase
	end 
end

assign duty = brake_n ? (11'h400+drv_mag[11:2]) : (11'h600); //make duty the first if not breaking, the second if it is.


endmodule
