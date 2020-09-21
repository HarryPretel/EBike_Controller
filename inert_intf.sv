module inert_intf(clk, rst_n, MISO, INT, SS_n, SCLK, MOSI, vld, incline);

//inputs
input clk, rst_n;
input MISO;
input INT;
//outputs:
output SS_n; //from SPI
output SCLK;
output MOSI;
output reg vld; //from SM
output[12:0] incline; //from inertial integrator

//intermediates:
//logic C_R_H, C_R_L, C_Y_H, C_Y_L, C_AY_H, C_AY_L, C_AZ_H, C_AZ_L;
logic wrt, done;
logic[15:0] cmd, roll_rt, yaw_rt, AY, AZ;
logic[7:0] rd_data;
logic hold_en;
logic [2:0]sm_2;
logic sm_2_inc;
logic sm_2_rst_n;

//instantiations
inertial_integrator iInteg(.clk(clk), .rst_n(rst_n), .vld(vld), .roll_rt(roll_rt), .yaw_rt(yaw_rt) ,. AY(AY), .AZ(AZ) , .incline(incline));
SPI_mstr iSpy(.clk(clk), .rst_n(rst_n), .SS_n(SS_n), .SCLK(SCLK), .MOSI(MOSI), .MISO(MISO), .wrt(wrt), .cmd(cmd), .done(done), .rd_data(rd_data));

//16 bit timer
logic [15:0]timer;
always@(posedge clk, negedge rst_n) begin
	if(!rst_n)
		timer <= 16'h0000;
	else
		timer <= timer + 1;
end

//double flop INT
logic INT_f, INT_ff;
always@(posedge clk, negedge rst_n) begin
	if(!rst_n) begin
		INT_f <= 0;
		INT_ff <= 0;
	end
	else begin
		INT_f <= INT;
		INT_ff <= INT_f; //Use INT_ff in the future
	end
end


// 4 16-bit flops to store the 8b rollL, rollH, yawl, yawH, AYL, AYH, AZL and AZH. 
always@(posedge clk, negedge rst_n) begin
	if(!rst_n) begin
		roll_rt <= 16'h0000;
		yaw_rt <= 16'h0000;
		AY <= 16'h0000;
		AZ <= 16'h0000;
	end
	else if(hold_en) begin
		case(sm_2)
			3'h0: roll_rt[7:0] <= rd_data;
			3'h1: roll_rt[15:8] <= rd_data;
			3'h2: yaw_rt[7:0] <= rd_data;
			3'h3: yaw_rt[15:8]  <= rd_data;
			3'h4: AY[7:0] <= rd_data;
			3'h5: AY[15:8] <= rd_data;
			3'h6: AZ[7:0] <= rd_data;
			3'h7: AZ[15:8] <= rd_data;
		endcase
	end
end

logic init_en;
always_comb begin
	cmd = 16'h0000;
	if(init_en) begin
		case(sm_2)
			3'h0: cmd = 16'h0d02;
			3'h1: cmd = 16'h1053;
			3'h2: cmd = 16'h1150;
			3'h3: cmd = 16'h1460;
		endcase
	end
	else begin
		case(sm_2)
			3'h0: cmd = 16'hA4xx;
			3'h1: cmd = 16'hA5xx;
			3'h2: cmd = 16'hA6xx;
			3'h3: cmd = 16'hA7xx;
			3'h4: cmd = 16'hAAxx;
			3'h5: cmd = 16'hABxx;
			3'h6: cmd = 16'hACxx;
			3'h7: cmd = 16'hADxx;
		endcase
	end
end



always@(posedge clk, negedge rst_n) begin
	if(!rst_n) sm_2 <= 3'h0;
	else if(!sm_2_rst_n) sm_2 <= 3'h0;
	else if(sm_2_inc) sm_2 <= sm_2 + 1'b1;
end



typedef enum reg [2:0] {RST, INIT, INITWRITE, WAIT,WRITE, READ} state_t;
state_t state, nxt_state;

 //setup for statemachine reset
always_ff @(posedge clk, negedge rst_n) begin
	 if (!rst_n)
	 	state <= RST;
	else
		state <= nxt_state;
end

always_comb begin
	//initial default values
	nxt_state = state;
	wrt = 0;
	sm_2_rst_n = 1;
	sm_2_inc = 0;
	hold_en = 0;
	vld = 0;
	init_en = 0;
	
	//cases:
	case(state)
		RST: begin
			if(&timer) begin
				nxt_state = INITWRITE;
			end
		end
		INITWRITE: begin
			init_en = 1;
			wrt = 1;
			nxt_state = INIT;
		end
		INIT: begin
			init_en = 1;
			
			if(sm_2==3'h3&&done) begin
				sm_2_rst_n = 0;
				nxt_state = WAIT;
			end
			else if(done) begin
				sm_2_inc = 1'b1; //this takes one clk cycle to actually inc so we need to delay wrt = 1. Added extra state.
				nxt_state = INITWRITE;
			end
		end
		WAIT: begin
			if(INT_ff) begin
				nxt_state = WRITE;
			end
		end
		WRITE: begin
			wrt = 1;
			nxt_state = READ;
		end
		READ: begin
			
			if(done&&(sm_2!=3'h7)) begin 
				hold_en = 1;
				nxt_state = WRITE;
				sm_2_inc = 1;
			end
			else if(done&&(sm_2==3'h7)) begin 
				hold_en = 1;
				sm_2_inc = 1;
				vld = 1;
				nxt_state = WAIT;
			end
		end
		default: nxt_state = RST;
	endcase
end

//at reset: wait for timer, then send the 4 commands from the table



endmodule

