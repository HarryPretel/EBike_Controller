module A2D_intf(clk, rst_n, batt, curr, brake, torque, SS_n, SCLK, MOSI, MISO);

input clk, rst_n;
output reg [11:0] batt;
output reg [11:0] curr;
output reg [11:0] brake;
output reg [11:0] torque;
output SS_n;
output SCLK;
output MOSI;
input MISO;

logic wrt;
logic done;
logic [15:0] rd_data;

logic [13:0] counter;

logic torque_en;
logic curr_en;
logic batt_en;
logic brake_en;

logic cnv_cmplt;
logic [1:0] round_robin;
logic [2:0] a2d_channel;

logic [15:0] cmd;


//SPI_mstr
SPI_mstr iSPI(.clk(clk), .rst_n(rst_n), .SS_n(SS_n), .SCLK(SCLK), .MOSI(MOSI), .MISO(MISO), .wrt(wrt), .cmd(cmd), .done(done), .rd_data(rd_data));

//14b free-running counter
always@(posedge clk, negedge rst_n) begin
	if(!rst_n) counter <= 1'h0;
	else counter <= counter + 1'h1;
end

//2b round-robin channel counter
always@(posedge clk, negedge rst_n) begin
	if(!rst_n) round_robin <= 1'h0;
	else if(cnv_cmplt) begin
		round_robin <= round_robin + 1'h1;
	end
end
assign a2d_channel = round_robin[1] ? 1'b1 + round_robin : round_robin;
assign cmd = {2'b00, a2d_channel, 11'h000};

//targeted enabler
always_comb begin
	torque_en = 1'h0;
	curr_en = 1'h0;
	batt_en = 1'h0;
	brake_en = 1'h0;
	if(cnv_cmplt) begin
		case(a2d_channel)
			3'b000: batt_en = 1'h1;
			3'b001: curr_en = 1'h1;
			3'b011: brake_en = 1'h1;
			3'b100: torque_en = 1'h1;
		endcase
	end
end

//state machine
typedef enum reg [1:0] {IDLE, REQUEST, PAUSE, RECEIVE } state_t;
state_t state, nxt_state;

always@(posedge clk, negedge rst_n) begin
	if(!rst_n) state <= IDLE;
	else state <= nxt_state;
end

always_comb begin
	nxt_state = state;
	cnv_cmplt = 1'h0;
	wrt = 1'h0;
	case(state)
		IDLE: begin
			if(&counter) begin
				nxt_state = REQUEST;
				wrt = 1'h1;
			end
		end
		REQUEST: begin
				//wrt = 1;
			if(done) begin
				nxt_state = PAUSE;
			end
		end
		PAUSE: begin
			nxt_state = RECEIVE;
			wrt = 1'h1;
		end
		RECEIVE: begin
			//wrt = 1;
			if(done) begin
				cnv_cmplt = 1'h1;
				nxt_state = IDLE;
			end
		end
		default: nxt_state = IDLE;
	endcase
end

//batt FF
always@(posedge clk, negedge rst_n) begin
	if(!rst_n) batt <= 1'h0;
	else if(batt_en) batt <= rd_data[11:0];
end

//curr FF
always@(posedge clk, negedge rst_n) begin
	if(!rst_n) curr <= 1'h0;
	else if(curr_en) curr <= rd_data[11:0];
end

//brake FF
always@(posedge clk, negedge rst_n) begin
	if(!rst_n) brake <= 1'h0;
	else if(brake_en) brake <= rd_data[11:0];
end

//torque FF
always@(posedge clk, negedge rst_n) begin
	if(!rst_n) torque <= 1'h0;
	else if(torque_en) torque <= rd_data[11:0];
end


endmodule