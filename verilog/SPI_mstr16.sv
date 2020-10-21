module SPI_mstr16(clk, rst_n, SS_n, SCLK, MOSI, MISO, wrt, cmd, done, rd_data);

//inputs and outputs
input clk, rst_n, MISO, wrt;
input reg [15:0] cmd;
output reg SS_n, SCLK, MOSI, done;
output reg [15:0] rd_data;

//intermediates
logic done15, ld_SCLK, init, smpl, shft, set_done;
logic [5:0] sclk_div;
logic [15:0] shft_reg; 
logic [3:0] bit_cntr;
logic MISO_smpl;

//---------------------------State Machine logic
//typedef for state machine
typedef enum reg [2:0] {IDLE, FPORCH, SAMPLE_SHIFT, BPORCH} state_t;
state_t state, next_state;

//setup for statemachine reset
always_ff @(posedge clk, negedge rst_n)
	 if (!rst_n)
	 	state <= IDLE;
	else
		state <= next_state;
//state machine logic		
always_comb begin
	//initial default values
	next_state = state;
	set_done = 0;
	smpl = 0;
	shft = 0;
	ld_SCLK = 0;
	init = 0;
	//cases:
	case(state)
		IDLE:
		begin
			if(wrt) begin //if writing
				ld_SCLK = 1; //select on a mux that sets counter
				init = 1; //sets the starting signal to low (active low)
				next_state = FPORCH; //sets next state to front porch
			end
		end
		FPORCH:
		begin
			if(&sclk_div) begin //if sampling
				next_state = SAMPLE_SHIFT; //go to sample/shift state
			end
		end
		SAMPLE_SHIFT:
		begin
			if((bit_cntr==4'b1111)&&(sclk_div==6'b111000)) begin//if done sampling
				next_state = BPORCH;
				ld_SCLK = 1;
				shft = 1;
			end
			else if(&sclk_div)
				shft = 1;
			else if(sclk_div == 6'b011111)
				smpl = 1;	
		end		
		BPORCH:
		begin
			set_done = 1; //set done
			//shft = 1;
			next_state = IDLE; //return to IDLE state
			if(sclk_div == 6'b111111)begin //used to be wrt
				//smpl = 1
				//shft = 1;
				
				//next_state = SAMPLE_SHIFT;
				init = 1;
				
			end
			else
				set_done = 1;
		end
	default: next_state = IDLE;
	endcase
end



//----------------------------Transceiver logic


//setup SCLK counter
always@(posedge clk) begin
if(ld_SCLK)
	sclk_div <= 6'b110000; //reset to this val
else
	sclk_div <= sclk_div + 1; //start counting
end

//set up shift counter
always@(posedge clk) begin
if(init)
	bit_cntr <= 4'b0000;
else if(shft)
		bit_cntr <= bit_cntr + 1;
end

assign done15 = &bit_cntr;

assign SCLK = sclk_div[5]; //SCLK takes MSB
//Decode various sclk_div states
assign rd_data = shft_reg;

//set up shifter
always@(posedge clk) begin
	case({init, shft})
			2'b11: shft_reg <= cmd;
			2'b10: shft_reg <= cmd;
			2'b01: shft_reg <= {shft_reg[14:0], MISO_smpl};
			2'b11: shft_reg <= shft_reg;
	endcase
end

assign MOSI = shft_reg[15]; //set MOSI to most sig bit of shft_reg

//setup MISO
always@(posedge clk) begin
if(smpl)
	MISO_smpl <= MISO;
end

always@(posedge clk, negedge rst_n) begin
	if( ~rst_n)begin
		done <= 1'b0;
	end
	else if(set_done) begin
		done <= 1'b1;
	end
	else if(init) begin
		done <= 1'b0;
	end
	
end

always@(posedge clk, negedge rst_n) begin
	if(~rst_n) begin
		SS_n <= 1'b1;
	end
	else if(set_done)  begin
		SS_n <= 1'b1;
	end
	else if(init) begin
		SS_n <= 1'b0;
	end
end




endmodule

