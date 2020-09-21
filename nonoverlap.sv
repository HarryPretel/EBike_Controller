module nonoverlap(clk, rst_n,highIn, lowIn,highOut, lowOut);
	input clk, rst_n, highIn, lowIn;
	output reg highOut, lowOut;
	reg highIn_F, lowIn_F;
	logic changed, clrCnt,cnt_full, outputsLow;
	reg [4:0] counter = 5'b0;
	
//pass signals thru flops to sync with clock
always @(posedge clk) begin
		highIn_F <= highIn;
end

always @(posedge clk) begin
		lowIn_F <= lowIn;
end

//5 bit counter
always @(posedge clk) begin
	if (clrCnt) //clear to be called by SM
		counter <= 5'b0;
	else if (&counter) //if counter is full
		cnt_full = 1; //set cnt_full
	else
		counter <= counter + 1; //inc counter
end		

//create state machine
typedef enum logic{IDLE, LOW} state_t;
state_t currState, nxt_state;

//SM synch RESET
always @(posedge clk, negedge rst_n) begin
    if (!rst_n)
		currState <= IDLE;
	else 
		currState <= nxt_state;
		
end

always @(posedge clk, negedge rst_n) begin
	if(!rst_n)
		highOut <= 1'b0;
	else if(outputsLow)
		highOut <= 1'b0;
	else 
		highOut <= highIn;
	end

always @(posedge clk, negedge rst_n) begin
	if(!rst_n)
		lowOut <= 1'b0;
	else if(outputsLow)
		lowOut <= 1'b0;
	else 
		lowOut <= lowIn;
	end
//STATE MACHINE
always_comb begin
	//initial vals:
	outputsLow = 0;
	clrCnt = 0;
	nxt_state = currState;
	case(currState) 
		IDLE: if((highIn ^ highIn_F) || (lowIn ^ lowIn_F))  //if there is a change
				nxt_state = LOW; //go to low state
			else 
				clrCnt = 1;
		LOW: if(~cnt_full) //if there is still time
				outputsLow = 1; //set outputs to low
			else if((highIn ^ highIn_F) || (lowIn ^ lowIn_F)) begin //if there was a change
				outputsLow = 1; //set outputs to low
				clrCnt = 1; //clear count
				end
			else if (cnt_full) begin //if counter filled
				nxt_state = IDLE; //go to IDLE
				clrCnt = 1; //clear counter
			end
		default:
			nxt_state = IDLE;
		endcase
		
	end
	endmodule	
	

