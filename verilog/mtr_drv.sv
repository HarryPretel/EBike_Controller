// Jacobs Motor Drive
module mtr_drv(clk, rst_n, duty, selGrn, selYlw, selBlu, highGrn, lowGrn, highYlw, lowYlw, highBlu, lowBlu);

input clk, rst_n;
input logic[1:0] selGrn, selYlw, selBlu;
input logic [10:0] duty;
output logic highGrn, lowGrn, highYlw, lowYlw, highBlu, lowBlu;
 
//intermediates: 
logic PWM_sig;
logic HG_temp, LG_temp, HY_temp, LY_temp, HB_temp, LB_temp;
//instaniate PWM11
PWM11 iDUT_PWM(.clk(clk), .rst_n(rst_n), .duty(duty), .PWM_sig(PWM_sig) );

//each assign statement represents a 4:1 mux. Correctly passes thru desired val depending on 2 bit select.

//GREEN:
assign HG_temp = ^selGrn ? (selGrn[0] ? ~PWM_sig : PWM_sig ): 1'b0;
assign LG_temp = selGrn[0] ? PWM_sig : (selGrn[1] ? ~PWM_sig : 1'b0  );

//YELLOW
assign HY_temp = ^selYlw ? (selYlw[0] ? ~PWM_sig : PWM_sig ): 1'b0;
assign LY_temp = selYlw[0] ? PWM_sig : (selYlw[1] ? ~PWM_sig : 1'b0  );

//BLUE
assign HB_temp = ^selBlu ? (selBlu[0] ? ~PWM_sig : PWM_sig ): 1'b0;
assign LB_temp = selBlu[0] ? PWM_sig : (selBlu[1] ? ~PWM_sig : 1'b0  );

//instantiate non_overlap: makes sure no overlap between high and low outputs. One for each signal.
nonoverlap iDUT_noG(.clk(clk), .rst_n(rst_n), .highIn(HG_temp), .lowIn(LG_temp), .highOut(highGrn), .lowOut(lowGrn));
nonoverlap iDUT_noY(.clk(clk), .rst_n(rst_n), .highIn(HY_temp), .lowIn(LY_temp), .highOut(highYlw), .lowOut(lowYlw));
nonoverlap iDUT_noB(.clk(clk), .rst_n(rst_n), .highIn(HB_temp), .lowIn(LB_temp), .highOut(highBlu), .lowOut(lowBlu));
endmodule
