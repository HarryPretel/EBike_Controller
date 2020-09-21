module intertial_intf_tb();

logic RST_n, clk, MISO, INT, SS_n, SCLK, MOSI;
logic [7:0] LED;

inert_intf_test iDUT(clk,RST_n,SS_n,SCLK,MOSI,MISO,INT,LED);

initial begin
clk = 1;
RST_n = 1;
INT = 0;
MISO = 1;@(posedge clk);
RST_n = 0;
repeat(2)@(posedge clk);
RST_n = 1;
repeat(70000)@(posedge clk);
INT = 1;
repeat(10)@(posedge clk);
INT = 0;
repeat(70000)@(posedge clk);
INT = 1;
repeat(10)@(posedge clk);
INT = 0;
repeat(70000)@(posedge clk);
INT = 1;
repeat(10)@(posedge clk);
INT = 0;
repeat(20000)@(posedge clk);
INT = 1;
repeat(65536)@(posedge clk);
$stop();

end

always #5 clk = ~clk;

endmodule
