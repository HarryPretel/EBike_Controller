module inert_intf_test(clk,RST_n,SS_n,SCLK,MOSI,MISO,INT,LED);

  input clk,RST_n;
  input INT;				// indicates new reading ready from inertial sensor
  input MISO;				// data from inertial sensor
  output SS_n,SCLK,MOSI;	// SPI interface
  output reg [7:0] LED;
  
  wire [12:0] incline;
  wire rst_n;


  /////////////////////////////////////
  // Instantiate reset synchronizer //
  ///////////////////////////////////
  rst_synch iRST(.clk(clk),.RST_n(RST_n),.rst_n(rst_n));


  /////////////////////////////////////
  // Instantiate inertial interface //
  ///////////////////////////////////
  inert_intf iNTF(.clk(clk),.rst_n(rst_n),.incline(incline),.vld(vld),
             .SS_n(SS_n),.SCLK(SCLK),.MOSI(MOSI),.MISO(MISO),.INT(INT));
			 
  always @(posedge clk, negedge rst_n)
    if (!rst_n)
	  LED <= 8'h00;
	else if (vld)
	  LED <= incline[8:1];
	  
endmodule
	  