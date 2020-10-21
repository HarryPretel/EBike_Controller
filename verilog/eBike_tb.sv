module eBike_tb();

  reg clk,RST_n;
  reg [11:0] BATT;				// analog values you apply to AnalogModel
  reg [11:0] BRAKE,TORQUE;		// analog values
  reg cadence;					// you have to have some way of applying a cadence signal
  reg tgglMd;	
  reg [15:0] YAW_RT;			// models angular rate of incline
  
  wire A2D_SS_n,A2D_MOSI,A2D_SCLK,A2D_MISO;		// A2D SPI interface
  wire highGrn,lowGrn,highYlw;					// FET control
  wire lowYlw,highBlu,lowBlu;					//   PWM signals
  wire hallGrn,hallBlu,hallYlw;					// hall sensor outputs
  wire inertSS_n,inertSCLK,inertMISO,inertMOSI,inertINT;	// Inert sensor SPI bus
  
  wire [1:0] setting;		// drive LEDs on real design
  wire [11:0] curr;			// comes from eBikePhysics back to AnalogModel
  
  //////////////////////////////////////////////////
  // Instantiate model of analog input circuitry //
  ////////////////////////////////////////////////
  AnalogModel iANLG(.clk(clk),.rst_n(RST_n),.SS_n(A2D_SS_n),.SCLK(A2D_SCLK),
                    .MISO(A2D_MISO),.MOSI(A2D_MOSI),.BATT(BATT),
		            .CURR(curr),.BRAKE(BRAKE),.TORQUE(TORQUE));

  ////////////////////////////////////////////////////////////////
  // Instantiate model inertial sensor used to measure incline //
  //////////////////////////////////////////////////////////////
  eBikePhysics iPHYS(.clk(clk),.RST_n(RST_n),.SS_n(inertSS_n),.SCLK(inertSCLK),
	             .MISO(inertMISO),.MOSI(inertMOSI),.INT(inertINT),
		     .yaw_rt(YAW_RT),.highGrn(highGrn),.lowGrn(lowGrn),
		     .highYlw(highYlw),.lowYlw(lowYlw),.highBlu(highBlu),
		     .lowBlu(lowBlu),.hallGrn(hallGrn),.hallYlw(hallYlw),
		     .hallBlu(hallBlu),.avg_curr(curr));

  //////////////////////
  // Instantiate DUT //
  ////////////////////
  eBike iDUT(.clk(clk),.RST_n(RST_n),.A2D_SS_n(A2D_SS_n),.A2D_MOSI(A2D_MOSI),
             .A2D_SCLK(A2D_SCLK),.A2D_MISO(A2D_MISO),.hallGrn(hallGrn),
			 .hallYlw(hallYlw),.hallBlu(hallBlu),.highGrn(highGrn),
			 .lowGrn(lowGrn),.highYlw(highYlw),.lowYlw(lowYlw),
			 .highBlu(highBlu),.lowBlu(lowBlu),.inertSS_n(inertSS_n),
			 .inertSCLK(inertSCLK),.inertMOSI(inertMOSI),
			 .inertMISO(inertMISO),.inertINT(inertINT),
			 .cadence(cadence),.tgglMd(tgglMd),.TX(TX),
			 .setting(setting));
	
  ///////////////////////////////////////////////////////////
  // Instantiate Something to monitor telemetry output??? //
  /////////////////////////////////////////////////////////
			 
	int error_limit = 50;
	int CADENCE_PWM = 2048;
	int test_number = 0;
	int prevOmega;
	int omega;
	int signed dOmega;
	int no_change_range = 500;
	int tests_passed = 0;
	int tests_failed = 0;
	int setting_count = 0;
	
initial begin
	clk = 0;
	RST_n = 0;
	tgglMd = 0;
	cadence = 1;
	
	TORQUE = 12'h000;
	BRAKE = 12'h802;
	BATT = 12'hB11;
	YAW_RT = 16'h0200;
	CADENCE_PWM = 2048;

	@(negedge clk);
	RST_n = 1;
	@(posedge clk);
	
	while(setting_count!=4) begin
		
		TORQUE = 12'hfff;
		run_test(1);
		
		
		TORQUE = 12'h0ff;
		run_test(-1);
	
	
		tgglMd = 0;
		run_test(1);
		tgglMd = 1;
		setting_count++;
	end

	end_simulation();

end
  
   
task run_test;
  input int expected_omega_change_direction;	//-1 for decrease; 1 for increase; 0 for stay the same (see no_change_range)
	begin
		prevOmega = iPHYS.omega;
		wait_error();
		omega = iPHYS.omega;
		case(expected_omega_change_direction)
			1: begin 
				if(omega > prevOmega + no_change_range) test_passed();
				else test_failed();
			end
			0: begin
				if(omega < prevOmega + no_change_range && omega > prevOmega - no_change_range) test_passed();
				else test_failed();
			end
			-1: begin
				if(omega < prevOmega - no_change_range) test_passed();
				else test_failed();
			end
		endcase
	end
  endtask
	
	task end_simulation;
		begin
			$display("%d tests passed. %d tests failed.", tests_passed, tests_failed);
			$display("TEST FINISHED.");
			$stop();
		end
	endtask
	
	task test_passed;
		begin
			dOmega = omega - prevOmega;
			$display("Test %d passed. omega = %d; dOmega = %d", test_number, omega, dOmega);
			tests_passed++;
		end
	endtask
	
	task test_failed;
		begin
			dOmega = omega - prevOmega;
			$display("Test %d FAILED. omega = %d; dOmega = %d", test_number, omega, dOmega);
			tests_failed++;
		end
	endtask
  
  task wait_error;
	begin
		test_number++;
		repeat(512) begin
			cadence = 1;
			repeat(CADENCE_PWM) @(posedge clk);
			cadence = 0;
			repeat(4096 - CADENCE_PWM) @(posedge clk);
		end
		while(iDUT.senseCndt.error > error_limit || iDUT.senseCndt.error < -error_limit) begin
			cadence = 1;
			repeat(CADENCE_PWM) @(posedge clk);
			cadence = 0;
			repeat(4096 - CADENCE_PWM) @(posedge clk);
		end
	end
  endtask
  
  always 
    #5 clk = ~clk;
  

	
endmodule
