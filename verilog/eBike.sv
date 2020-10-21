 module eBike(clk,RST_n,A2D_SS_n,A2D_MOSI,A2D_SCLK,
             A2D_MISO,hallGrn,hallYlw,hallBlu,highGrn,
			 lowGrn,highYlw,lowYlw,highBlu,lowBlu,
			 inertSS_n,inertSCLK,inertMOSI,inertMISO,
			 inertINT,cadence,TX,tgglMd,setting);
			 
  input clk;				// 50MHz clk
  input RST_n;				// active low RST_n from push button
  output A2D_SS_n;			// Slave select to A2D on DE0
  output A2D_SCLK;			// SPI clock to A2D on DE0
  output A2D_MOSI;			// serial output to A2D (what channel to read)
  input A2D_MISO;			// serial input from A2D
  input hallGrn;			// hall position input for "Green" phase
  input hallYlw;			// hall position input for "Yellow" phase
  input hallBlu;			// hall position input for "Blue" phase
  output highGrn;			// high side gate drive for "Green" phase
  output lowGrn;			// low side gate drive for "Green" phase
  output highYlw;			// high side gate drive for "Yellow" phase
  output lowYlw;			// low side gate drive for "Yellow" phas
  output highBlu;			// high side gate drive for "Blue" phase
  output lowBlu;			// low side gate drive for "Blue" phase
  output inertSS_n;			// Slave select to inertial (tilt) sensor
  output inertSCLK;			// SCLK signal to inertial (tilt) sensor
  output inertMOSI;			// Serial out to inertial (tilt) sensor  
  input inertMISO;			// Serial in from inertial (tilt) sensor
  input inertINT;			// Alerts when inertial sensor has new reading
  input cadence;			// pulse input from pedal cadence sensor
  input tgglMd;				// used to select setting[1:0] (from PB switch)
  output reg [1:0] setting;	// 11 => easy, 10 => normal, 01 => hard, 00 => off
  output TX;				// serial output of measured batt,curr,torque
  
  ///////////////////////////////////////////////
  // Declare any needed internal signals here // We still need to do this
  /////////////////////////////////////////////
	wire rst_n;									// global reset from reset_synch
	logic [11:0] brake, torque, curr, batt_v;
	logic brake_n;
	logic not_pedaling;
	logic [12:0] error, incline;
	logic [11:0] drv_mag;
	logic [10:0] duty;
	logic [1:0] selGrn, selYlw, selBlu;
  
  
  ///////// Any needed macros follow /////////
  localparam FAST_SIM = 1;
  
  /////////////////////////////////////
  // Instantiate reset synchronizer //
  ///////////////////////////////////
	rst_synch ResetSynchronizer(.RST_n(RST_n), .clk(clk), .rst_n(rst_n));
  
  ///////////////////////////////////////////////////////
  // Instantiate A2D_intf to read torque & batt level // Jacob and Harry
  /////////////////////////////////////////////////////
	A2D_intf iA2D(.clk(clk), .rst_n(rst_n), .batt(batt_v), .curr(curr), .brake(brake), .torque(torque), .SS_n(A2D_SS_n), .SCLK(A2D_SCLK), .MOSI(A2D_MOSI), .MISO(A2D_MISO));
 
  ////////////////////////////////////////////////////////////
  // Instantiate SensorCondition block to filter & average //
  // readings and provide cadence_vec, and zero_cadence   // Roshan and Connor
  // Don't forget to pass FAST_SIM parameter!!           //
  ////////////////////////////////////////////////////////
	SensorCondition #(FAST_SIM) senseCndt(.clk(clk), .rst_n(rst_n), .torque(torque), .cadence(cadence), .curr(curr), .incline(incline), .setting(setting), .batt(batt_v), .error(error), .not_pedaling(not_pedaling), .TX(TX));

  ///////////////////////////////////////////////////
  // Instantiate PID to determine drive magnitude //
  // Don't forget to pass FAST_SIM parameter!!   //  Connor and Roshan
  ////////////////////////////////////////////////	
	PID #(FAST_SIM) iPID(.clk(clk), .rst_n(rst_n), .error(error), .not_pedaling(not_pedaling), .drv_mag(drv_mag));
  
  ////////////////////////////////////////////////
  // Instantiate brushless DC motor controller //
  //////////////////////////////////////////////
	brushless iBrush(.clk(clk), .drv_mag(drv_mag), .hallGrn(hallGrn), .hallYlw(hallYlw), .hallBlu(hallBlu), .brake_n(brake_n), .duty(duty), .selGrn(selGrn), .selYlw(selYlw), .selBlu(selBlu));

  ///////////////////////////////
  // Instantiate motor driver //
  /////////////////////////////
	mtr_drv iMtr(.clk(clk), .rst_n(rst_n), .selGrn(selGrn), .selYlw(selYlw), .selBlu(selBlu), .duty(duty), .highGrn(highGrn), .lowGrn(lowGrn), .highYlw(highYlw), .lowYlw(lowYlw), .highBlu(highBlu), .lowBlu(lowBlu));

  /////////////////////////////////////////////////////////////
  // Instantiate inertial sensor to measure incline (pitch) // Harry and Jacob
  ///////////////////////////////////////////////////////////
	inert_intf iNTF(.clk(clk), .rst_n(rst_n), .MISO(inertMISO), .INT(inertINT), .SS_n(inertSS_n), .SCLK(inertSCLK), .MOSI(inertMOSI), .vld(), .incline(incline));
		
  ////////////////////////////////////////////////////////
  // Instantiate (or infer) tggleMd/setting[1:0] logic // Never Tested
  //////////////////////////////////////////////////////
	pushButton iPush(.tgglMd(tgglMd), .setting(setting), .rst_n(rst_n), .clk(clk));
  
  ///////////////////////////////////////////////////////////////////////
  // brake_n should be asserted if brake A2D reading lower than 0x800 // Connor and Roshan
  /////////////////////////////////////////////////////////////////////
	assign brake_n = (brake < 12'h800) ? 0 : 1;

endmodule
