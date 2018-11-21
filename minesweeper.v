`timescale 1ns/1ns

module minesweeper
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
		KEY,							// On Board Keys
		SW,
		HEX0,
		HEX2,
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]
		PS2_CLK,
		PS2_DAT
	);

	input			CLOCK_50;				//	50 MHz
	input	[3:0]	KEY;
	input [9:0] SW;
	
	output [6:0] HEX0, HEX2;
	
	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[7:0]	VGA_R;   				//	VGA Red[7:0] Changed from 10 to 8-bit DAC
	output	[7:0]	VGA_G;	 				//	VGA Green[7:0]
	output	[7:0]	VGA_B;   				//	VGA Blue[7:0]
	
	wire resetn;
	assign resetn = KEY[0];
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.

	wire [5:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;
	wire [5:0] Position;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			// Signals for the DAC to drive the monitor.
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 2;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn
	// for the VGA controller, in addition to any other functionality your design may require.
	
		//PS2 Code
	/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/

// Inputs



// Bidirectionals
inout				PS2_CLK;
inout				PS2_DAT;

// Outputs
//HEXs were here

/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/

// Internal Wires
wire		[7:0]	ps2_key_data;
wire				ps2_key_pressed;


// Internal Registers
reg			[7:0]	last_data_received;
wire [7:0] data_out;
reg [7:0] data_out_reg;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                             *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential Logic                              *
 *****************************************************************************/

always @(posedge CLOCK_50)
begin
//	if (KEY[0] == 1'b0)
//		last_data_received <= 8'h00;
	if (ps2_key_pressed == 1'b1)
	  last_data_received <= ps2_key_data;
	  data_out_reg <= data_out;
	
end



/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/


/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

PS2_Controller PS2 (
	// Inputs
	.CLOCK_50				(CLOCK_50),
	.reset				(~KEY[0]),

	// Bidirectionals
	.PS2_CLK			(PS2_CLK),
 	.PS2_DAT			(PS2_DAT),

	// Outputs
	.received_data		(ps2_key_data),
	.received_data_en	(ps2_key_pressed)
);
	
	wire [4:0] d;
	reg [4:0] draw; //0 - down, 1 - up, 2 - left, 3 - right, 4 - escape
	

	always @(posedge CLOCK_50) 
	 begin
	   if(data_out_reg == 8'h72)
		//trigger down
		 draw[0] <= 1'b1;
  
		else if(data_out_reg  == 8'h75)
			//trigger up
		  draw[1] <= 1'b1;
  
		else if(data_out_reg  == 8'h6b)
		  //trigger left
		 draw[2] <= 1'b1;
  
		else if(data_out_reg  == 8'h74)
		 //trigger right
		 draw[3] <= 1'b1;
  
		else if(data_out_reg  == 8'h76)
		  //trigger reset (ESC key)
		 draw[4] <= 1'b1;

		else
		 draw <= 5'b00000;
		 
     end
	 
	 assign d = draw;
	 assign data_out = ps2_key_pressed?ps2_key_data:8'h00;
	
	
	grid_select G0( 
			.clock(CLOCK_50), 
			.resetn(resetn), 
			.up(d[1]), 
		 	.down(d[0]), 
			.left(d[2]), 
			.right(d[3]), 
			.clear(d[4]),
			.click(~KEY[1]),
			
			.Position(Position),
			
			.Xout(x), 
			.Yout(y),
			.ColourOut(colour), 
			.writeEn(writeEn)
	);
	
	hex_decoder H0( .hex_digits({1'b0, Position[2:0]}), .segments(HEX0) );
	hex_decoder H2( .hex_digits({1'b0, Position[5:3]}), .segments(HEX2) );
	
endmodule

module grid_select
	(
		input clock, resetn,
		
		input up, down, left, right, clear, click,
		
		output [7:0] Xout,
		output [6:0] Yout,
		output [5:0] ColourOut,
		output writeEn,
		output [5:0] Position
	);
	
	wire en_x, en_y, op_x, op_y;
	wire plot_en, erase_en, clear_en, ld_clr;
	wire countPul, grid_pulse, framePulse;
	
	wire mine_reset, mine_write, draw_mine, draw_click;
	wire is_mine, end_pulse, reset_pulse;

	control C0 (
			.clock(clock),
			.resetn(resetn),
			
			.up(up),
			.down(down),
			.left(left),
			.right(right),
			.clear(clear),
			.click(click),
			
			.countPul(countPul),
			.grid_pulse(grid_pulse),
			.framePulse(framePulse),
			.reset_pulse(reset_pulse),
			
			.en_x(en_x),
			.en_y(en_y),
			.op_x(op_x),
			.op_y(op_y),
			
			.is_mine(is_mine),
			.mine_reset(mine_reset),
			.draw_mine(draw_mine),
			.draw_click(draw_click),
			.end_pulse(end_pulse),
			.mine_write(mine_write),
			
			.plot_en(plot_en),
			.erase_en(erase_en),
			.clear_en(clear_en),
			.ld_clr(ld_clr)
	);
	
	datapath D0 (
			.clock(clock),
			.resetn(resetn),
			
			.countPul(countPul),
			.grid_pulse(grid_pulse), 
			.framePulse(framePulse),
			
			.en_x(en_x),
			.en_y(en_y),
			.op_x(op_x),
			.op_y(op_y),
			
			.plot_en(plot_en),
			.erase_en(erase_en),
			.clear_en(clear_en),
			.ld_clr(ld_clr),
			
			.Position(Position),
			
			.draw_mine(draw_mine),
			.draw_click(draw_click),
			
			.writeEn(writeEn),
			
			.Xout(Xout),
			.Yout(Yout),
			.ColourOut(ColourOut)
	);	
	
	mine_datapath D1 (
			.clock(clock),
			.resetn(resetn),
			
			.mine_reset(mine_reset),
			.en_write(mine_write),
			.en_shift(en_shift),
			
			.position(Position),
			
			.is_mine(is_mine),
			.end_pulse(end_pulse),
			.reset_pulse(reset_pulse)
	);
	
endmodule


// control for grid_select
module control
	(
		input clock, resetn,
		
		input countPul, grid_pulse, framePulse, end_pulse, reset_pulse,
		input up, down, left, right, clear, click,
		input is_mine,
		
		output reg en_x, en_y,
		output reg op_x, op_y,
		output reg plot_en, clear_en, ld_clr, mine_reset, mine_write,
		output reg erase_en, draw_mine, draw_click, en_shift
	);
	
	reg firstn; // first move indicator
	reg [5:0] current_state, next_state;
	
	localparam 	S_ERASE_GRID 	= 6'd0,
					S_INCR_GRID		= 6'd1,
					S_CLEAR_WAIT	= 6'd2,
					S_WAIT			= 6'd3,
					S_ERASE_UP		= 6'd4,
					S_ERASE_DOWN	= 6'd5,
					S_ERASE_LEFT	= 6'd6,
					S_ERASE_RIGHT	= 6'd7,
					S_INCR_UP		= 6'd8,
					S_INCR_DOWN		= 6'd9,
					S_INCR_LEFT		= 6'd10,
					S_INCR_RIGHT	= 6'd11,
					S_DRAW			= 6'd12,
					S_DRAW_WAIT		= 6'd13,
					S_FRAME_WAIT	= 6'd14,
					S_GENERATE		= 6'd15,
					S_DRAW_CLICK_R	= 6'd16,
					S_DRAW_MINE_R	= 6'd17,
					S_INCR_PLOT_FAIL	= 6'd18,
					S_FAIL_WAIT		= 6'd19;
					
	// State Table
	always@(*)
	begin: state_table
		case(current_state)
			S_ERASE_GRID: next_state = countPul ? S_INCR_GRID : S_ERASE_GRID;
			S_INCR_GRID: next_state = grid_pulse ? S_CLEAR_WAIT : S_ERASE_GRID;
			S_CLEAR_WAIT: next_state = reset_pulse ? S_DRAW_WAIT : S_CLEAR_WAIT;
			S_WAIT:
				begin
					if(up == 1'b1)
						next_state = S_ERASE_UP;
					else if(down == 1'b1)
						next_state = S_ERASE_DOWN;
					else if(left == 1'b1)
						next_state = S_ERASE_LEFT;
					else if(right == 1'b1)
						next_state = S_ERASE_RIGHT;
					else if(clear == 1'b1)
						next_state = S_ERASE_GRID;
					else if(click == 1'b1 && firstn == 1'b0)
						next_state = S_GENERATE;
					else if(click == 1'b1 && firstn == 1'b1 && is_mine == 1'b0)
						next_state = S_DRAW_CLICK_R;
					else if(click == 1'b1 && firstn == 1'b1 && is_mine == 1'b1)
						next_state = S_DRAW_MINE_R;
					else
						next_state = S_WAIT;
				end
			S_ERASE_UP: next_state = countPul ? S_INCR_UP : S_ERASE_UP;
			S_ERASE_DOWN: next_state = countPul ? S_INCR_DOWN : S_ERASE_DOWN;
			S_ERASE_LEFT: next_state = countPul ? S_INCR_LEFT : S_ERASE_LEFT;
			S_ERASE_RIGHT: next_state = countPul ? S_INCR_RIGHT : S_ERASE_RIGHT;
			S_INCR_UP: next_state = S_DRAW;
			S_INCR_DOWN: next_state = S_DRAW;
			S_INCR_LEFT: next_state = S_DRAW;
			S_INCR_RIGHT: next_state = S_DRAW;
			S_DRAW: next_state = countPul ? S_DRAW_WAIT : S_DRAW;
			S_DRAW_WAIT: next_state = ( up == 1'b0 && down == 1'b0 && left == 1'b0 && right == 1'b0 && click == 1'b0 && clear == 1'b0) ? S_FRAME_WAIT : S_DRAW_WAIT;
			S_FRAME_WAIT: next_state = framePulse ? S_WAIT : S_FRAME_WAIT;
			S_GENERATE: next_state = end_pulse ? S_DRAW_CLICK_R : S_GENERATE;
			S_DRAW_CLICK_R: next_state = countPul ? S_DRAW_WAIT : S_DRAW_CLICK_R;
			S_DRAW_MINE_R: next_state = countPul ? S_INCR_PLOT_FAIL : S_DRAW_MINE_R;
			S_INCR_PLOT_FAIL: next_state = grid_pulse ? S_FAIL_WAIT : (is_mine ? S_DRAW_MINE_R : S_INCR_PLOT_FAIL);
			S_FAIL_WAIT: next_state = clear ? S_ERASE_GRID : S_FAIL_WAIT;
			default: next_state = S_ERASE_GRID;
		endcase
	end
	
	// Output signals
	always@(*)
	begin: enable_signals
		en_x = 1'b0;
		en_y = 1'b0;
		op_x = 1'b0;
		op_y = 1'b0;
		plot_en = 1'b0;
		erase_en = 1'b0;
		clear_en = 1'b0;
		ld_clr = 1'b0;
		mine_reset = 1'b0;
		mine_write = 1'b0;
		draw_mine = 1'b0;
		draw_click = 1'b0;
		en_shift = 1'b0;
		
		case(current_state)
			S_ERASE_GRID:
				begin
					erase_en = 1'b1;
					plot_en = 1'b1;
					ld_clr = 1'b1;
				end
			S_INCR_GRID: clear_en = 1'b1;
			S_CLEAR_WAIT: mine_reset = 1'b1;
			S_WAIT: en_shift = 1'b1;
			S_ERASE_UP:
				begin
					erase_en = 1'b1;
					plot_en = 1'b1;
				end
			S_ERASE_DOWN:
				begin
					erase_en = 1'b1;
					plot_en = 1'b1;
				end
			S_ERASE_LEFT:
				begin
					erase_en = 1'b1;
					plot_en = 1'b1;
				end
			S_ERASE_RIGHT:
				begin
					erase_en = 1'b1;
					plot_en = 1'b1;
				end
			S_INCR_UP:
				begin
					en_y = 1'b1;
					op_y = 1'b1;
				end
			S_INCR_DOWN:
				begin
					en_y = 1'b1;
					op_y = 1'b0;
				end
			S_INCR_LEFT:
				begin
					en_x = 1'b1;
					op_x = 1'b1;
				end
			S_INCR_RIGHT:
				begin
					en_x = 1'b1;
					op_x = 1'b0;
				end
			S_DRAW: plot_en = 1'b1;
			S_GENERATE: mine_write = 1'b1;
			S_DRAW_CLICK_R:
				begin
					draw_click = 1'b1;
					plot_en = 1'b1;
				end
			S_DRAW_MINE_R:
				begin
					draw_mine = 1'b1;
					plot_en = 1'b1;
					ld_clr = 1'b1;
				end
			S_INCR_PLOT_FAIL:
				begin
					clear_en = 1'b1;
					ld_clr = 1'b1;
				end
		endcase
	end
	
	// State FFs
	always@(posedge clock)
	begin: state_FFs
		if(resetn == 1'b0)
			current_state <= S_ERASE_GRID;
		else
			current_state <= next_state;
	end
	
	// register to hold if first move has occurred yet
	always@(posedge clock)begin
		if(resetn == 1'b0 || mine_reset == 1'b1)
			firstn <= 1'b0;
		else if(click == 1'b1)
			firstn <= 1'b1;
	end

endmodule


// datapath for grid select
module datapath
	(
		input clock, resetn,
		
		input en_x, en_y,
		input op_x, op_y,
		input plot_en, erase_en, clear_en, ld_clr,
		input draw_click, draw_mine,
		
		output countPul, grid_pulse, framePulse,
		
		output reg writeEn,
		
		output [5:0] Position,
		
		output [5:0] ColourOut,
		output [6:0] Yout,
		output [7:0] Xout
	);
	
	reg [2:0] grid_x, grid_y;
	reg [5:0] plot_count, grid_count;
	reg [5:0] address;
	
	reg eraser, clickr, miner;
	
	wire [5:0] grey_square, red_square; // correspond to outputs of RAM module
	wire [8:0] Xpos;
	wire [7:0] Ypos;
	
	// X position on the grid
	always@(posedge clock)begin
		if(resetn == 1'b0 || grid_pulse == 1'b1)
			grid_x <= 3'b000;
		else if(en_x == 1'b1)
			begin
				if(op_x == 1'b0)
					grid_x <= grid_x + 1;
				else
					grid_x <= grid_x - 1;
			end
	end
	
	// Y position on the grid
	always@(posedge clock)begin
		if(resetn == 1'b0 || grid_pulse == 1'b1)
			grid_y <= 3'b000;
		else if(en_y == 1'b1)
			begin
				if(op_y == 1'b0)
					grid_y <= grid_y + 1;
				else
					grid_y <= grid_y - 1;
			end
	end
	
	assign Position = {(ld_clr ? grid_count[5:3] : grid_x), (ld_clr ? grid_count[2:0] : grid_y)};
	
	assign Xpos = (ld_clr ? grid_count[5:3] : grid_x) * 4'b1000;
	assign Ypos = (ld_clr ? grid_count[2:0] : grid_y) * 4'b1000;
	
	assign Xout = Xpos + plot_count[2:0] + 8'd48; // shift to centre of screen
	assign Yout = Ypos + plot_count[5:3] + 7'd28; // shift to centre of screen (48, 28)
	
	// registers for x and y colours by pixel, as well as erase_en
	// for Xout and Yout and eraser
	always@(posedge clock)begin
		if(resetn == 1'b0)
			begin
				eraser <= 1'b0;
				clickr <= 1'b0;
				miner <= 1'b0;
				plot_count <= 6'b000000;
				writeEn <= 1'b0;
			end
		else
			begin
				eraser <= erase_en; // eraser is just a clock cycle behind erase_en
				clickr <= draw_click;
				miner <= draw_mine;
				plot_count <= address;
				writeEn <= plot_en;
			end
	end
	
	// Plot counter
	always@(posedge clock)begin
		if(resetn == 1'b0)
			address <= 6'b000000;
		else if(plot_en == 1'b1)
			address <= address + 1;
	end
	
	assign countPul = (address == 6'b111111) ? 1'b1 : 1'b0;
	
	always@(posedge clock)begin
		if(resetn == 1'b0)
			grid_count <= 6'b000000;
		else if(clear_en == 1'b1)
			grid_count <= grid_count + 1;
	end
	
	assign grid_pulse = (grid_count == 6'b111111) ? 1'b1 : 1'b0;
	
	
	// Colour management
	
	// RAM Modules
	red_square_64x6 R0( .address(address), .clock(clock), .data(6'b000000), .wren(1'b0), .q(red_square)); // RAM for a red square image
	grey_square_64x6 R1( .address(address), .clock(clock), .data(6'b000000), .wren(1'b0), .q(grey_square)); // RAM for a grey square image
	
	// MUX
	assign ColourOut = clickr ? 6'b111111 : (miner ? (6'b011101) : (eraser ? grey_square : (6'b110000 + grey_square)));
	
	// Rate Divider the counts 1 frame
	RateDivider R2(.R(26'd833333), .Clock(clock), .Qpul(framePulse));
	
endmodule

// Rate divider which provides a signal that a frame has passed
module RateDivider(input [25:0] R, input Clock, output Qpul); // reset is needed for simulations
	
	reg [25:0] Q;
	
	always@(posedge Clock)begin
		if(Qpul == 1'b1) // When pulse is sent, reset to R!
			Q <= R;
		else
			Q <= Q-1;
	end
	
	assign Qpul = ~|Q; // NOR of all bits of Q, when Q is 0
	
endmodule

// Seven segment decoder
module hex_decoder(input [3:0] hex_digits, output reg [6:0] segments);
	
	always @(*)begin
        case (hex_digits)
            4'h0: segments = 7'b1000000;
            4'h1: segments = 7'b1111001;
            4'h2: segments = 7'b0100100;
            4'h3: segments = 7'b0110000;
            4'h4: segments = 7'b0011001;
            4'h5: segments = 7'b0010010;
            4'h6: segments = 7'b0000010;
            4'h7: segments = 7'b1111000;
            4'h8: segments = 7'b0000000;
            4'h9: segments = 7'b0011000;
            4'hA: segments = 7'b0001000;
            4'hB: segments = 7'b0000011;
            4'hC: segments = 7'b1000110;
            4'hD: segments = 7'b0100001;
            4'hE: segments = 7'b0000110;
            4'hF: segments = 7'b0001110;   
            default: segments = 7'h7f;
		  endcase
	end
	
endmodule


