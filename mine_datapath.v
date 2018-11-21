`timescale 1ns/1ns

module mine_datapath
	(
		input clock, resetn,
		input mine_reset, en_write, en_shift,
		input [5:0] position,
		
		output is_mine,
		output end_pulse,
		output reset_pulse
	);
	
	
	wire [31:0] mine0, mine1;
	wire address;
	wire wren;
	
	reg [3:0] select;
	reg [5:0] mine_site;
	reg enabler;
	reg [63:0] mine_data; // 64 bit register for mine data :)
	
	reg [5:0] reset_count;
	
	wire data_out;
	
	assign is_mine = 	data_out | (en_write ? (( (position == mine_site) ? ( 1'b1 ) : ( 1'b0 ) )) : 1'b0); // either dataout is 1 which represents a mine or fakes a mine on mine writing if the coordinate matches the mine site
	assign end_pulse = ((select == 4'b1001)?1'b1:1'b0) & enabler & ~is_mine;
	assign reset_pulse = (reset_count == 6'b111111)?1'b1:1'b0;
	
	// two 32 bit LSFRs with different seeds
	LFSR_32bit L0( .clock(clock), .resetn(resetn), .seed(32'hFE37C0DE), .ld(mine_reset), .en(en_shift | is_mine), .q(mine0) );
	LFSR_32bit L1( .clock(clock), .resetn(resetn), .seed(32'hDAD50A55), .ld(mine_reset), .en(en_shift | is_mine), .q(mine1) );
	
	// keep changing position taken off of 32 bit LFSR to produce coordinates
	always@(*)begin
		case(select)
			4'd0: mine_site = mine0[29:24];
			4'd1: mine_site = mine0[23:18];
			4'd2: mine_site = mine0[17:12];
			4'd3: mine_site = mine0[11:6];
			4'd4: mine_site = mine0[5:0];
			4'd5: mine_site = mine1[29:24];
			4'd6: mine_site = mine1[23:18];
			4'd7: mine_site = mine1[17:12];
			4'd8: mine_site = mine1[11:6];
			4'd9: mine_site = mine1[5:0];
			default: mine_site = 6'b000000;
		endcase
	end
	
	// counter counting how many mines have been added successfully to the RAM
	always@(posedge clock)begin
		if(resetn == 1'b0 || mine_reset == 1'b1)
			select <= 4'b0000;
		else if(is_mine == 1'b0 && enabler == 1'b1)
			begin
				if(select == 4'b1001)
					select <= 4'b0000;
				else
					select <= select + 1;
			end
	end
	
	// T flip flop for enabler
	always@(posedge clock)begin
		if(resetn == 1'b0 || mine_reset == 1'b1)
			enabler <= 1'b0;
		else if(is_mine == 1'b0)
			enabler <= ~enabler;
	end
	
	// ram holding location of mines
	mine_data_64x1 R0
	( 
			.clock(clock),
			.wren(mine_reset ? 1'b1 : (en_write & ~data_out & enabler) ), // specific case to write
			.data(mine_reset ? 1'b0 : 1'b1), 
			.address( mine_reset ? reset_count : (en_write ? ( mine_site ) : (position)) ), // determine which address to read
			.q(data_out)
	);
	
//	assign address = en_write ? ( mine_site ) : (position);
//	assign wren = en_write & ~data_out & enabler;
//	
//	always@(posedge clock)begin
//		if(resetn == 1'b0 || mine_reset == 1'b1)
//			mine_data <= 64'd0;
//		else if(wren == 1'b1)
//			mine_data[address] <= 1'b1;
//		else if(wren == 1'b0)
//			data_out <= mine_data[address];
//	end

	always@(posedge clock)begin
		if(resetn == 1'b0)
			reset_count <= 6'b000000;
		else if(mine_reset = 1'b1)
			reset_count <= reset_count + 1;
	end
	
endmodule


// Random number generator
module LFSR_32bit(input clock, resetn, input [31:0] seed, input ld, en, output reg [31:0] q);
	
	wire end_bit;
	assign end_bit = q[22] ^ q[2] ^ q[1] ^ q[0];
	
	always@(posedge clock)begin
		if(resetn == 1'b0)
			q <= 32'b00000000000000000000000000000000;
		else if(ld == 1'b1)
			q <= seed;
		else if(en == 1'b1)
			begin
				q <= q << 1;
				q[0] <= end_bit;
			end
	end

endmodule

		