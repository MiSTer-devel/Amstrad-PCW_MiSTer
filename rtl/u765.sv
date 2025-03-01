// ====================================================================
//
//  NEC u765 FDC
//
//  Copyright (C) 2017 Gyorgy Szombathelyi <gyurco@freemail.hu>
//
//  Updated for PCW disks and timing, including interrupt, ndma mode and TC handling
//  Changes Copyright (C) 2020 Stephen Eddy
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

//TODO:
//GAP, CRC generation
//WRITE DELETE should write the Deleted Address Mark to the SectorInfo
//SCAN commands
//real FORMAT (but this would require squeezing/expanding the image file)

// For accurate head stepping rate, set CYCLES to cycles/ms
// 4MHz = 4000 (default).  If a faster clock is fed in, this will just speed up the simulation in line
// SPECCY_SPEEDLOCK_HACK: auto mess-up weak sector on C0H0S2
module u765 #(parameter CYCLES = 20'd4000, SPECCY_SPEEDLOCK_HACK = 0)
(
	input wire       clk_sys,   // sys clock
	input wire       ce,        // chip enable
	input wire       reset,	    // reset
	input wire [1:0] ready,     // disk is inserted in MiST(er)
	input wire [1:0] motor,     // drive motor
	input wire [1:0] available, // drive available (fake ready signal for SENSE DRIVE command)
	input wire       a0,
	input wire       nRD,       // i/o read
	input wire       nWR,       // i/o write
	input wire   [7:0] din,       // i/o data in
	output logic [7:0] dout,      // i/o data out

	input wire		 tc,		// terminal count (terminate)
	output logic     int_out,   // Output interrupt line
	input wire [1:0] density,	// CF2 = 0, CF2DD = 1
	output logic 	 activity_led,	// Activity LED

	input wire  [1:0] img_mounted, // signaling that new image has been mounted
	input wire        img_wp,      // write protect. latched at img_mounted
	input wire [31:0] img_size,    // size of image in bytes
	output logic [31:0] sd_lba,
	output logic  [1:0] sd_rd,
	output logic  [1:0] sd_wr,
	input wire       sd_ack,
	input wire [8:0] sd_buff_addr,
	input wire [7:0] sd_buff_dout,
	output logic [7:0] sd_buff_din,
	input wire         sd_buff_wr,

	output logic [7:0] old_state
);

//localparam OVERRUN_TIMEOUT = 26'd35000000;
// This should equal 13 us
localparam OVERRUN_TIMEOUT = CYCLES * 10'd100;		// 13us seconds assuming base clock of 4Mhz
// Sector time - We are going to fix this to 9 sectors per track for PCW
localparam SECTOR_TIME = ((CYCLES * 20'd200) / 20'd9) / 20'd4;  // SECTOR time for timing of disk speed.

localparam UPD765_MAIN_D0B = 0;
localparam UPD765_MAIN_D1B = 1;
localparam UPD765_MAIN_D2B = 2;
localparam UPD765_MAIN_D3B = 3;
localparam UPD765_MAIN_CB  = 4;
localparam UPD765_MAIN_EXM = 5;
localparam UPD765_MAIN_DIO = 6;
localparam UPD765_MAIN_RQM = 7;

// Disk densities
localparam CF2 = 1'b0;
localparam CF2DD = 1'b1;

localparam UPD765_SD_BUFF_TRACKINFO = 1'd0;
localparam UPD765_SD_BUFF_SECTOR = 1'd1;

typedef enum bit [5:0]
{
 COMMAND_IDLE,     					// 00 - x00            

 COMMAND_READ_TRACK,  				// 01 - x01

 COMMAND_WRITE_DELETED_DATA, 		// 02 - x02
 COMMAND_WRITE_DATA, 				// 03 - x03

 COMMAND_READ_DELETED_DATA,  		// 05 - x04 
 COMMAND_READ_DATA,  				// 05 - x05

 COMMAND_RW_DATA_EXEC,  			// 06 - x06
 COMMAND_RW_DATA_EXEC1, 			// 07 - x07
 COMMAND_RW_DATA_EXEC2, 			// 08 - x08
 COMMAND_RW_DATA_EXEC3, 			// 09 - x09
 COMMAND_RW_DATA_EXEC4, 			// 10 - x0a
 COMMAND_RW_DATA_EXEC5, 			// 11 - x0b
 COMMAND_RW_DATA_WAIT_SECTOR, 		// 12 - x0c
 COMMAND_RW_DATA_EXEC_WEAK, 		// 13 - x0d
 COMMAND_RW_DATA_EXEC6, 			// 14 - x0e
 COMMAND_RW_DATA_EXEC7, 			// 15 - x0f
 COMMAND_RW_DATA_EXEC8, 			// 16 - x10

 COMMAND_READ_ID, 					// 17 - x11
 COMMAND_READ_ID1, 					// 18 - x12
 COMMAND_READ_ID2, 					// 19 - x13
 COMMAND_READ_ID_EXEC1, 			// 20 - x14
 COMMAND_READ_ID_WAIT_SECTOR, 		// 21 - x15
 COMMAND_READ_ID_EXEC2, 			// 22 - x16

 COMMAND_FORMAT_TRACK, 				// 23 - x17
 COMMAND_FORMAT_TRACK1, 			// 24 - x18
 COMMAND_FORMAT_TRACK2, 			// 25 - x19
 COMMAND_FORMAT_TRACK3, 			// 26 - x1a
 COMMAND_FORMAT_TRACK4, 			// 27 - x1b
 COMMAND_FORMAT_TRACK5, 			// 28 - x1c
 COMMAND_FORMAT_TRACK6, 			// 29 - x1d
 COMMAND_FORMAT_TRACK7, 			// 30 - x1e
 COMMAND_FORMAT_TRACK8, 			// 31 - x1f

 COMMAND_SCAN_EQUAL, 				// 32 - x20
 COMMAND_SCAN_LOW_OR_EQUAL, 		// 33 - x21
 COMMAND_SCAN_HIGH_OR_EQUAL, 		// 34 - x22

 COMMAND_RECALIBRATE, 				// 35 - x23

 COMMAND_SENSE_INTERRUPT_STATUS, 	// 36 - x24
 COMMAND_SENSE_INTERRUPT_STATUS1, 	// 37 - x25
 COMMAND_SENSE_INTERRUPT_STATUS2, 	// 38 - x26

 COMMAND_SPECIFY, 					// 39 - x27
 COMMAND_SPECIFY_WR, 				// 40 - x28

 COMMAND_SENSE_DRIVE_STATUS, 		// 41 - x29
 COMMAND_SENSE_DRIVE_STATUS_RD, 	// 42 - x2a

 COMMAND_SEEK, 						// 43 - x2b
 COMMAND_SEEK_EXEC1, 				// 44 - x2c

 COMMAND_SETUP, 					// 45 - x2d

 COMMAND_READ_RESULTS, 				// 46 - x2e

 COMMAND_INVALID, 					// 47 - x2f
 COMMAND_INVALID1, 					// 48 - x30

 COMMAND_RELOAD_TRACKINFO, 			// 49 - x31
 COMMAND_RELOAD_TRACKINFO1, 		// 50 - x32
 COMMAND_RELOAD_TRACKINFO2, 		// 51 - x33
 COMMAND_RELOAD_TRACKINFO3 			// 52 - x34

} state_t;

typedef enum bit [1:0] {
	PHASE_COMMAND,
	PHASE_EXECUTE,
	PHASE_RESPONSE
} phase_t;

// sector/trackinfo buffers
logic    [7:0] buff_data_in/* synthesis keep */;
logic    [7:0] buff_data_out;
logic    [8:0] buff_addr;
logic          buff_wr, buff_wait;
logic          sd_buff_type;
logic          hds, ds0;

u765_dpram sbuf
(
	.clock(clk_sys),
	// SD card read / write access
	.address_a({ds0, sd_buff_type,hds,sd_buff_addr}),
	.data_a(sd_buff_dout),
	.wren_a(sd_buff_wr & sd_ack),
	.q_a(sd_buff_din),
	// FDC module read write access for processor
	.address_b({ds0, sd_buff_type,hds,buff_addr}),
	.data_b(buff_data_out),
	.wren_b(buff_wr),
	.q_b(buff_data_in)
);

// // Clone SD writes into sd_debug for memory debugging using in system member debugger in Quartus
// // Comment out this section for release
// logic [7:0] debug_data /*synthesis noprune*/;
// sd_debug sd_debug(
// 	.clock(clk_sys),
// 	.address({ds0, sd_buff_type,hds,sd_buff_addr}),
// 	.data(sd_buff_dout),
// 	.wren(sd_buff_wr & sd_ack),
// 	.q(debug_data)
// );

//track offset buffer
//single port buffer in RAM
logic [15:0] image_track_offsets[1024]; //offset of tracks * 256 * 2 drives
reg    [8:0] image_track_offsets_addr = 0;
reg          image_track_offsets_wr;
reg   [15:0] image_track_offsets_out, image_track_offsets_in;

always @(posedge clk_sys) begin
	if (image_track_offsets_wr) begin
		image_track_offsets[{ds0, image_track_offsets_addr}] <= image_track_offsets_out;
		image_track_offsets_in <= image_track_offsets_out;
	end else begin
		image_track_offsets_in <= image_track_offsets[{ds0, image_track_offsets_addr}];
	end
end

logic       rd;
assign rd = nWR & ~nRD;
logic       wr;
assign wr = ~nWR & nRD;
logic [7:0] i_total_sectors;

phase_t phase;

reg  [7:0] m_status;  //main status register
reg  [7:0] m_data;    //data register
reg int_state[2];	// interrupt states for both drives

logic ndma_mode = 1'b1;
state_t last_state/* synthesis noprune */;

assign int_out = int_state[0] | int_state[1];
assign dout = a0 ? m_data : m_status;
assign old_state = last_state;
assign activity_led = (phase == PHASE_EXECUTE);

always @(posedge clk_sys) begin

   //prefix internal CE protected registers with i_, so it's easier to write constraints

	//per-drive data
	reg[31:0] image_size[2];
	reg       image_ready[2] = '{ 0, 0 };
	reg [7:0] image_tracks[2];
	reg       image_sides[2]; //1 side - 0, 2 sides - 1
	reg [1:0] image_wp;
	reg       image_trackinfo_dirty[2];
	reg       image_edsk[2]; //DSK - 0, EDSK - 1
	reg [1:0] image_scan_state[2] = '{ 0, 0 };
	reg [1:0] image_density;
	reg [7:0] i_current_track_sectors[2][2] /* synthesis keep */;  //number of sectors on the current track /head/drive
	reg [7:0] i_current_sector_pos[2][2] /* synthesis keep */; //sector where the head currently positioned
	reg[19:0] i_steptimer[2], i_rpm_timer[2][2];
	reg [3:0] i_step_state[2]; //counting cycles_time for steptimer

	reg [7:0] ncn[2]; //new cylinder number
	reg [7:0] pcn[2]; //present cylinder number
	reg [2:0] next_weak_sector[2];
	reg [1:0] seek_state[2];

	reg old_wr, old_rd;
	reg [7:0] i_track_size;
	reg [31:0] i_seek_pos;
	reg [7:0] i_sector_c, i_sector_h, i_sector_r, i_sector_n;
	reg [7:0] i_sector_st1, i_sector_st2;
	reg [15:0] i_sector_size;
	reg [7:0] i_current_sector;
	reg i_scanning;
 	reg [2:0] i_weak_sector;
	reg [15:0] i_bytes_to_read;
	reg [2:0] i_substate;
	reg [1:0] old_mounted;
	reg [15:0] i_track_offset;
	reg [5:0] ack;
	reg sd_busy;
	reg [19:0] i_timeout;
	reg [7:0] i_head_timer;
	reg i_rtrack, i_write, i_rw_deleted;
	reg [7:0] status[4] = '{0, 0, 0, 0}; //st0-3
	state_t state;
	state_t i_command;
   	reg i_current_drive, i_scan_lock = 0;
	reg [3:0] i_srt; //stepping rate
	reg [3:0] i_hut; //head unload time
	reg [6:0] i_hlt; //head load time
	reg [7:0] i_c;
	reg [7:0] i_h;
	reg [7:0] i_r;
	reg [7:0] i_n;
	reg [7:0] i_eot;
	//reg [7:0] i_gpl;
	reg [7:0] i_dtl;
	reg [7:0] i_sc;
	//reg [7:0] i_d;
	reg i_bc; //bad cylinder
	reg old_hds;
	reg old_tc;
	logic [7:0] tmp_ncn;

	reg i_mt;
	//reg i_mfm;
	reg i_sk;

	buff_wait <= 0;
	i_total_sectors = i_current_track_sectors[ds0][hds];

	//new image mounted
	for(int i=0;i<2;i++) begin 
		old_mounted[i] <= img_mounted[i];
		if(old_mounted[i] & ~img_mounted[i]) begin
			image_wp[i] <= img_wp;
			image_size[i] <= img_size;
			image_scan_state[i] <= |img_size; //hacky
			image_ready[i] <= 0;
			image_density[i] <= (img_size > 250000) ? CF2DD : CF2; // very hacky
			int_state[i] <= 0;
			seek_state[i] <= 0;
			next_weak_sector[i] <= 0;
			i_current_sector_pos[i] <= '{ 0, 0 };
		end
	end

	if (ce) begin
		i_current_drive <= ~i_current_drive;
	end

   //Process the image file
	if (ce) begin
		case (image_scan_state[i_current_drive])
			0: ;//no new image
			1: //read the first 512 byte
				if (~sd_busy & ~i_scan_lock & state == COMMAND_IDLE) begin
					sd_buff_type <= UPD765_SD_BUFF_SECTOR;
					i_scan_lock <= 1;
					ds0 <= i_current_drive;
					sd_rd[i_current_drive] <= 1;
					sd_lba <= 0;
					sd_busy <= 1;
					i_track_offset<= 16'h1; //offset 100h
					image_track_offsets_addr <= 0;
					buff_addr <= 0;
					buff_wait <= 1;
					image_scan_state[i_current_drive] <= 2;
				end
			2: //process the header - Update all the image track offsets for every track
				if (~sd_busy & ~buff_wait) begin
					if (buff_addr == 0) begin
						if (buff_data_in == "E")
							image_edsk[i_current_drive] <= 1;
						else if (buff_data_in == "M")
							image_edsk[i_current_drive] <= 0;
						else begin
							image_ready[i_current_drive] <= 0;
							image_scan_state[i_current_drive] <= 0;
							i_scan_lock <= 0;
						end
					end else if (buff_addr == 9'h30) image_tracks[i_current_drive] <= buff_data_in;
					else if (buff_addr == 9'h31) image_sides[i_current_drive] <= buff_data_in[1];
					else if (buff_addr == 9'h33) i_track_size <= buff_data_in;
					else if (buff_addr >= 9'h34) begin
						if (image_track_offsets_addr[8:1] != image_tracks[i_current_drive]) begin
							image_track_offsets_wr <= 1;
							if (image_edsk[i_current_drive]) begin
								image_track_offsets_out <= buff_data_in ? i_track_offset : 16'd0;
								i_track_offset <= i_track_offset + buff_data_in;
							end else begin
								image_track_offsets_out <= i_track_offset;
								i_track_offset <= i_track_offset + i_track_size;
							end
							image_scan_state[i_current_drive] <= 3;
						end else begin
							image_ready[i_current_drive] <= 1;
							image_scan_state[i_current_drive] <= 0;
							image_trackinfo_dirty[i_current_drive] <= 1;
							i_scan_lock <= 0;
						end
					end
					buff_addr <= buff_addr + 1'd1;
					buff_wait <= 1;
				end
			3: begin
					image_track_offsets_wr <= 0;
					image_track_offsets_addr <= image_track_offsets_addr + { ~image_sides[i_current_drive], image_sides[i_current_drive] };
					image_scan_state[i_current_drive] <= 2;
				end
		endcase
	end

	//the FDC
   if (reset) begin
	  	old_tc <= 1'b0;
		m_status <= 8'h80;
		state <= COMMAND_IDLE;
		last_state <= COMMAND_IDLE;
		phase <= PHASE_COMMAND;
		status[0] <= 0;
		status[1] <= 0;
		status[2] <= 0;
		status[3] <= 0;
		ncn <= '{ 0, 0 };
		pcn <= '{ 0, 0 };
		int_state <= '{ 0, 0 };
		seek_state <= '{ 0, 0 };
		image_trackinfo_dirty <= '{ 1, 1 };
		{ ack, sd_busy } <= 0;
		sd_rd <= 0;
		sd_wr <= 0;
		sd_busy <= 0;
		image_track_offsets_wr <= 0;
		//restart "mounting" of image(s)
		if (image_scan_state[0]) image_scan_state[0] <= 1;
		if (image_scan_state[1]) image_scan_state[1] <= 1;
		i_scan_lock <= 0;
		i_srt <= 4;
		ndma_mode <= 1'b1;
	end else if (ce) begin

		ack <= {ack[4:0], sd_ack};
		if(ack[5:4] == 'b01)	begin
			sd_rd <= 0;
			sd_wr <= 0;
		end
		if(ack[5:4] == 'b10) sd_busy <= 0;

		old_wr <= wr;
		old_rd <= rd;

		//seek (track stepping - step 0 = not stepping)
		case(seek_state[i_current_drive])
			0: ;//no seek in progress
			1: if (pcn[i_current_drive] == ncn[i_current_drive]) begin
					int_state[i_current_drive] <= 1;
					seek_state[i_current_drive] <= 0;
					//i_current_sector <= 1'd1;
				end else begin
					image_trackinfo_dirty[i_current_drive] <= 1;
					begin
						if (pcn[i_current_drive] > ncn[i_current_drive]) pcn[i_current_drive] <= pcn[i_current_drive] - 1'd1;
						if (pcn[i_current_drive] < ncn[i_current_drive]) pcn[i_current_drive] <= pcn[i_current_drive] + 1'd1;
						i_step_state[i_current_drive] <= i_srt;
						i_steptimer[i_current_drive] <= CYCLES;
						seek_state[i_current_drive] <= 2;
					end
				end
			2: if(i_steptimer[i_current_drive]) begin
					i_steptimer[i_current_drive] <= i_steptimer[i_current_drive] - 1'd1;
				end else if (~&i_step_state[i_current_drive]) begin
					i_step_state[i_current_drive] <= i_step_state[i_current_drive] + 1'd1;
					i_steptimer[i_current_drive] <= CYCLES;
				end else begin
					seek_state[i_current_drive] <= 1;
				end
		endcase

		//disk rotation
		if (motor[i_current_drive]) begin
			for (int i=0; i<2 ;i++) begin
				if (i_rpm_timer[i_current_drive][i] >= SECTOR_TIME) begin
					// i_current_sector_pos is physical sector number on track (e.g. 1,2,3,etc)
					i_current_sector_pos[i_current_drive][i] <=
					i_current_sector_pos[i_current_drive][i] == i_current_track_sectors[i_current_drive][i] - 1'd1 ?
						8'd0 : i_current_sector_pos[i_current_drive][i] + 1'd1;
					i_rpm_timer[i_current_drive][i] <= 0;
				end else begin
					if(state != COMMAND_RW_DATA_EXEC5 &&
						state != COMMAND_RW_DATA_EXEC6 &&
						state != COMMAND_RW_DATA_EXEC7)
						i_rpm_timer[i_current_drive][i] <= i_rpm_timer[i_current_drive][i] + 1'd1;
				end
			end
		end

		m_status[UPD765_MAIN_D0B] <= |seek_state[0];
		m_status[UPD765_MAIN_D1B] <= |seek_state[1];
		m_status[UPD765_MAIN_CB] <= state != COMMAND_IDLE;

		old_tc <= tc;
		if(~old_tc && tc && m_status[UPD765_MAIN_EXM]) begin // && ~sd_busy) begin		// TC signal jump straight to reading results
			state <= COMMAND_READ_RESULTS ;	// TC caused drive to reset state
			phase <= PHASE_RESPONSE;
			m_status[UPD765_MAIN_EXM] <= 1'b0;
			case(last_state)
				COMMAND_SCAN_EQUAL,
				COMMAND_SCAN_LOW_OR_EQUAL,
				COMMAND_SCAN_HIGH_OR_EQUAL,
				COMMAND_READ_DATA,
				COMMAND_READ_DELETED_DATA,
				COMMAND_FORMAT_TRACK,
				COMMAND_WRITE_DATA: int_state[ds0] <= 1'b1;
				default: int_state[ds0] <= 1'b0;
			endcase
//			int_state[ds0] <= 1'b1;
			i_substate <= 0;
		end 
		else begin
			case(state)
				COMMAND_IDLE:
				begin
					m_status[UPD765_MAIN_DIO] <= 0;
					m_status[UPD765_MAIN_RQM] <= !image_scan_state[0] & !image_scan_state[1];
					// reset tc
					//tc <= 1'b0;
					phase <= PHASE_COMMAND;
					if (~old_wr & wr & a0 & !image_scan_state[0] & !image_scan_state[1]) begin
						i_mt <= din[7];
						//i_mfm <= din[6];
						i_sk <= din[5];

						i_substate <= 0;
						
						casex (din[7:0])
							8'bXXX_00110: begin state <= COMMAND_READ_DATA; last_state <= COMMAND_READ_DATA; end
							8'bXXX_01100: begin state <= COMMAND_READ_DELETED_DATA; last_state <= COMMAND_READ_DELETED_DATA; end
							8'bXX0_00101: begin state <= COMMAND_WRITE_DATA; last_state <= COMMAND_WRITE_DATA; end
							8'bXX0_01001: begin state <= COMMAND_WRITE_DELETED_DATA; last_state <= COMMAND_WRITE_DELETED_DATA; end
							8'b0XX_00010: begin state <= COMMAND_READ_TRACK; last_state <= COMMAND_READ_TRACK; end
							8'b0X0_01010: begin state <= COMMAND_READ_ID; last_state <= COMMAND_READ_ID;  end
							8'b0X0_01101: begin state <= COMMAND_FORMAT_TRACK; last_state <= COMMAND_FORMAT_TRACK;  end
							8'bXXX_10001: begin state <= COMMAND_SCAN_EQUAL; last_state <= COMMAND_SCAN_EQUAL; end
							8'bXXX_11001: begin state <= COMMAND_SCAN_LOW_OR_EQUAL; last_state <= COMMAND_SCAN_LOW_OR_EQUAL;  end
							8'bXXX_11101: begin state <= COMMAND_SCAN_HIGH_OR_EQUAL; last_state <= COMMAND_SCAN_HIGH_OR_EQUAL; end 
							8'b000_00111: begin state <= COMMAND_RECALIBRATE; last_state <= COMMAND_RECALIBRATE; end
							8'b000_01000: begin state <= COMMAND_SENSE_INTERRUPT_STATUS; last_state <= COMMAND_SENSE_INTERRUPT_STATUS; end
							8'b000_00011: begin state <= COMMAND_SPECIFY; last_state <= COMMAND_SPECIFY; end
							8'b000_00100: begin state <= COMMAND_SENSE_DRIVE_STATUS; last_state <= COMMAND_SENSE_DRIVE_STATUS; end
							8'b000_01111: begin state <= COMMAND_SEEK; last_state <= COMMAND_SEEK; end
							default: begin state <= COMMAND_INVALID; last_state <= COMMAND_INVALID; end 
						endcase
					end else if(~old_rd & rd & a0) begin
						m_data <= 8'hff;
					end
				end

				COMMAND_SENSE_INTERRUPT_STATUS:
				begin
					m_status[UPD765_MAIN_DIO] <= 1;
					state <= COMMAND_SENSE_INTERRUPT_STATUS1;
				end

				COMMAND_SENSE_INTERRUPT_STATUS1:
				if (~old_rd & rd & a0) begin
					if (int_state[0]) begin
						m_data <= ( ncn[0] == pcn[0] && ready[0] && image_ready[0] ) ? 8'h20 : 8'he8; //drive A: interrupt
						state <= COMMAND_SENSE_INTERRUPT_STATUS2;
					end else if (int_state[1]) begin
						m_data <= ( ncn[1] == pcn[1] && ready[1] && image_ready[1] ) ? 8'h21 : 8'he9; //drive B: interrupt
						state <= COMMAND_SENSE_INTERRUPT_STATUS2;
					end else begin
						m_data <= 8'h80;
						state <= COMMAND_IDLE;
					end;
				end

				COMMAND_SENSE_INTERRUPT_STATUS2:
				if (~old_rd & rd & a0) begin
					m_data <= int_state[0] ? 
						((image_density[0]==CF2 && density[0]==CF2DD)  ? pcn[0] << 1 : pcn[0]) :  
						((image_density[1]==CF2 && density[1]==CF2DD) ? pcn[1] << 1 : pcn[1]);
					int_state[int_state[0] ? 0 : 1] <= 0;
					state <= COMMAND_IDLE;
				end

				COMMAND_SENSE_DRIVE_STATUS:
				begin
					int_state <= '{ 0, 0 };
					if (~old_wr & wr & a0) begin
						state <= COMMAND_SENSE_DRIVE_STATUS_RD;
						m_status[UPD765_MAIN_DIO] <= 1;
						ds0 <= din[0];
						hds <= image_density[din[0]] ? din[2] : 1'b0;	// Was missing
					end
				end

				COMMAND_SENSE_DRIVE_STATUS_RD:
				if (~old_rd & rd & a0) begin
					m_data <= { 1'b0,
								ready[ds0] & image_wp[ds0],         //write protected
								motor[ds0] & available[ds0],        //ready - needed for controller detection
								image_ready[ds0] & !pcn[ds0],       //track 0
								image_ready[ds0] & image_sides[ds0],//two sides
								image_ready[ds0] & hds,             //head address
								1'b0,                               //us1
								ds0 };                              //us0
					state <= COMMAND_IDLE;
				end

				COMMAND_SPECIFY:
				begin
					int_state <= '{ 0, 0 };
					if (~old_wr & wr & a0) begin
						i_hut <= din[3:0];
						i_srt <= din[7:4];
						state <= COMMAND_SPECIFY_WR;
					end
				end

				COMMAND_SPECIFY_WR:
				if (~old_wr & wr & a0) begin
					i_hlt <= din[7:1];
					ndma_mode <= din[0];
					state <= COMMAND_IDLE;
				end

				COMMAND_RECALIBRATE:
				begin
					if (~old_wr & wr & a0) begin
						ds0 <= din[0];
						int_state[din[0]] <= 0;
						ncn[din[0]] <= 0;
						seek_state[din[0]] <= 1;
						state <= COMMAND_IDLE;
					end
				end

				COMMAND_SEEK:
				begin
					if (~old_wr & wr & a0) begin
						ds0 <= din[0];
						hds <= image_density[din[0]] ? din[2] : 1'b0;	// Was missing
						int_state[din[0]] <= 0;
						state <= COMMAND_SEEK_EXEC1;
					end
				end

				COMMAND_SEEK_EXEC1:
				if (~old_wr & wr & a0) begin
					// This next line is intentionally blocking
					if(image_density[ds0]==CF2 && density[ds0]==CF2DD)
					begin
						ncn[ds0] <= din >> 1;
						if ((motor[ds0] && ready[ds0] && image_ready[ds0] && (din >> 1)<image_tracks[ds0]) || !din) begin
							seek_state[ds0] <= 1;
						end else begin
							//Seek error
							int_state[ds0] <= 1;				
						end
					end 
					else begin
						ncn[ds0] <= din;
						if ((motor[ds0] && ready[ds0] && image_ready[ds0] && din<image_tracks[ds0]) || !din) begin
							seek_state[ds0] <= 1;
						end else begin
							//Seek error
							int_state[ds0] <= 1;				
						end
					end
					state <= COMMAND_IDLE;
				end

				COMMAND_READ_ID:
				begin
					int_state <= '{ 0, 0 };
					state <= COMMAND_READ_ID1;
				end

				COMMAND_READ_ID1:
				if (~old_wr & wr & a0) begin
					ds0 <= din[0];
					if (~motor[din[0]] | ~ready[din[0]] | ~image_ready[din[0]]) begin
						status[0] <= 8'h40;
						status[1] <= 8'b101;
						status[2] <= 0;
						state <= COMMAND_READ_RESULTS;
						int_state[din[0]] <= 1'b1;
						phase <= PHASE_RESPONSE;
					end else	if (din[2] & ~image_sides[din[0]]) begin
						status[0] <= 8'h48; //no side B
						status[1] <= 0;
						status[2] <= 0;
						state <= COMMAND_READ_RESULTS;
						int_state[din[0]] <= 1'b1;
						phase <= PHASE_RESPONSE;
					end else begin
						hds <= image_density[din[0]] ? din[2] : 1'b0;
						m_status[UPD765_MAIN_RQM] <= 0;
						i_command <= COMMAND_READ_ID2;
						state <= COMMAND_RELOAD_TRACKINFO;
						phase <= PHASE_EXECUTE;
					end
				end

				COMMAND_READ_ID2:
				begin
					image_track_offsets_addr <= { pcn[ds0], hds };
					buff_wait <= 1;
					state <= COMMAND_READ_ID_EXEC1;
				end

				COMMAND_READ_ID_EXEC1:
				if (~sd_busy & ~buff_wait) begin
					if (image_track_offsets_in) begin
						state <= COMMAND_READ_ID_WAIT_SECTOR;
					end else begin
						//empty track
						status[0] <= 8'h40;
						status[1] <= 8'b101;
						status[2] <= 0;
						state <= COMMAND_READ_RESULTS;
						int_state[ds0] <= 1'b1;
						phase <= PHASE_RESPONSE;
					end
				end

				// Actually sets the offset to sector table in track (started with TRACKINFO)
				COMMAND_READ_ID_WAIT_SECTOR:
				if (~sd_busy & ~buff_wait & (!i_rpm_timer[ds0][hds])) begin
					sd_buff_type <= UPD765_SD_BUFF_TRACKINFO;
					// 18h = offset to list of sectors in sector table for current track
					buff_addr <= { image_track_offsets_in[0], 8'h18 + (i_current_sector_pos[ds0][hds] << 3) }; //get the current sectorInfo
					buff_wait <= 1;
					state <= COMMAND_READ_ID_EXEC2;
				end

				COMMAND_READ_ID_EXEC2:
				if (~buff_wait) begin
					if (buff_addr[2:0] == 8'h00) i_sector_c <= buff_data_in;
					else if (buff_addr[2:0] == 8'h01) i_sector_h <= buff_data_in;
					else if (buff_addr[2:0] == 8'h02) i_sector_r <= buff_data_in;
					else if (buff_addr[2:0] == 8'h03) begin
						i_sector_n <= buff_data_in;
						status[0] <= 0;
						status[1] <= 0;
						status[2] <= 0;
						state <= COMMAND_READ_RESULTS;
						int_state[ds0] <= 1'b1;
						phase <= PHASE_RESPONSE;
					end
					buff_addr <= buff_addr + 1'd1;
					buff_wait <= 1;
				end

				COMMAND_READ_TRACK:
				begin
					int_state <= '{ 0, 0 };
					i_command <= COMMAND_RW_DATA_EXEC;
					state <= COMMAND_SETUP;
					{i_rtrack, i_write, i_rw_deleted} <= 3'b100;
				end

				COMMAND_WRITE_DATA:
				begin
					int_state <= '{ 0, 0 };
					i_command <= COMMAND_RW_DATA_EXEC;
					state <= COMMAND_SETUP;
					{i_rtrack, i_write, i_rw_deleted} <= 3'b010;
				end

				COMMAND_WRITE_DELETED_DATA:
				begin
					int_state <= '{ 0, 0 };
					i_command <= COMMAND_RW_DATA_EXEC;
					state <= COMMAND_SETUP;
					{i_rtrack, i_write, i_rw_deleted} <= 3'b011;
				end

				COMMAND_READ_DATA:
				begin
					int_state <= '{ 0, 0 };
					i_command <= COMMAND_RW_DATA_EXEC;
					state <= COMMAND_SETUP;
					{i_rtrack, i_write, i_rw_deleted} <= 3'b000;
				end

				COMMAND_READ_DELETED_DATA:
				begin
					int_state <= '{ 0, 0 };
					i_command <= COMMAND_RW_DATA_EXEC;
					state <= COMMAND_SETUP;
					{i_rtrack, i_write, i_rw_deleted} <= 3'b001;
				end

				COMMAND_RW_DATA_EXEC:
				if (i_write & image_wp[ds0]) begin
					status[0] <= 8'h40;
					status[1] <= 8'h02; //not writeable
					status[2] <= 0;
					state <= COMMAND_READ_RESULTS;
					int_state[ds0] <= 1'b1;
					phase <= PHASE_RESPONSE;
				end else begin
					m_status[UPD765_MAIN_RQM] <= 0;
					i_command <= COMMAND_RW_DATA_EXEC1;
					state <= COMMAND_RELOAD_TRACKINFO;
				end

				// Setup track offsets in image based on disk
				COMMAND_RW_DATA_EXEC1:
				begin
					m_status[UPD765_MAIN_DIO] <= ~i_write;
					if (i_rtrack) i_r <= 1;
					i_bc <= 1;
					// Read from the track stored at the last seek
					// even if different one is given in the command
					image_track_offsets_addr <= { pcn[ds0], hds };
					buff_wait <= 1;
					state <= COMMAND_RW_DATA_EXEC2;
					//i_current_sector <= 1'd1;
				end

				// Trigger loading track information block for current track
				COMMAND_RW_DATA_EXEC2:				
				if (~sd_busy & ~buff_wait) begin
					i_current_sector <= 1'd1;
					//i_scanning <= 0;
					sd_buff_type <= UPD765_SD_BUFF_TRACKINFO;
					i_seek_pos <= {image_track_offsets_in+1'd1,8'd0}; //TrackInfo+256bytes
					buff_addr <= {image_track_offsets_in[0], 8'h14}; //sector size
					buff_wait <= 1;
					state <= COMMAND_RW_DATA_EXEC3;
				end

				//process trackInfo + sectorInfo and load i_sector variables
				COMMAND_RW_DATA_EXEC3:
				if (~sd_busy & ~buff_wait) begin
					if (buff_addr[7:0] == 8'h14) begin
						if (!image_edsk[ds0]) i_sector_size <= 8'h80 << buff_data_in[2:0];
						buff_addr[7:0] <= 8'h18; //sector info list
						buff_wait <= 1;
				// i_current_sector is the Sector id on the track and they can be out of order in the image (e.g. 1,6,3,9,etc)
				end else if (i_current_sector > i_total_sectors) begin
						m_status[UPD765_MAIN_EXM] <= 0;
						//sector not found or end of track
						status[0] <= i_rtrack ? 8'h00 : 8'h40;
						status[1] <= i_rtrack ? 8'h00 : 8'h04;
						status[2] <= i_rtrack | ~i_bc ? 8'h00 : (i_sector_c == 8'hff ? 8'h02 : 8'h10); //bad/wrong cylinder
						state <= COMMAND_READ_RESULTS;
						int_state[ds0] <= 1'b1;
						phase <= PHASE_RESPONSE;
					end else begin
						//process sector info list
						case (buff_addr[2:0])
							0: i_sector_c <= buff_data_in;
							1: i_sector_h <= buff_data_in;
							2: i_sector_r <= buff_data_in;
							3: i_sector_n <= buff_data_in;
							4: i_sector_st1 <= buff_data_in;
							5: i_sector_st2 <= buff_data_in;
							6: if (image_edsk[ds0]) i_sector_size[7:0] <= buff_data_in;
							7: begin
									// start scanning of the sector IDs from the sector at the current head position
									if (image_edsk[ds0]) i_sector_size[15:8] <= buff_data_in;
									state <= COMMAND_RW_DATA_EXEC4;
								end
						endcase
						buff_addr <= buff_addr + 1'd1;
						buff_wait <= 1;
					end
				end

				// Examine the last found sector and check if we located the right one
				COMMAND_RW_DATA_EXEC4:
				if ((i_rtrack && i_current_sector == i_r) ||
				(~i_rtrack && i_sector_c == i_c && i_sector_r == i_r && i_sector_h == i_h && (i_sector_n == i_n || !i_n))) begin
					//sector found in the sector info list
					if (i_sk & ~i_rtrack & (i_rw_deleted ^ i_sector_st2[6])) begin
						state <= COMMAND_RW_DATA_EXEC8;
					end else begin
						i_bytes_to_read <= i_n ? (8'h80 << (i_n[3] ? 4'h8 : i_n[2:0])) : i_dtl;
						i_timeout <= OVERRUN_TIMEOUT;
						i_weak_sector <= 0;
						state <= COMMAND_RW_DATA_WAIT_SECTOR;
					end
				end else begin
					//try the next sector in the sectorinfo list
					if (i_sector_c == i_c) i_bc <= 0;
					i_current_sector <= i_current_sector + 1'd1;
					i_seek_pos <= i_seek_pos + i_sector_size;
					state <= COMMAND_RW_DATA_EXEC3;
				end

				//wait for the sector needed for positioning at the head - delay only
				COMMAND_RW_DATA_WAIT_SECTOR:
				if ((i_current_sector_pos[ds0][hds] == i_current_sector - 1'd1) && !i_rpm_timer[ds0][hds]) begin
					m_status[UPD765_MAIN_EXM] <= 1;
					state <= COMMAND_RW_DATA_EXEC_WEAK;
				end

				// Copy protection, PCW skips to RW_DATA_EXEC5 (not true for EDSK)
				COMMAND_RW_DATA_EXEC_WEAK:
				if (image_edsk[ds0] &&
					(i_sector_size == { i_bytes_to_read, 1'b0 } || // 2 weak sectors
					(i_sector_size == ({ i_bytes_to_read, 1'b0 } + i_bytes_to_read)) || // 3 weak sectors
					(i_sector_size == { i_bytes_to_read, 2'b00 } ))) begin // 4 weak sectors
					//if sector data == 2,3,4x sector size, then handle multiple version of the same sector (weak sectors)
					//otherwise extra data is considered as GAP data
					if (i_weak_sector != next_weak_sector[ds0]) begin
						i_seek_pos <= i_seek_pos + i_bytes_to_read;
						i_sector_size <= i_sector_size - i_bytes_to_read;
						i_weak_sector <= i_weak_sector + 1'd1;
					end else begin
						next_weak_sector[ds0] <= next_weak_sector[ds0] + 1'd1;
						state <= COMMAND_RW_DATA_EXEC5;
					end
				end else begin
					if (SPECCY_SPEEDLOCK_HACK & 
						i_current_sector == 2 & !pcn[ds0] & ~hds & i_sector_st1[5] & i_sector_st2[5])
						next_weak_sector[ds0] <= next_weak_sector[ds0] + 1'd1;
					else
						next_weak_sector[ds0] <= 0;
//					if (i_bytes_to_read > i_sector_size) i_bytes_to_read <= i_sector_size;
					state <= COMMAND_RW_DATA_EXEC5;
				end

				//Read the LBA for the sector into the RAM
				COMMAND_RW_DATA_EXEC5:
				if (~sd_busy & ~buff_wait) begin
					sd_buff_type <= UPD765_SD_BUFF_SECTOR;
					sd_rd[ds0] <= 1;
					sd_lba <= i_seek_pos[31:9];
					sd_busy <= 1;
					buff_addr <= i_seek_pos[8:0];
					buff_wait <= 1;
					state <= COMMAND_RW_DATA_EXEC6;
				end

				//Read from/write to Speccy
				COMMAND_RW_DATA_EXEC6:
				if (~sd_busy & ~buff_wait) begin
					if (!i_bytes_to_read) begin
						//end of the current sector in buffer, so write it to SD card
						if (i_write && buff_addr && i_seek_pos < image_size[ds0]) begin
							sd_lba <= i_seek_pos[31:9];
							sd_wr[ds0] <= 1;
							sd_busy <= 1;
						end
						state <= COMMAND_RW_DATA_EXEC8;
					end else if (~m_status[UPD765_MAIN_RQM]) begin
						m_status[UPD765_MAIN_RQM] <= 1;	
						if(ndma_mode) int_state[ds0] <= 1'b1;
					end else if (~i_write & ~old_rd & rd & a0) begin
						if (&buff_addr) begin
							//sector continues on the next LBA
							state <= COMMAND_RW_DATA_EXEC5;
						end
						//Speedlock: fuzz 'weak' sectors last bytes
						//weak sector is cyl 0, head 0, sector 2
						m_data <= buff_data_in;

						m_status[UPD765_MAIN_RQM] <= 0;
						if (i_sector_size) begin
							i_sector_size <= i_sector_size - 1'd1;
							buff_addr <= buff_addr + 1'd1;
							buff_wait <= 1;
							i_seek_pos <= i_seek_pos + 1'd1;
						end
						i_bytes_to_read <= i_bytes_to_read - 1'd1;
						i_timeout <= OVERRUN_TIMEOUT;
						if(ndma_mode) int_state[ds0] <= 1'b0;
					end else if (i_write & ~old_wr & wr & a0) begin
						buff_wr <= 1;
						buff_data_out <= din;
						i_timeout <= OVERRUN_TIMEOUT;
						m_status[UPD765_MAIN_RQM] <= 0;
						state <= COMMAND_RW_DATA_EXEC7;
						if(ndma_mode) int_state[ds0] <= 1'b0;
					end else begin
						i_timeout <= i_timeout - 1'd1;
					end
				end

				COMMAND_RW_DATA_EXEC7:
				begin
					buff_wr <= 0;
					if (i_sector_size) begin
						i_sector_size <= i_sector_size - 1'd1;
						buff_addr <= buff_addr + 1'd1;
						buff_wait <= 1;
						i_seek_pos <= i_seek_pos + 1'd1;
					end
					i_bytes_to_read <= i_bytes_to_read - 1'd1;
					if (&buff_addr) begin
						//sector continues on the next LBA
						//so write out the current before reading the next
						if (i_seek_pos < image_size[ds0]) begin
							sd_lba <= i_seek_pos[31:9];
							sd_wr[ds0] <= 1;
							sd_busy <= 1;
						end
						state <= COMMAND_RW_DATA_EXEC5;
					end else begin
						state <= COMMAND_RW_DATA_EXEC6;
					end
				end

				//End of reading/writing sector, what's next?
				COMMAND_RW_DATA_EXEC8:
				if (~sd_busy) begin
					if (~i_rtrack & ~(i_sk & (i_rw_deleted ^ i_sector_st2[6])) &
						((i_sector_st1[5] & i_sector_st2[5]) | (i_rw_deleted ^ i_sector_st2[6]))) begin
						//deleted mark or crc error
						m_status[UPD765_MAIN_EXM] <= 0;
						status[0] <= 8'h40;
						status[1] <= i_sector_st1;
						status[2] <= i_sector_st2 | (i_rw_deleted ? 8'h40 : 8'h0);
						state <= COMMAND_READ_RESULTS;
						int_state[ds0] <= 1'b1;
						phase <= PHASE_RESPONSE;
					end else	if ((i_rtrack ? i_current_sector : i_sector_r) == i_eot) begin
						//end of cylinder.  Has to read all tracks to get here!
						m_status[UPD765_MAIN_EXM] <= 0;
						status[0] <= i_rtrack ? 8'h00 : 8'h00; // was 8'h40 
						status[1] <= 8'h00;	// Was 0x80
						status[2] <= (i_rw_deleted ^ i_sector_st2[6]) ? 8'h40 : 8'h0;
						state <= COMMAND_READ_RESULTS;
						int_state[ds0] <= 1'b1;
						phase <= PHASE_RESPONSE;
						
					end else begin
						//read the next sector (multi-sector transfer)
						if (i_mt & image_sides[ds0]) begin
							hds <= ~hds;
							i_h <= ~i_h;
							image_track_offsets_addr <= { pcn[ds0], ~hds };
							buff_wait <= 1;
						end
						if (~i_mt | hds | ~image_sides[ds0]) i_r <= i_r + 1'd1;
						state <= COMMAND_RW_DATA_EXEC2;
					end
				end

				COMMAND_FORMAT_TRACK:
				begin
					int_state <= '{ 0, 0 };
					if (~old_wr & wr & a0) begin
						ds0 <= din[0];
						state <= COMMAND_FORMAT_TRACK1;
					end
				end

				COMMAND_FORMAT_TRACK1: //doesn't modify the media
				if (~old_wr & wr & a0) begin
					i_n <= din;
					state <= COMMAND_FORMAT_TRACK2;
				end

				COMMAND_FORMAT_TRACK2:
				if (~old_wr & wr & a0) begin
					i_sc <= din;
					state <= COMMAND_FORMAT_TRACK3;
				end

				COMMAND_FORMAT_TRACK3:
				if (~old_wr & wr & a0) begin
					//i_gpl <= din;
					state <= COMMAND_FORMAT_TRACK4;
				end

				COMMAND_FORMAT_TRACK4:
				if (~old_wr & wr & a0) begin
					//i_d <= din;
					m_status[UPD765_MAIN_EXM] <= 1;
					state <= COMMAND_FORMAT_TRACK5;
				end

				COMMAND_FORMAT_TRACK5:
				begin
					phase <= PHASE_EXECUTE;
					if (!i_sc) begin
						m_status[UPD765_MAIN_EXM] <= 0;
						status[0] <= 0;
						status[1] <= 0;
						status[2] <= 0;
						state <= COMMAND_READ_RESULTS;
						int_state[ds0] <= 1'b1;
						phase <= PHASE_RESPONSE;
					end else	if (~old_wr & wr & a0) begin
						i_c <= din;
						state <= COMMAND_FORMAT_TRACK6;
					end
				end

				COMMAND_FORMAT_TRACK6:
				if (~old_wr & wr & a0) begin
					i_h <= image_density[ds0] ? din : 8'b0;
					state <= COMMAND_FORMAT_TRACK7;
				end

				COMMAND_FORMAT_TRACK7:
				if (~old_wr & wr & a0) begin
					i_r <= din;
					state <= COMMAND_FORMAT_TRACK8;
				end

				COMMAND_FORMAT_TRACK8:
				if (~old_wr & wr & a0) begin
					i_n <= din;
					i_sc <= i_sc - 1'd1;
					i_r <= i_r + 1'd1;
					state <= COMMAND_FORMAT_TRACK5;
				end

				COMMAND_SCAN_EQUAL:
				begin
					int_state <= '{ 0, 0 };
					if (~old_wr & wr & a0) begin
						state <= COMMAND_IDLE;
					end
				end

				COMMAND_SCAN_HIGH_OR_EQUAL:
				begin
					int_state <= '{ 0, 0 };
					if (~old_wr & wr & a0) begin
						state <= COMMAND_IDLE;
					end
				end

				COMMAND_SCAN_LOW_OR_EQUAL:
				begin
					int_state <= '{ 0, 0 };
					if (~old_wr & wr & a0) begin
						state <= COMMAND_IDLE;
					end
				end

				COMMAND_SETUP:
				if (!old_wr & wr & a0) begin
					case (i_substate)
						0: begin
								ds0 <= din[0];		// device
								hds <= image_density[din[0]] ? din[2] : 1'b0;		// head polarity
								i_substate <= 1;
							end
						1: begin
								i_c <= din;			// track
								i_substate <= 2;
							end
						2:	begin
								i_h <= image_density[ds0] ? din : 8'b0;			// head
								i_substate <= 3;
							end
						3: begin
								i_r <= din;			// sector
								i_substate <= 4;
							end
						4: begin
								i_n <= din;			// sector len (1 = 256, 2 = 512)
								i_substate <= 5;
							end
						5: begin
								i_eot <= din;		// last sector in track
								i_substate <= 6;
							end
						6:	begin
								//i_gpl <= din;		// gap len (seems to be ignored)
								i_substate <= 7;
							end
						7: begin
								i_dtl <= din;
								i_substate <= 0;
								if (~motor[ds0] | ~ready[ds0] | ~image_ready[ds0]) begin
									status[0] <= 8'h40;
									status[1] <= 8'b101;
									status[2] <= 0;
									state <= COMMAND_READ_RESULTS;
								end else if (hds & ~image_sides[ds0]) begin
									hds <= 0;
									status[0] <= 8'h48; //no side B
									status[1] <= 0;
									status[2] <= 0;
									state <= COMMAND_READ_RESULTS;
								end else begin
									phase <= PHASE_EXECUTE;
									state <= i_command;
								end
							end
					endcase
				end

				COMMAND_READ_RESULTS:
				begin
					phase <= PHASE_RESPONSE;
					m_status[UPD765_MAIN_DIO] <= 1;
					if (~sd_busy & ~buff_wait) begin
						m_status[UPD765_MAIN_RQM] <= 1;
						if (~old_rd & rd & a0) begin
							//m_status[UPD765_MAIN_RQM] <= 0;	// toggle request line for one period to generate interrupt
							case (i_substate)
								0: begin
										//if(read_or_write) int_state[ds0] = 1'b1;
										m_data <= {status[0][7:3], hds, 1'b0, ds0 }; // status[0][7:3]
										i_substate <= 1;
										int_state[ds0] <= 1'b0; 
									end
								1: begin
										m_data <= status[1];
										i_substate <= 2;
									end
								2: begin
										m_data <= status[2];
										i_substate <= 3;
									end
								3: begin
										// Changes based on if TC issued and last sector
										//if(tc && i_sector_r == i_current_track_sectors[ds0][hds]) m_data <= i_sector_c + 8'd1; 
										//else m_data <= i_sector_c;
										m_data <= i_sector_c;
										i_substate <= 4;
									end
								4: begin
										m_data <= i_sector_h;
										i_substate <= 5;
									end
								5: begin
										// Changes based on if TC issued and last sector
										//if(tc && i_sector_r == i_current_track_sectors[ds0][hds]) m_data <= 8'd1;
										//else m_data <= tc ? i_sector_r + 8'd1 : i_sector_r;
										m_data <= i_sector_r; //i_eot + 8'd1;
										i_substate <= 6;
									end
								6: begin
										m_data <= i_sector_n;
										state <= COMMAND_IDLE;
									end
								7: ;//not happen
							endcase
						end
					end else m_status[UPD765_MAIN_RQM] <= 0;

				end

				COMMAND_INVALID:
				begin
					int_state <= '{ 0, 0 };
					m_status[UPD765_MAIN_DIO] <= 1;
//					m_status[UPD765_MAIN_RQM] <= 1;
					status[0] <= 8'h80;
					state <= COMMAND_INVALID1;
				end

				COMMAND_INVALID1:
				if (~old_rd & rd & a0) begin
					state <= COMMAND_IDLE;
					m_data <= status[0];
//					int_state[ds0] <= 1'b1;
				end

				COMMAND_RELOAD_TRACKINFO:
				if (image_ready[ds0] & image_trackinfo_dirty[ds0]) begin
					//i_rpm_timer[ds0] <= '{ 0, 0 };
					next_weak_sector[ds0] <= 0;
					image_track_offsets_addr <= { pcn[ds0], 1'b0 };
					old_hds <= hds;
					hds <= 0;
					buff_wait <= 1;
					state <= COMMAND_RELOAD_TRACKINFO1;
				end else begin
					state <= i_command;
				end

				COMMAND_RELOAD_TRACKINFO1:
				if (~buff_wait & ~sd_busy) begin
					if (image_ready[ds0] && image_track_offsets_in) begin
						sd_buff_type <= UPD765_SD_BUFF_TRACKINFO;
						sd_rd[ds0] <= 1;
						sd_lba <= image_track_offsets_in[15:1];
						sd_busy <= 1;
						state <= COMMAND_RELOAD_TRACKINFO2;
					end else begin
						image_trackinfo_dirty[ds0] <= 0;
						hds <= old_hds;
						state <= i_command;
					end
				end

				COMMAND_RELOAD_TRACKINFO2:
				if (~sd_busy) begin
					buff_addr <= {image_track_offsets_in[0], 8'h15}; //number of sectors
					buff_wait <= 1;
					state <= COMMAND_RELOAD_TRACKINFO3;
				end

				// We now have the track buffer loaded into buff_addr and offset pointing to num sectors in track
				COMMAND_RELOAD_TRACKINFO3:
				if (~sd_busy & ~buff_wait) begin
					i_current_track_sectors[ds0][hds] <= buff_data_in;
					//i_rpm_time[ds0][hds] <= buff_data_in ? TRACK_TIME/buff_data_in : cycles_time;

					//assume the head position is at the middle of a track after a seek
					i_current_sector_pos[ds0][hds] <= buff_data_in[7:1];

					if (hds == image_sides[ds0]) begin
						image_trackinfo_dirty[ds0] <= 0;
						hds <= old_hds;
						state <= i_command; // Exit back to COMMAND_READ_ID2 / COMMAND_RW_DATA_EXEC1
					end else begin //read TrackInfo from the other head if 2 sided
						image_track_offsets_addr <= { pcn[ds0], 1'b1 };
						hds <= 1;
						buff_wait <= 1;
						state <= COMMAND_RELOAD_TRACKINFO1;
					end
				end

			endcase //status
		end
	end
end

endmodule

module u765_dpram #(parameter DATAWIDTH=8, ADDRWIDTH=12)
(
	input	                clock,

	input	[ADDRWIDTH-1:0] address_a,
	input	[DATAWIDTH-1:0] data_a,
	input	                wren_a,
	output reg [DATAWIDTH-1:0] q_a,

	input	[ADDRWIDTH-1:0] address_b,
	input	[DATAWIDTH-1:0] data_b,
	input	                wren_b,
	output reg [DATAWIDTH-1:0] q_b
);

logic [DATAWIDTH-1:0] ram[0:(1<<ADDRWIDTH)-1];

always_ff@(posedge clock) begin
	if(wren_a) begin
		ram[address_a] <= data_a;
		q_a <= data_a;
	end else begin
		q_a <= ram[address_a];
	end
end

always_ff@(posedge clock) begin
	if(wren_b) begin
		ram[address_b] <= data_b;
		q_b <= data_b;
	end else begin
		q_b <= ram[address_b];
	end
end

endmodule
