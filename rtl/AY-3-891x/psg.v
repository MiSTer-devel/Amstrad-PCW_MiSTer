//-------------------------------------------------------------------------------------------------
//  AY-3-891x
//  Copyright (C) 2022 Kyp069 <kyp069@gmail.com>
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
//-------------------------------------------------------------------------------------------------
module psg
//-------------------------------------------------------------------------------------------------
(
	input  wire       clock, // main clock (at least twice clock enable)
	input  wire       sel,   // 0 for a further division by two (same a YM2149)
	input  wire       ce,    // clock enable (posedge clock)

	input  wire       reset, // active-low asynchronous reset
	input  wire       bdir,  // bus direction
	input  wire       bc1,   // bus control (bc2 is always 1)
	input  wire[ 7:0] d,     // data in
	output reg [ 7:0] q,     // data out

	output wire[11:0] a,     // audio channel a (12 bit unsigned)
	output wire[11:0] b,     // audio channel b (12 bit unsigned)
	output wire[11:0] c,     // audio channel c (12 bit unsigned)
	output wire[14:0] mix,   // mix of all three channels (14 bit unsigned)

	input  wire[ 7:0] ioad,  // io port a data in
	output wire[ 7:0] ioaq,  // io port a data out

	input  wire[ 7:0] iobd,  // io port b data in
	output wire[ 7:0] iobq   // io port b data out
);
//-------------------------------------------------------------------------------------------------

localparam ADDRMASK = 4'b0000;

// bdir bc1 (bc2 == 1)
// 0    0   idle
// 0    1   rd data
// 1    0   wr data
// 1    1   wr addr

reg[3:0] addr;
always @(posedge clock, negedge reset) 
	if(!reset) addr <= 1'd0;
	else if(ce) if(bdir && bc1 && d[7:4] == ADDRMASK) addr <= d[3:0];

//-------------------------------------------------------------------------------------------------

// R0  PPPP PPPP  channel a tone period  7..0
// R1  ---- PPPP  channel a tone period 11..8
// R2  PPPP PPPP  channel b tone period  7..0
// R3  ---- PPPP  channel b tone period 11..8
// R4  PPPP PPPP  channel c tone period  7..0
// R5  ---- PPPP  channel c tone period 11..8
// R6  ---P PPPP  noise shift period
// R7  I--- ----  io port b in/out control, in when low
//     -I-- ----  io port a in/out control, in when low
//     --C- ----  mix noise with channel c, active low
//     ---B ----  mix noise with channel b, active low
//     ---- A---  mix noise with channel a, active low
//     ---- -C--  enable channel c, active low
//     ---- --B-  enable channel b, active low
//     ---- ---A  enable channel a, active low
// R8  ---M ----  channel a mode, 0=level, 1=envelope
//     ---- LLLL  channel a level
// R9  ---M ----  channel b mode, 0=level, 1=envelope
//     ---- LLLL  channel b level
// R10 ---M ----  channel c mode, 0=level, 1=envelope
//     ---- LLLL  channel c level
// R11 PPPP PPPP  envelope period  7..0
// R12 PPPP PPPP  envelope period 15..8
// R13 ---- C---  envelope shape continue control
//     ---- -A--  envelope shape attack control
//     ---- --A-  envelope shape alternate control
//     ---- ---H  envelope shape hold control
// R14 DDDD DDDD  io port a data
// R15 DDDD DDDD  io port b data

reg[11:0] a_period, b_period, c_period;
reg[ 3:0] a_level, b_level, c_level;
reg[ 7:0] a_data, b_data;
reg[15:0] e_period;
reg[ 4:0] n_period;

reg a_mix_noise, b_mix_noise, c_mix_noise;
reg a_enable, b_enable, c_enable;
reg a_mode, b_mode, c_mode;
reg a_data_io, b_data_io;
reg e_hold, e_alternate, e_attack, e_continue;

always @(posedge clock, negedge reset) 
	if(!reset) begin
		a_data <= 1'd0;
		b_data <= 1'd0;
		a_period <= 1'd0;
		b_period <= 1'd0;
		c_period <= 1'd0;
		e_period <= 1'd0;
		n_period <= 1'd0;
		{ a_mode, a_level } <= 1'd0;
		{ b_mode, b_level } <= 1'd0;
		{ c_mode, c_level } <= 1'd0;
		{ e_continue, e_attack, e_alternate, e_hold } <= 1'd0;
		{ b_data_io, a_data_io, c_mix_noise, b_mix_noise, a_mix_noise, c_enable, b_enable, a_enable } <= 1'd0;
	end
	else if(ce) if(bdir && !bc1)
		case(addr)
			 0: a_period[ 7:0] <= d;
			 1: a_period[11:8] <= d[3:0];
			 2: b_period[ 7:0] <= d;
			 3: b_period[11:8] <= d[3:0];
			 4: c_period[ 7:0] <= d;
			 5: c_period[11:8] <= d[3:0];
			 6: n_period <= d[4:0];
			 7: { b_data_io, a_data_io, c_mix_noise, b_mix_noise, a_mix_noise, c_enable, b_enable, a_enable } <= d;
			 8: { a_mode, a_level } <= d[4:0];
			 9: { b_mode, b_level } <= d[4:0];
			10: { c_mode, c_level } <= d[4:0];
			11: e_period[ 7:0] <= d;
			12: e_period[15:8] <= d;
			13: { e_continue, e_attack, e_alternate, e_hold } <= d[3:0];
			14: a_data <= d;
			15: b_data <= d;
		endcase

always @(*)
	case(addr)
		 0: q = a_period[ 7:0];
		 1: q = { 4'd0, a_period[11:8] };
		 2: q = b_period[ 7:0];
		 3: q = { 4'd0, b_period[11:8] };
		 4: q = c_period[ 7:0];
		 5: q = { 4'd0, c_period[11:8] };
		 6: q = { 3'd0, n_period };
		 7: q = { b_data_io, a_data_io, c_mix_noise, b_mix_noise, a_mix_noise, c_enable, b_enable, a_enable };
		 8: q = { 3'd0, a_mode, a_level };
		 9: q = { 3'd0, b_mode, b_level };
		10: q = { 3'd0, c_mode, c_level };
		11: q = e_period[ 7:0];
		12: q = e_period[15:8];
		13: q = { e_continue, e_attack, e_alternate, e_hold };
		14: q = a_data_io ? a_data : ioad;
		15: q = b_data_io ? b_data : iobd;
	endcase

//-------------------------------------------------------------------------------------------------

reg[3:0] cc;
wire stb = sel ? &cc[2:0] : &cc[3:0];
always @(negedge clock, negedge reset)
	if(!reset) cc <= 1'd0;
	else if(ce) cc <= cc+1'd1;

//-------------------------------------------------------------------------------------------------

reg[11:0] a_count;
wire a_count_ge = a_count >= a_period;
always @(posedge clock, negedge reset)
	if(!reset) a_count <= 12'd1;
	else if(ce) if(stb) if(a_count_ge) a_count <= 12'd1; else a_count <= a_count+1'd1;

reg a_ff;
always @(posedge clock, negedge reset)
	if(!reset) a_ff <= 1'b1;
	else if(ce) if(stb) if(a_count_ge) a_ff <= ~a_ff;

//-------------------------------------------------------------------------------------------------

reg[11:0] b_count;
wire b_count_ge = b_count >= b_period;
always @(posedge clock, negedge reset)
	if(!reset) b_count <= 12'd1;
	else if(ce) if(stb) if(b_count_ge) b_count <= 12'd1; else b_count <= b_count+1'd1;

reg b_ff;
always @(posedge clock, negedge reset)
	if(!reset) b_ff <= 1'b1;
	else if(ce) if(stb) if(b_count_ge) b_ff <= ~b_ff;

//-------------------------------------------------------------------------------------------------

reg[11:0] c_count;
wire c_count_ge = c_count >= c_period;
always @(posedge clock, negedge reset)
	if(!reset) c_count <= 12'd1;
	else if(ce) if(stb) if(c_count_ge) c_count <= 12'd1; else c_count <= c_count+1'd1;

reg c_ff;
always @(posedge clock, negedge reset)
	if(!reset) c_ff <= 1'b1;
	else if(ce) if(stb) if(c_count_ge) c_ff <= ~c_ff;

//-------------------------------------------------------------------------------------------------

reg  r13wd, r13wp;
wire e_reset = !reset || r13wp;
wire r13w = bdir && !bc1 && addr == 13;
always @(posedge clock, negedge reset)
	if(!reset) { r13wd, r13wp } <= 1'd0;
	else if(ce) begin r13wd <= r13w; r13wp <= r13w && !r13wd; end

reg[15:0] e_count;
wire e_count_ge = e_count >= e_period;
always @(posedge clock, posedge e_reset)
	if(e_reset) e_count <= 1'd1;
	else if(ce) if(stb) if(e_count_ge) e_count <= 1'd1; else e_count <= e_count+1'd1;

reg e_ff;
always @(posedge clock, posedge e_reset)
	if(e_reset) e_ff <= 1'b1;
	else if(ce) if(stb) if(e_count_ge) e_ff <= ~e_ff;

reg e_ff_p;
always @(posedge clock, posedge e_reset)
	if(e_reset) e_ff_p <= 1'b0;
	else if(ce) if(stb) begin e_ff_p <= 1'b0; if(e_count_ge) if(!e_ff) e_ff_p <= 1'b1; end

reg v_continue;
always @(posedge clock, posedge e_reset)
	if(e_reset) v_continue <= 1'b1;
	else if(ce) if(stb) if(e_ff_p) if(&sc) v_continue <= e_continue;

reg v_attack;
always @(posedge clock, posedge e_reset)
	if(e_reset) v_attack <= 1'b0;
	else if(ce) if(stb) if(e_ff_p) if(&sc && e_alternate && !v_hold) v_attack <= ~v_attack;

reg v_hold;
always @(posedge clock, posedge e_reset)
	if(e_reset) v_hold <= 1'b0;
	else if(ce) if(stb) if(e_ff_p) if(&sc) v_hold <= e_hold;

reg[3:0] sc;
always @(posedge clock, posedge e_reset)
	if(e_reset) sc <= 1'd0;
	else if(ce) if(stb) if(e_ff_p) if((&sc && !e_hold) || (!(&sc) && !v_hold)) sc <= sc+1'd1;

wire env_s = e_attack ? ~v_attack : v_attack;
wire[3:0] e_level = !v_continue ? 1'd0 : !env_s ? ~sc : sc;

//-------------------------------------------------------------------------------------------------

reg[4:0] n_count;
wire n_count_ge = n_count >= n_period;
always @(posedge clock, negedge reset)
	if(!reset) n_count <= 1'd1;
	else if(ce) if(stb) if(n_count_ge) n_count <= 1'd1; else n_count <= n_count+1'd1;

reg n_ff;
always @(posedge clock, negedge reset)
	if(!reset) n_ff <= 1'b1;
	else if(ce) if(stb) if(n_count_ge) n_ff <= ~n_ff;

reg n_ff_p;
always @(posedge clock, negedge reset)
	if(!reset) n_ff_p <= 1'b0;
	else if(ce) if(stb) begin n_ff_p <= 1'b0; if(n_count_ge) if(!n_ff) n_ff_p <= 1'b1; end

reg[16:0] n_lfsr;
wire n_bit = n_lfsr[0];
always @(posedge clock, negedge reset)
	if(!reset) n_lfsr <= 17'b1_0000_0000_0000_0000;
	else if(ce) if(stb) if(n_ff_p) n_lfsr <= { n_lfsr[3]^n_lfsr[0], n_lfsr[16:1] };

//-------------------------------------------------------------------------------------------------

reg[11:0] dac[0:15]; // -3 dB steps, value = 4095*10^(-3*(15-N)/20)
initial begin
//	dac[ 0] = 12'h017;
	dac[ 0] = 12'h000; dac[ 1] = 12'h021; dac[ 2] = 12'h02E; dac[ 3] = 12'h041;
	dac[ 4] = 12'h05C; dac[ 5] = 12'h081; dac[ 6] = 12'h0B7; dac[ 7] = 12'h102;
	dac[ 8] = 12'h16D; dac[ 9] = 12'h204; dac[10] = 12'h2D8; dac[11] = 12'h405;
	dac[12] = 12'h5AD; dac[13] = 12'h804; dac[14] = 12'hB53; dac[15] = 12'hFFF;
end

//-------------------------------------------------------------------------------------------------

assign a = (a_enable | a_ff) & (a_mix_noise | n_bit) ? dac[a_mode ? e_level : a_level] : 1'd0;
assign b = (b_enable | b_ff) & (b_mix_noise | n_bit) ? dac[b_mode ? e_level : b_level] : 1'd0;
assign c = (c_enable | c_ff) & (c_mix_noise | n_bit) ? dac[c_mode ? e_level : c_level] : 1'd0;

assign ioaq = a_data;
assign iobq = b_data;

assign mix = { 3'd0, a }+{ 3'd0, b }+{ 3'd0, c };

//-------------------------------------------------------------------------------------------------
endmodule
//-------------------------------------------------------------------------------------------------
