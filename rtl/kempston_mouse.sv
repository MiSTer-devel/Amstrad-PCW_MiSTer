//
// PCW for MiSTer AMX Mouse emulation module
//
// Copyright (c) 2020 Stephen Eddy
//
// All rights reserved
//
// Redistribution and use in source and synthezised forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// * Redistributions in synthesized form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the author nor the names of other contributors may
//   be used to endorse or promote products derived from this software without
//   specific prior written agreement from the author.
//
// * License is granted for non-commercial use only.  A fee may not be charged
//   for redistributions as source code or in synthesized/hardware form without 
//   specific prior written agreement from the author.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Keyboard and Joystick mapper for Amstrad PCW keyboard matrix

module kempston_mouse
(
	input wire         reset,		// reset when driven high
	input wire         clk_sys,		// should be same clock as clk_sys from HPS_IO
    
    // Inputs from generic mouse module
    input wire signed [8:0] mouse_x,      // Signed 9 bit value (twos complement)
    input wire signed [8:0] mouse_y,
    input wire mouse_left,
    input wire mouse_right,
	input wire input_pulse,

	input        sel,
	input  [2:0] addr,
	output [7:0] dout
);

logic [7:0] data;
assign dout = sel ? data : 8'hff;

function automatic signed [9:0] clamp_delta(input signed [8:0] v);
    if (v > 9'sd63)       clamp_delta = 10'sd63;
    else if (v < -9'sd64) clamp_delta = -10'sd64;
    else                  clamp_delta = {v[8], v};
endfunction

logic [9:0] dxp = 10'd0, dyp = 10'd0;

always @(posedge clk_sys)
begin
	reg p1, p2;
	p1 <= input_pulse;
	p2 <= p1;

	if(reset) begin
		dxp <= 10'd0;
		dyp <= 10'd0;
	end
	else if(p1 != p2) begin
		dxp <= dxp + clamp_delta(mouse_x);
		dyp <= dyp + clamp_delta(mouse_y);
	end
end

// Data drivers
always_comb
begin
	casez(addr)
		3'b0?0: data = dxp[9:2];							// X pos
		3'b0?1: data = dyp[9:2];							// Y pos
		3'b100: data = {6'b1,~mouse_left,~mouse_right};	// Buttons
		default: data = 8'hff;
	endcase
end

endmodule
