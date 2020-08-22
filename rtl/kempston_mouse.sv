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

// Magnitude
logic [7:0] dxm, dym;
assign dxm = mouse_x[8] ? ~mouse_x[7:0] + 8'd1 : mouse_x[7:0];
assign dym = mouse_y[8] ? ~mouse_y[7:0] + 8'd1 : mouse_y[7:0];
// Tracked position
logic [7:0] dxp;
logic [7:0] dyp;

// Update absolute positions on mouse input strobe
always @(posedge clk_sys)
begin
	reg old_pulse;
	old_pulse <= input_pulse;

	if(reset) begin
		dxp <= 8'd0; // dx != dy for better mouse detection
		dyp <= 8'd0;
	end
	// Update positions on new mouse data and wrap around
	else if(old_pulse != input_pulse) begin
		dxp <= mouse_x[8] ? dxp - dxm / 8'd8 : dxp + dxm / 8'd8;
		dyp <= mouse_y[8] ? dyp - dym / 8'd8 : dyp + dym / 8'd8;
    end
end

// Data drivers
always_comb
begin
	casez(addr)
		3'b0?0: data <= dxp; //unsigned'(dxp) & 8'hff; 			// X pos
		3'b0?1: data <= dyp; //unsigned'(dyp) & 8'hff;				// Y pos
		3'b100: data <= {6'b1,~mouse_left,~mouse_right};	// Buttons
		default: data <= 8'hff;
	endcase
end

endmodule
