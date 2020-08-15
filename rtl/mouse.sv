//
// PCW for MiSTer Mouse emulation module
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

module mouse
(
	input wire         reset,		// reset when driven high
	input wire         clk_sys,		// should be same clock as clk_sys from HPS_IO
    
    // [24] toggle, [23:16] buttons, [15:8] move X, [7:0] move Y
	input wire  [24:0] ps2_mouse,	

    // Buttons and axis output
    output logic mouse_left,
    output logic mouse_middle,
    output logic mouse_right,
    output logic signed [8:0] mouse_x,      // Signed 9 bit value (twos complement)
    output logic signed [8:0] mouse_y
);

logic [23:0] mouse_latch;
logic old_strobe;

assign mouse_left = mouse_latch[0];
assign mouse_middle = mouse_latch[2];
assign mouse_right = mouse_latch[1];

assign mouse_x = {mouse_latch[4],mouse_latch[15:8]};
assign mouse_y = {mouse_latch[5],mouse_latch[23:16]};

// Latch updates to mouse positon.  Bit 24 is input latch
always @(posedge clk_sys)
begin
    old_strobe <= ps2_mouse[24];
    if(reset) mouse_latch <= 24'b0;

    if(old_strobe != ps2_mouse[24])
    begin
        mouse_latch <= ps2_mouse[23:0];
    end
end

endmodule