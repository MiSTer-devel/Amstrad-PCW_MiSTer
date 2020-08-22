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

module amx_mouse
(
	input wire         reset,		// reset when driven high
	input wire         clk_sys,		// should be same clock as clk_sys from HPS_IO
    
    // Inputs from generic mouse module
    input wire signed [8:0] mouse_x,      // Signed 9 bit value (twos complement)
    input wire signed [8:0] mouse_y,
    input wire mouse_left,
    input wire mouse_middle,
    input wire mouse_right,

    input wire sel,                 // Select enable line
    input wire [1:0] addr,          // Address line

    output logic [7:0] dout         // Data output for address
);

reg [7:0] data;
assign dout = sel ? data : 8'hff;

wire signed [5:0] dxt = mouse_x / 6'd8;
wire signed [5:0] dyt = mouse_y / 6'd8;

wire signed [3:0] dx = (dxt > 7) ? 4'd7 : (dxt < -8) ? -4'd8 : dxt[3:0];
wire signed [3:0] dy = (dyt > 7) ? 4'd7 : (dyt < -8) ? -4'd8 : dyt[3:0];

always @(posedge clk_sys)
begin
    logic old_sel;

    old_sel <= sel;
    if(~old_sel & sel) 
    begin
        case(addr)
            2'b00: data <= (dy < 0) ? {(~dy + 1'd1),4'b0} : {4'b0,dy};
            2'b01: data <= (dx >= 0) ? {4'b0,dx} : {(~dx + 1'd1),4'b0};
            2'b10: data <= {5'b1,~mouse_right,~mouse_middle,~mouse_left};
            2'b11: data <= 8'hff;
    endcase
    end
end

endmodule
