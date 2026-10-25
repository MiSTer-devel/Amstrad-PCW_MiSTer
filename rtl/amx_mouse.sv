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
    input wire input_pulse,         // ps2_mouse[24], toggles once per packet

    input wire sel,                 // Select enable line
    input wire [1:0] addr,          // Address line

    output logic [7:0] dout         // Data output for address
);

reg [7:0] data;
assign dout = sel ? data : 8'hff;

logic signed [9:0] acc_x = 10'sd0, acc_y = 10'sd0;

function automatic signed [4:0] steps(input signed [9:0] acc);
    logic [9:0] mag;
    mag = acc[9] ? -acc : acc;
    mag = mag >> 1;
    if (mag > 10'd15) mag = 10'd15;
    steps = acc[9] ? -$signed({1'b0, mag[3:0]}) : $signed({1'b0, mag[3:0]});
endfunction

wire signed [4:0] dx = steps(acc_x);
wire signed [4:0] dy = steps(acc_y);
wire [4:0] ndx = -dx;
wire [4:0] ndy = -dy;

function automatic signed [9:0] acc_next(input signed [9:0] acc, input add,
                                         input signed [8:0] delta, input take,
                                         input signed [4:0] step);
    logic signed [11:0] n;
    n = {{2{acc[9]}}, acc};
    if (add)  n = n + {{3{delta[8]}}, delta};
    if (take) n = n - {{6{step[4]}}, step, 1'b0};
    if (n > 12'sd32)       n = 12'sd32;
    else if (n < -12'sd32) n = -12'sd32;
    acc_next = n[9:0];
endfunction

always @(posedge clk_sys)
begin
    logic old_sel;
    logic p1, p2;

    old_sel <= sel;
    p1 <= input_pulse;
    p2 <= p1;

    if (reset) begin
        acc_x <= 10'sd0;
        acc_y <= 10'sd0;
    end else begin
        acc_x <= acc_next(acc_x, p1 ^ p2, mouse_x, ~old_sel & sel && addr == 2'b01, dx);
        acc_y <= acc_next(acc_y, p1 ^ p2, mouse_y, ~old_sel & sel && addr == 2'b00, dy);
    end

    if(~old_sel & sel) 
    begin
        case(addr)
            2'b00: data <= dy[4] ? {ndy[3:0],4'b0} : {4'b0,dy[3:0]};
            2'b01: data <= dx[4] ? {ndx[3:0],4'b0} : {4'b0,dx[3:0]};
            2'b10: data <= {5'b1,~mouse_right,~mouse_middle,~mouse_left};
            2'b11: data <= 8'h00;
    endcase
    end
end

endmodule
