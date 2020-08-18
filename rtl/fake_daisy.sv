//
// PCW for MiSTer Fake Daisywheel module
// required to boot 9512+ 3.5" disks
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

module fake_daisy
(
	input wire          reset,		    // reset when driven high
	input wire          clk_sys,		// should be same clock as clk_sys from HPS_IO
    input wire          ce,             // Clock enable line from CPU
    input wire          sel,            // Select signal
    
    // Responds to following PCW address: 00 - FC, 01 - FD, 10 - 01FC
	input wire  [1:0]   address,	    // Address
    input wire          wr,             // 1 = Write, 0 = Read
    input wire  [7:0]   din,
    output logic [7:0]  dout

);

localparam HOST_RECV = 8'b10110001;
localparam HOST_SEND = 8'b10110010;
logic [7:0] status;     // Current status
logic [7:0] command;    // First byte of any command received
logic [7:0] data;       // Second byte of any command received
logic [7:0] out;        // Output from command


// State machine for processing commands written to controller via FC
always @(posedge clk_sys)
begin
    logic old_sel;

    if(reset)
    begin
        status <= HOST_SEND;
        command <= 8'h00;
        data <= 8'h00;
        out <= 8'h00;
        old_sel <= 1'b0;
    end
    else begin
        if(ce) begin
            old_sel <= sel;
            if(~old_sel && sel && wr && address==2'b10) begin  // New command received
                command <= din;   
                status <= HOST_SEND;
            end
            if(~old_sel && sel && wr && address==2'b00) begin  // Data for command recieved, update output
                data <= din;
                // Set output based on command
                if(command==8'h12 || command==8'h02 || command==8'h06 || command==8'h0a || command==8'h0b)
                begin
                    status <= HOST_RECV;
                    out <= 8'h00;
                end
                else begin
                    status <= HOST_SEND;
                    out <= 8'hff;
                end
            end
        end
    end
end

always_comb
begin
    if(sel & ce) begin
        if(~wr && address[0]==1'b0) dout = out;
        else dout = status;
    end else dout = 8'hff;
end

endmodule