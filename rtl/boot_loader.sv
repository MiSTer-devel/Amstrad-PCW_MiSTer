//
// Boot Loader for PCW_MiSTer
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

module boot_loader(
    input wire [8:0] address,   // Address to load data from
    output logic [7:0] data     // Data returned
);
localparam LENGTH = 275;

assign data = mem[address];

logic [7:0] mem [0:274];
// Setup bootroom code to be transferred into address 0 at start.  Length 275 bytes
initial begin
    mem = '{default:'0};
    mem  = '{8'hC3, 8'h02, 8'h01, 8'hF3, 8'h83, 8'hED, 8'h41, 8'h0D, 
             8'h78, 8'h05, 8'h87, 8'h20, 8'hF8, 8'h31, 8'hF0, 8'hFF, 
             8'h3E, 8'h09, 8'hD3, 8'hF8, 8'h11, 8'h32, 8'h07, 8'h06, 
             8'hC8, 8'hDC, 8'hB1, 8'h00, 8'hCD, 8'h84, 8'h00, 8'h1D, 
             8'hF2, 8'h17, 8'h00, 8'h3E, 8'h80, 8'hD3, 8'hF7, 8'h0E, 
             8'h09, 8'h0C, 8'h79, 8'hD3, 8'hF8, 8'h06, 8'h21, 8'hCD, 
             8'hB1, 8'h00, 8'hCB, 8'h51, 8'h20, 8'hF1, 8'h15, 8'h20, 
             8'hF0, 8'h21, 8'hF5, 8'hFF, 8'h77, 8'hCB, 8'h7E, 8'h28, 
             8'hFC, 8'h3C, 8'hD3, 8'hF8, 8'h1D, 8'h1D, 8'h3E, 8'h06, 
             8'hD3, 8'hF8, 8'hCD, 8'hE4, 8'h00, 8'h09, 8'h66, 8'h00, 
             8'h00, 8'h00, 8'h01, 8'h02, 8'h01, 8'h2A, 8'hFF, 8'h21, 
             8'h00, 8'hF0, 8'hDB, 8'h00, 8'h87, 8'h30, 8'hFB, 8'h87, 
             8'hF2, 8'h6B, 8'h00, 8'hED, 8'hA2, 8'h20, 8'hF3, 8'h7C, 
             8'h1F, 8'h38, 8'hEF, 8'h3E, 8'h05, 8'hD3, 8'hF8, 8'hCD, 
             8'hC7, 8'h00, 8'hE6, 8'hCB, 8'hC0, 8'h47, 8'h21, 8'h10, 
             8'hF0, 8'h24, 8'h86, 8'h25, 8'h86, 8'h2C, 8'h10, 8'hF9, 
             8'h3C, 8'h20, 8'hA0, 8'hE9, 8'h0E, 8'h80, 8'hCD, 8'hDB, 
             8'h00, 8'h05, 8'h03, 8'h0F, 8'hFF, 8'h07, 8'h00, 8'hCD, 
             8'hA5, 8'h00, 8'h06, 8'hC8, 8'h38, 8'h1B, 8'hCD, 8'h44, 
             8'h00, 8'hCD, 8'h44, 8'h00, 8'h0E, 8'h00, 8'hCD, 8'hDB, 
             8'h00, 8'h03, 8'h0F, 8'h00, 8'h14, 8'hCD, 8'hBD, 8'h00, 
             8'h30, 8'hFB, 8'h17, 8'h38, 8'hF8, 8'h17, 8'hD8, 8'h06, 
             8'h14, 8'h3E, 8'hB3, 8'hE3, 8'hE3, 8'hE3, 8'hE3, 8'h3D, 
             8'h20, 8'hF9, 8'h10, 8'hF5, 8'hC9, 8'hDB, 8'hF8, 8'hE6, 
             8'h20, 8'hC8, 8'hCD, 8'hE9, 8'h00, 8'h01, 8'h08, 8'h21, 
             8'h02, 8'h01, 8'hDB, 8'h00, 8'h87, 8'h30, 8'hFB, 8'h3A, 
             8'h02, 8'h01, 8'hF0, 8'hED, 8'hA2, 8'hE3, 8'hE3, 8'hE3, 
             8'hE3, 8'h18, 8'hEF, 8'hDB, 8'hF8, 8'hE6, 8'h40, 8'h28, 
             8'hFA, 8'h79, 8'hD3, 8'hF7, 8'hCD, 8'hBD, 8'h00, 8'h38, 
             8'hFB, 8'hE3, 8'h46, 8'h23, 8'hE3, 8'h0E, 8'h01, 8'hE3, 
             8'hDB, 8'h00, 8'h87, 8'h30, 8'hFB, 8'hFA, 8'hFB, 8'h00, 
             8'h7E, 8'hED, 8'h79, 8'h23, 8'hE3, 8'hE3, 8'hE3, 8'h10, 
             8'hEE, 8'hC9, 8'hAF, 8'hD3, 8'hF0, 8'h01, 8'h00, 8'h00, 
             8'h3E, 8'hD3, 8'h02, 8'h03, 8'h3E, 8'hF8, 8'h02, 8'hAF, 
             8'hC3, 8'h00, 8'h00};
end

endmodule