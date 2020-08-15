//
// Z80 Register Debugger
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

module z80_debugger
(
    input clk_sys,                   // System clock
    input ce,                        // Chip enable
    input m1_n,                      // M1 Instruction fetch signal
    // Signals to / from t80pa core
    input  wire  [211:0] REG_in,     // Z80 Register set as defined in T80pa / T80
    output logic [15:0] PC,          // Program Counter       
    output logic [15:0] SP,          // Stack Pointer
    output logic [7:0]  AC,          // Accumulator
    output logic [15:0] BC,          // BC Register
    output logic [15:0] DE,          // DE Register   
    output logic [15:0] HL,          // HL Register
    output logic [15:0] IX,          // IX Register
    output logic [15:0] IY,          // IY Register
    // Flag enables
    output logic Z,        // Zero
    output logic N,        // Negative
    output logic P,        // Parity / Overflow
    output logic C         // Carry
); 

localparam FLAG_CARRY = 0;
localparam FLAG_PARITY = 2;
localparam FLAG_ZERO = 6;
localparam FLAG_SIGN = 7;

logic [15:0] reg_PC;
logic [15:0] reg_SP;
logic [7:0]  reg_AC;
logic [15:0] reg_BC;
logic [15:0] reg_DE;
logic [15:0] reg_HL;
logic [15:0] reg_IX;
logic [15:0] reg_IY;
logic reg_Z;
logic reg_N;
logic reg_P;
logic reg_C;
logic [7:0] reg_flags;

always @(posedge clk_sys)
begin
    if(ce && ~m1_n) 
    begin
        reg_PC <= REG_in[79:64];  
        reg_SP <= REG_in[63:48];  
        reg_AC <= REG_in[7:0];   
        reg_BC <= REG_in[95:80];  
        reg_DE <= REG_in[111:96]; 
        reg_HL <= REG_in[127:112];
        reg_IX <= REG_in[143:128];
        reg_IY <= REG_in[207:192]; 
        reg_flags <= REG_in[15:8];
    end
end

assign PC = reg_PC;
assign SP = reg_SP; 
assign AC = reg_AC;
assign BC = reg_BC;
assign DE = reg_DE;
assign HL = reg_HL;
assign IX = reg_IX;
assign IY = reg_IY;

assign Z = reg_flags[FLAG_ZERO];
assign N = reg_flags[FLAG_SIGN];
assign P = reg_flags[FLAG_PARITY];
assign C = reg_flags[FLAG_CARRY];

// REG_in bit locations for various flags
// 211-210 - IFF2, IFF1
// 209-208 - IM
// 207-192 - I,Y
// 191-176 - H',L'
// 175-160 - D',E'
// 159-144 - B',C'
// 143-128 - I,X
// 127-112 - H,L
// 111-096 - D,E
// 095-080 - B,C
// 079-064 - PCH, PCL
// 063-048 - SPH, SPL
// 047-040 - R
// 039-032 - I
// 031-024 - Flags'
// 023-016 - ACC'
// 015-008 - Flags
// 007-000 - ACC

/* wire ADDRESS_JOYSTICK = ( CPU_A[7:4]  == 4'b0010 )?1'b1:1'b0;
wire ADDRESS_JOYSTICK_1_A = (ADDRESS_JOYSTICK & (CPU_A[0] == 1'b0))?1'b1:1'b0;
wire [7:0] JOYSTICK_1_DATA_A = { 3'b111, ~fire_1, ~right_1, ~left_1,~down_1,~up_1 };
wire ADDRESS_JOYSTICK_1_B = (ADDRESS_JOYSTICK & (CPU_A[1] == 1'b0))?1'b1:1'b0;
wire [7:0] JOYSTICK_1_DATA_B = { 3'b111, ~arm_1, 4'b1111};
wire ADDRESS_JOYSTICK_2_A = (ADDRESS_JOYSTICK & (CPU_A[2] == 1'b0))?1'b1:1'b0;
wire [7:0] JOYSTICK_2_DATA_A = { 3'b111, ~fire_2, ~right_2, ~left_2,~down_2,~up_2 };
wire ADDRESS_JOYSTICK_2_B = (ADDRESS_JOYSTICK & (CPU_A[3] == 1'b0))?1'b1:1'b0;
wire [7:0] JOYSTICK_2_DATA_B = { 3'b111, ~arm_2, 4'b1111};

wire [7:0] CPU_D_INPORT    =    
    (ADDRESS_JOYSTICK_1_A ? JOYSTICK_1_DATA_A : 'b1) &
    (ADDRESS_JOYSTICK_1_B ? JOYSTICK_1_DATA_B : 'b1) &
    (ADDRESS_JOYSTICK_2_A ? JOYSTICK_2_DATA_A : 'b1) &
    (ADDRESS_JOYSTICK_2_B ? JOYSTICK_2_DATA_B : 'b1); */
                        
endmodule