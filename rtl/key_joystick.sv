//
// PCW for MiSTer Keyboard, Keymouse and Keyjoy module
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

// Keyboard, Keymouse and Joystick mapper for Amstrad PCW keyboard matrix

module key_joystick
(
	input wire         reset,		// reset when driven high
	input wire         clk_sys,		// should be same clock as clk_sys from HPS_IO

	input wire  [10:0] ps2_key,		// [7:0] - scancode,
									// [8]   - extended (i.e. preceded by scan 0xE0),
									// [9]   - pressed
									// [10]  - toggles with every press/release
	input wire   [7:0] joy0,        // Joystick 0
	input wire   [7:0] joy1,        // Joystick 1
	input wire 		   keymouse,	  // Keymouse in use

	// Mouse functions
	input wire		   mouse_left,
	input wire		   mouse_middle,
	input wire		   mouse_right,
	input wire signed [8:0] mouse_x,
	input wire signed [8:0] mouse_y,
	input wire mouse_pulse,

    input wire         lk1,         // Link 1 on motherboard - Doesn't do selftest if 1
    input wire         lk2,         // Link 2 on motherboard
    input wire         lk3,         // Link 3 on motherboard
	input wire   [3:0] addr,		// Address lines for keyboard.  Mapped into 3FF0-3FFB in bank 3
	output logic [7:0] key_data, 	// data lines returned from scanning keyboard and joystick
	// Colour line control signals frm F9/F10/F11 keys
	output logic 	  line_up,	
	output logic      line_down, 
	output logic	  toggle_full	
);

localparam REPEAT_TIME = 64000000 / 150;  // Repeat time for F9 and F10 keys (10 per second)
logic		 up_key;
logic		 down_key;
integer		 repeat_count;

logic  [7:0] keys[15:0];
logic        pressed = 0;
logic  [7:0] code;
logic		 extended = 0;
logic        shifted = 0;
logic        capslock = 0;
logic        shiftstate;
logic caps_pressed_prev = 0; // Para detectar transiciones
logic extra_state = 1'b1;      // Estado de la tecla EXTRA (comienza activo)
logic extra_pressed_prev = 0;  // Para detectar transiciones

assign shiftstate = capslock | shifted;

// Magnitude
logic [6:0] dxm, dym;
assign dxm = mouse_x[8] ? ~mouse_x[6:0] + 7'd1 : mouse_x[6:0];
assign dym = mouse_y[8] ? ~mouse_y[6:0] + 7'd1 : mouse_y[6:0];
// Tracked position
logic [6:0] dxp;
logic [6:0] dyp;

// Update absolute positions on mouse input strobe
always @(posedge clk_sys)
begin
	reg old_pulse;
	old_pulse <= mouse_pulse;

	if(reset) begin
	   
		dxp <= 7'd0; // dx != dy for better mouse detection
		dyp <= 7'd0;
	end
	// Update positions on new mouse data and wrap around
	else if(old_pulse != mouse_pulse) begin
		dxp <= mouse_x[8] ? dxp - dxm / 7'd8 : dxp + dxm / 7'd8;
		dyp <= mouse_y[8] ? dyp - dym / 7'd8 : dyp + dym / 7'd8;
    end
end

// Output row address for keyboard.  KP=Keypad, Jn=Joystick
always_comb
begin
	case(addr)
		4'h0: key_data = keys[0];	// 3FF0 - KP2,KP3,KP6,KP9,Paste,F1/F2,KP0,F3/F4
		4'h1: key_data = keys[1];	// 3FF1 - KP1,KP5,KP4,KP8,Copy,Cut,PTR,Exit
		4'h2: key_data = keys[2];	// 3FF2 - +,Half,Shift(Both),KP7,>,Return,],Del->
		4'h3: key_data = keys[3];	// 3FF3 - .,?,;,<,P,[,-,=]
		4'h4: key_data = keys[4];	// 3FF4 - ',',M,K,L,I,O,9,0
		4'h5: key_data = keys[5];	// 3FF5 - Space,N,J,H,Y,U,7,8
		4'h6: key_data = keys[6];   // 3FF6 - V,B,F,G,T,R,S,6
		4'h7: key_data = keys[7];	// 3FF7 - X,C,D,S,W,E,3,4
		4'h8: key_data = keys[8];	// 3FF8 - Z,Shf-Lock,A,Tab,Q,Stop,2,1
		4'h9: key_data = keys[9];	// 3FF9 - <-Del,NA,J1:F1,J1:F2,J1:R,J1:L,J1:D,J1:U
		4'ha: key_data = keys[10];	// 3FFA - Alt, KP.,KP_Enter,F7/F8,[-],Cancel,Extra,F5/F6
		// 3FFB - NA,NA,J2:F1,J2:F2,J2:R,J2:L,J2:D,J2:U
		4'hb: key_data = ~keymouse ? keys[11] : {mouse_middle, dxp};
		// 3FFC - Mixed. See logic below
		4'hc: key_data = ~keymouse ? keys[12] : {dyp[6:5],keys[12][5:0]};
		// 3FFD - Mixed. See logic below
		4'hd: key_data = ~keymouse ? keys[13] : {keys[13][7:5],dyp[4:0]};
		// 3FFE - Mixed. See logic below
		4'he: key_data = ~keymouse ? keys[14] : {mouse_left,mouse_right,keys[14][5:0]};
		4'hf: key_data = keys[15];	// 3FFF - Mixed. See logic below
	endcase
end

// Detect new input and update latches
reg  input_strobe = 'b0;
always @(posedge clk_sys) begin
	reg old_state;

	input_strobe <= 'b0;
	old_state <= ps2_key[10];
    if(reset) begin
        pressed <= 1'b0;
        extended <= 1'b0;
        input_strobe <= 1'b0;
        old_state <= 1'b0;
        code <= 8'b0;
    end
	if(old_state != ps2_key[10]) begin
		pressed <= ps2_key[9];
        extended <= ps2_key[8];
		code <= ps2_key[7:0];
		input_strobe <= 'b1;
	end
end

// Process keyboard input with combined case for code, pressed, and extended state
always @(posedge clk_sys) begin
    // Clear all key states on a reset
	if(reset) begin
		keys[0]  <= 8'b00000000;
		keys[1]  <= 8'b00000000;
		keys[2]  <= 8'b00000000;
		keys[3]  <= 8'b00000000;
		keys[4]  <= 8'b00000000;
		keys[5]  <= 8'b00000000;
		keys[6]  <= 8'b00000000;
		keys[7]  <= 8'b00000000;
		keys[8]  <= 8'b00000000;
		keys[9]  <= 8'b00000000;
		keys[10] <= 8'b00000000;
		keys[11] <= 8'b00000000;
		keys[12] <= 8'b00000000;
		keys[13] <= 8'b00000000;
		keys[14] <= 8'b00000000;
		keys[15] <= 8'b00000000;
		line_up <= 1'b0;
		line_down <= 1'b0;
	end

    // Main keyboard processing using combined code, pressed, and extended
	if(input_strobe) begin
        // Use a combined case statement for {code, pressed, extended}
        casez({code, pressed, extended})
            // LEFT SHIFT key handling
            {8'h12, 1'b1, 1'b0}: begin 
                keys[2][5] <= 1'b1;   // LEFT SHIFT (PC)
                shifted    <= 1'b1;
            end
            {8'h12, 1'b0, 1'b0}: begin 
                keys[2][5] <= 1'b0;   // LEFT SHIFT (PC) released
                shifted    <= 1'b0;
            end
            // PRT SCR key handling (extended LEFT SHIFT)
            {8'h12, 1'b1, 1'b1}: begin 
                keys[1][1] <= 1'b1;   // PTR (PCW)
            end
            {8'h12, 1'b0, 1'b1}: begin 
                keys[1][1] <= 1'b0;   // PTR (PCW) released
            end
            
            // RIGHT SHIFT key handling
            {8'h59, 1'b1, 1'bz}: begin
                keys[2][5]   <= 1'b1; // RIGHT SHIFT (PC)
                shifted      <= 1'b1;
                keys[12][5]  <= lk2;  // For 3FFC
                keys[14][5]  <= lk2;  // For 3FFE
                keys[15][5]  <= lk2;  // For 3FFF
            end
            {8'h59, 1'b0, 1'bz}: begin
                keys[2][5]   <= 1'b0; // RIGHT SHIFT (PC) released
                shifted      <= 1'b0;
                keys[12][5]  <= 1'b0; // For 3FFC
                keys[14][5]  <= 1'b0; // For 3FFE
                keys[15][5]  <= 1'b0; // For 3FFF
            end
            
            // ALT key handling
            {8'h14, 1'b1, 1'bz}: keys[10][7] <= 1'b1;
            {8'h14, 1'b0, 1'bz}: keys[10][7] <= 1'b0;
            
            // CTRL (PC) -> EXTRA (PCW)
             {8'h11, 1'b1, 1'bz}: keys[10][1] <= 1'b1;
             {8'h11, 1'b0, 1'bz}: keys[10][1] <= 1'b0;
            
            // F1 (PC) -> F1/F2 (PCW)
            {8'h05, 1'b1, 1'bz}: begin
                keys[0][2]  <= 1'b1;
                keys[12][1] <= 1'b1; // For 3FFC
            end
            {8'h05, 1'b0, 1'bz}: begin
                keys[0][2]  <= 1'b0;
                keys[12][1] <= 1'b0; // For 3FFC
            end
            
            // F2 (PC) -> F1/F2 (PCW) with shift
            {8'h06, 1'b1, 1'bz}: begin
                keys[0][2]  <= 1'b1;
                keys[2][5]  <= 1'b1; // LEFT SHIFT
                shifted     <= 1'b1;
            end
            {8'h06, 1'b0, 1'bz}: begin
                keys[0][2]  <= 1'b0;
                keys[2][5]  <= 1'b0; // LEFT SHIFT released
                shifted     <= 1'b0;
            end
            
            // F3 (PC) -> F3/F4 (PCW)
            {8'h04, 1'b1, 1'bz}: begin
                keys[0][0]  <= 1'b1;
                keys[12][0] <= 1'b1; // For 3FFC
            end
            {8'h04, 1'b0, 1'bz}: begin
                keys[0][0]  <= 1'b0;
                keys[12][0] <= 1'b0; // For 3FFC
            end
            
            // F4 (PC) -> F3/F4 (PCW) with shift
            {8'h0C, 1'b1, 1'bz}: begin
                keys[0][0]  <= 1'b1;
                keys[2][5]  <= 1'b1; // LEFT SHIFT
                shifted     <= 1'b1;
            end
            {8'h0C, 1'b0, 1'bz}: begin
                keys[0][0]  <= 1'b0;
                keys[2][5]  <= 1'b0; // LEFT SHIFT released
                shifted     <= 1'b0;
            end
            
            // F5 (PC) -> F5/F6 (PCW)
            {8'h03, 1'b1, 1'bz}: keys[10][0] <= 1'b1;
            {8'h03, 1'b0, 1'bz}: keys[10][0] <= 1'b0;
            
            // F6 (PC) -> F5/F6 (PCW) with shift
            {8'h0B, 1'b1, 1'bz}: begin
                keys[10][0] <= 1'b1;
                keys[2][5]  <= 1'b1; // LEFT SHIFT
                shifted     <= 1'b1;
            end
            {8'h0B, 1'b0, 1'bz}: begin
                keys[10][0] <= 1'b0;
                keys[2][5]  <= 1'b0; // LEFT SHIFT released
                shifted     <= 1'b0;
            end
            
            // F7 (PC) -> F7/F8 (PCW)
            {8'h83, 1'b1, 1'bz}: keys[10][4] <= 1'b1;
            {8'h83, 1'b0, 1'bz}: keys[10][4] <= 1'b0;
            
            // F8 (PC) -> F7/F8 (PCW) with shift
            {8'h0A, 1'b1, 1'bz}: begin
                keys[10][4] <= 1'b1;
                keys[2][5]  <= 1'b1; // LEFT SHIFT
                shifted     <= 1'b1;
            end
            {8'h0A, 1'b0, 1'bz}: begin
                keys[10][4] <= 1'b0;
                keys[2][5]  <= 1'b0; // LEFT SHIFT released
                shifted     <= 1'b0;
            end
            
            // Alphabetic keys
            {8'h1c, 1'b1, 1'bz}: begin 
                keys[8][5]  <= 1'b1; // A
                keys[12][2] <= lk2 ? 1'b1 : keys[12][2]; // For 3FFC
                keys[14][0] <= 1'b1; // For 3FFE
                keys[15][2] <= 1'b1; // For 3FFF
            end
            {8'h1c, 1'b0, 1'bz}: begin 
                keys[8][5]  <= 1'b0; // A
                keys[12][2] <= lk2 ? 1'b0 : keys[12][2]; // For 3FFC
                keys[14][0] <= 1'b0; // For 3FFE
                keys[15][2] <= 1'b0; // For 3FFF
            end
            
            {8'h32, 1'b1, 1'bz}: begin 
                keys[6][6]  <= 1'b1; // B
                keys[14][1] <= 1'b1; // For 3FFE
                keys[15][1] <= 1'b1; // For 3FFF
            end
            {8'h32, 1'b0, 1'bz}: begin 
                keys[6][6]  <= 1'b0; // B
                keys[14][1] <= 1'b0; // For 3FFE
                keys[15][1] <= 1'b0; // For 3FFF
            end
            
            {8'h21, 1'b1, 1'bz}: begin 
                keys[7][6]  <= 1'b1; // C
                keys[14][1] <= 1'b1; // For 3FFE
                keys[15][2] <= 1'b1; // For 3FFF
            end
            {8'h21, 1'b0, 1'bz}: begin 
                keys[7][6]  <= 1'b0; // C
                keys[14][1] <= 1'b0; // For 3FFE
                keys[15][2] <= 1'b0; // For 3FFF
            end
            
            {8'h23, 1'b1, 1'bz}: begin 
                keys[7][5]  <= 1'b1; // D
                keys[12][3] <= lk2 ? 1'b1 : keys[12][3]; // For 3FFC
                keys[14][2] <= 1'b1; // For 3FFE
                keys[15][2] <= 1'b1; // For 3FFF
            end
            {8'h23, 1'b0, 1'bz}: begin 
                keys[7][5]  <= 1'b0; // D
                keys[12][3] <= lk2 ? 1'b0 : keys[12][3]; // For 3FFC
                keys[14][2] <= 1'b0; // For 3FFE
                keys[15][2] <= 1'b0; // For 3FFF
            end
            
            {8'h24, 1'b1, 1'bz}: begin 
                keys[7][2]  <= 1'b1; // E
                keys[14][2] <= 1'b1; // For 3FFE
                keys[15][2] <= 1'b1; // For 3FFF
            end
            {8'h24, 1'b0, 1'bz}: begin 
                keys[7][2]  <= 1'b0; // E
                keys[14][2] <= 1'b0; // For 3FFE
                keys[15][2] <= 1'b0; // For 3FFF
            end
            
            {8'h2b, 1'b1, 1'bz}: begin 
                keys[6][5]  <= 1'b1; // F
                keys[14][0] <= 1'b1; // For 3FFE
                keys[15][3] <= 1'b1; // For 3FFF
            end
            {8'h2b, 1'b0, 1'bz}: begin 
                keys[6][5]  <= 1'b0; // F
                keys[14][0] <= 1'b0; // For 3FFE
                keys[15][3] <= 1'b0; // For 3FFF
            end
            
            {8'h34, 1'b1, 1'bz}: begin 
                keys[6][4]  <= 1'b1; // G
                keys[14][0] <= 1'b1; // For 3FFE
                keys[15][0] <= 1'b1; // For 3FFF
            end
            {8'h34, 1'b0, 1'bz}: begin 
                keys[6][4]  <= 1'b0; // G
                keys[14][0] <= 1'b0; // For 3FFE
                keys[15][0] <= 1'b0; // For 3FFF
            end
            
            {8'h33, 1'b1, 1'bz}: begin 
                keys[5][4]  <= 1'b1; // H
                keys[14][0] <= 1'b1; // For 3FFE
                keys[15][0] <= 1'b1; // For 3FFF
            end
            {8'h33, 1'b0, 1'bz}: begin 
                keys[5][4]  <= 1'b0; // H
                keys[14][0] <= 1'b0; // For 3FFE
                keys[15][0] <= 1'b0; // For 3FFF
            end
            
            {8'h43, 1'b1, 1'bz}: keys[4][3] <= 1'b1; // I
            {8'h43, 1'b0, 1'bz}: keys[4][3] <= 1'b0; // I
            
            {8'h3b, 1'b1, 1'bz}: begin 
                keys[5][5]  <= 1'b1; // J
                keys[14][0] <= 1'b1; // For 3FFE
                keys[15][0] <= 1'b1; // For 3FFF
            end
            {8'h3b, 1'b0, 1'bz}: begin 
                keys[5][5]  <= 1'b0; // J
                keys[14][0] <= 1'b0; // For 3FFE
                keys[15][0] <= 1'b0; // For 3FFF
            end
            
            {8'h42, 1'b1, 1'bz}: begin 
                keys[4][5]  <= 1'b1; // K
                keys[15][0] <= 1'b1; // For 3FFF
            end
            {8'h42, 1'b0, 1'bz}: begin 
                keys[4][5]  <= 1'b0; // K
                keys[15][0] <= 1'b0; // For 3FFF
            end
            
            {8'h4b, 1'b1, 1'bz}: begin 
                keys[4][4]  <= 1'b1; // L
                keys[14][2] <= 1'b1; // For 3FFE
                keys[15][0] <= 1'b1; // For 3FFF
            end
            {8'h4b, 1'b0, 1'bz}: begin 
                keys[4][4]  <= 1'b0; // L
                keys[14][2] <= 1'b0; // For 3FFE
                keys[15][0] <= 1'b0; // For 3FFF
            end
            
            {8'h3a, 1'b1, 1'bz}: begin 
                keys[4][6]  <= 1'b1; // M
                keys[14][1] <= 1'b1; // For 3FFE
                keys[15][1] <= 1'b1; // For 3FFF
            end
            {8'h3a, 1'b0, 1'bz}: begin 
                keys[4][6]  <= 1'b0; // M
                keys[14][1] <= 1'b0; // For 3FFE
                keys[15][1] <= 1'b0; // For 3FFF
            end
            
            {8'h31, 1'b1, 1'bz}: begin 
                keys[5][6]  <= 1'b1; // N
                keys[14][1] <= 1'b1; // For 3FFE
                keys[15][1] <= 1'b1; // For 3FFF
            end
            {8'h31, 1'b0, 1'bz}: begin 
                keys[5][6]  <= 1'b0; // N
                keys[14][1] <= 1'b0; // For 3FFE
                keys[15][1] <= 1'b0; // For 3FFF
            end
            
            {8'h44, 1'b1, 1'bz}: begin 
                keys[4][2]  <= 1'b1; // O
                keys[14][2] <= 1'b1; // For 3FFE
                keys[15][2] <= 1'b1; // For 3FFF
            end
            {8'h44, 1'b0, 1'bz}: begin 
                keys[4][2]  <= 1'b0; // O
                keys[14][2] <= 1'b0; // For 3FFE
                keys[15][2] <= 1'b0; // For 3FFF
            end
            
            {8'h4d, 1'b1, 1'bz}: begin 
                keys[3][3]  <= 1'b1; // P
                keys[14][3] <= 1'b1; // For 3FFE
                keys[15][3] <= 1'b1; // For 3FFF
            end
            {8'h4d, 1'b0, 1'bz}: begin 
                keys[3][3]  <= 1'b0; // P
                keys[14][3] <= 1'b0; // For 3FFE
                keys[15][3] <= 1'b0; // For 3FFF
            end
            
            {8'h15, 1'b1, 1'bz}: begin 
                keys[8][3]  <= 1'b1; // Q
                keys[14][2] <= 1'b1; // For 3FFE
                keys[15][2] <= 1'b1; // For 3FFF
            end
            {8'h15, 1'b0, 1'bz}: begin 
                keys[8][3]  <= 1'b0; // Q
                keys[14][2] <= 1'b0; // For 3FFE
                keys[15][2] <= 1'b0; // For 3FFF
            end
            
            {8'h2d, 1'b1, 1'bz}: begin 
                keys[6][2]  <= 1'b1; // R
                keys[14][3] <= 1'b1; // For 3FFE
                keys[15][3] <= 1'b1; // For 3FFF
            end
            {8'h2d, 1'b0, 1'bz}: begin 
                keys[6][2]  <= 1'b0; // R
                keys[14][3] <= 1'b0; // For 3FFE
                keys[15][3] <= 1'b0; // For 3FFF
            end
            
            {8'h1b, 1'b1, 1'bz}: begin 
                keys[7][4]  <= 1'b1; // S
                keys[12][4] <= lk2 ? 1'b1 : keys[12][4]; // For 3FFC
                keys[15][3] <= 1'b1; // For 3FFF
            end
            {8'h1b, 1'b0, 1'bz}: begin 
                keys[7][4]  <= 1'b0; // S
                keys[12][4] <= lk2 ? 1'b0 : keys[12][4]; // For 3FFC
                keys[15][3] <= 1'b0; // For 3FFF
            end
            
            {8'h2c, 1'b1, 1'bz}: keys[6][3] <= 1'b1; // T
            {8'h2c, 1'b0, 1'bz}: keys[6][3] <= 1'b0; // T
            
            {8'h3c, 1'b1, 1'bz}: keys[5][2] <= 1'b1; // U
            {8'h3c, 1'b0, 1'bz}: keys[5][2] <= 1'b0; // U
            
            {8'h2a, 1'b1, 1'bz}: begin 
                keys[6][7]  <= 1'b1; // V
                keys[14][1] <= 1'b1; // For 3FFE
                keys[15][3] <= 1'b1; // For 3FFF
            end
            {8'h2a, 1'b0, 1'bz}: begin 
                keys[6][7]  <= 1'b0; // V
                keys[14][1] <= 1'b0; // For 3FFE
                keys[15][3] <= 1'b0; // For 3FFF
            end
            
            {8'h1d, 1'b1, 1'bz}: begin 
                keys[7][3]  <= 1'b1; // W
                keys[12][0] <= lk2 ? 1'b1 : keys[12][0]; // For 3FFC
                keys[14][3] <= 1'b1; // For 3FFE
                keys[15][3] <= 1'b1; // For 3FFF
            end
            {8'h1d, 1'b0, 1'bz}: begin 
                keys[7][3]  <= 1'b0; // W
                keys[12][0] <= lk2 ? 1'b0 : keys[12][0]; // For 3FFC
                keys[14][3] <= 1'b0; // For 3FFE
                keys[15][3] <= 1'b0; // For 3FFF
            end
            
            {8'h22, 1'b1, 1'bz}: begin 
                keys[7][7]  <= 1'b1; // X
                keys[12][1] <= lk2 ? 1'b1 : keys[12][1]; // For 3FFC
                keys[14][1] <= 1'b1; // For 3FFE
                keys[15][3] <= 1'b1; // For 3FFF
            end
            {8'h22, 1'b0, 1'bz}: begin 
                keys[7][7]  <= 1'b0; // X
                keys[12][1] <= lk2 ? 1'b0 : keys[12][1]; // For 3FFC
                keys[14][1] <= 1'b0; // For 3FFE
                keys[15][3] <= 1'b0; // For 3FFF
            end
            
            {8'h35, 1'b1, 1'bz}: keys[5][3] <= 1'b1; // Y
            {8'h35, 1'b0, 1'bz}: keys[5][3] <= 1'b0; // Y
            
            {8'h1a, 1'b1, 1'bz}: begin 
                keys[8][7]  <= 1'b1; // Z
                keys[14][1] <= 1'b1; // For 3FFE
                keys[15][2] <= 1'b1; // For 3FFF
            end
            {8'h1a, 1'b0, 1'bz}: begin 
                keys[8][7]  <= 1'b0; // Z
                keys[14][1] <= 1'b0; // For 3FFE
                keys[15][2] <= 1'b0; // For 3FFF
            end
            
            // Number keys
            {8'h16, 1'b1, 1'bz}: keys[8][0] <= 1'b1; // 1
            {8'h16, 1'b0, 1'bz}: keys[8][0] <= 1'b0; // 1
            
            {8'h1e, 1'b1, 1'bz}: keys[8][1] <= 1'b1; // 2
            {8'h1e, 1'b0, 1'bz}: keys[8][1] <= 1'b0; // 2
            
            {8'h26, 1'b1, 1'bz}: keys[7][1] <= 1'b1; // 3
            {8'h26, 1'b0, 1'bz}: keys[7][1] <= 1'b0; // 3
            
            {8'h25, 1'b1, 1'bz}: keys[7][0] <= 1'b1; // 4
            {8'h25, 1'b0, 1'bz}: keys[7][0] <= 1'b0; // 4
            
            {8'h2e, 1'b1, 1'bz}: keys[6][1] <= 1'b1; // 5
            {8'h2e, 1'b0, 1'bz}: keys[6][1] <= 1'b0; // 5
            
            {8'h36, 1'b1, 1'bz}: keys[6][0] <= 1'b1; // 6
            {8'h36, 1'b0, 1'bz}: keys[6][0] <= 1'b0; // 6
            
            {8'h3d, 1'b1, 1'bz}: keys[5][1] <= 1'b1; // 7
            {8'h3d, 1'b0, 1'bz}: keys[5][1] <= 1'b0; // 7
            
            {8'h3e, 1'b1, 1'bz}: keys[5][0] <= 1'b1; // 8
            {8'h3e, 1'b0, 1'bz}: keys[5][0] <= 1'b0; // 8
            
            {8'h46, 1'b1, 1'bz}: keys[4][1] <= 1'b1; // 9
            {8'h46, 1'b0, 1'bz}: keys[4][1] <= 1'b0; // 9
            
            {8'h45, 1'b1, 1'bz}: keys[4][0] <= 1'b1; // 0
            {8'h45, 1'b0, 1'bz}: keys[4][0] <= 1'b0; // 0
            
            // Special characters
            // Comma/less than
            {8'h41, 1'b1, 1'bz}: begin 
                if (shiftstate) begin
                    keys[3][4] <= 1'b1; // <
                    keys[15][0] <= 1'b1; // For 3FFF
                end else begin
                    keys[4][7] <= 1'b1; // ,
                    keys[14][2] <= 1'b1; // For 3FFE
                    keys[15][1] <= 1'b1; // For 3FFF
                end
            end
            {8'h41, 1'b0, 1'bz}: begin 
                keys[3][4] <= 1'b0; // <
                keys[4][7] <= 1'b0; // ,
                keys[14][2] <= 1'b0; // For 3FFE
                keys[15][0] <= 1'b0; // For 3FFF
                keys[15][1] <= 1'b0; // For 3FFF
            end
            
            // Period/greater than
            {8'h49, 1'b1, 1'bz}: begin 
                if (shiftstate) begin
                    keys[2][3] <= 1'b1; // >
                    keys[15][0] <= 1'b1; // For 3FFF
                end else begin
                    keys[3][7] <= 1'b1; // .
                    keys[14][3] <= 1'b1; // For 3FFE
                    keys[15][1] <= 1'b1; // For 3FFF
                end
            end
            {8'h49, 1'b0, 1'bz}: begin 
                keys[2][3] <= 1'b0; // >
                keys[3][7] <= 1'b0; // .
                keys[14][3] <= 1'b0; // For 3FFE
                keys[15][0] <= 1'b0; // For 3FFF
                keys[15][1] <= 1'b0; // For 3FFF
            end
            
            // Other common keys
            {8'h0d, 1'b1, 1'bz}: keys[8][4] <= 1'b1; // TAB
            {8'h0d, 1'b0, 1'bz}: keys[8][4] <= 1'b0; // TAB
            
            {8'h76, 1'b1, 1'bz}: begin
                keys[1][0] <= 1'b1; // ESCAPE (PC) -> EXIT (PCW)
                keys[12][2] <= 1'b1; // For 3FFC
            end
            {8'h76, 1'b0, 1'bz}: begin
                keys[1][0] <= 1'b0; // ESCAPE (PC) -> EXIT (PCW)
                keys[12][2] <= 1'b0; // For 3FFC
            end
            
            {8'h29, 1'b1, 1'bz}: begin
                keys[5][7] <= 1'b1; // SPACE
                keys[12][4] <= 1'b1; // For 3FFC
                keys[13][5] <= 1'b1; // For 3FFD
                keys[14][4] <= 1'b1; // For 3FFE
                keys[15][4] <= 1'b1; // For 3FFF
            end
            {8'h29, 1'b0, 1'bz}: begin
                keys[5][7] <= 1'b0; // SPACE
                keys[12][4] <= 1'b0; // For 3FFC
                keys[13][5] <= 1'b0; // For 3FFD
                keys[14][4] <= 1'b0; // For 3FFE
                keys[15][4] <= 1'b0; // For 3FFF
            end
            
            // Numeric keypad handling
            // NUM 0 and INSERT
            {8'h70, 1'b1, 1'b0}: begin
                keys[0][1] <= 1'b1; // NUM 0
                keys[12][3] <= 1'b1; // For 3FFC
            end
            {8'h70, 1'b0, 1'b0}: begin
                keys[0][1] <= 1'b0; // NUM 0
                keys[12][3] <= 1'b0; // For 3FFC
            end
            {8'h70, 1'b1, 1'b1}: keys[1][2] <= 1'b1; // INS (PC) -> CUT (PCW)
            {8'h70, 1'b0, 1'b1}: keys[1][2] <= 1'b0; // INS (PC) -> CUT (PCW)
            
            // NUM 1 and END
            {8'h69, 1'b1, 1'b0}: begin
                keys[1][7] <= 1'b1; // NUM 1
                keys[13][2] <= 1'b1; // For 3FFD
            end
            {8'h69, 1'b0, 1'b0}: begin
                keys[1][7] <= 1'b0; // NUM 1
                keys[13][2] <= 1'b0; // For 3FFD
            end
            {8'h69, 1'b1, 1'b1}: keys[10][2] <= 1'b1; // END (PC) -> CANCEL (PCW)
            {8'h69, 1'b0, 1'b1}: keys[10][2] <= 1'b0; // END (PC) -> CANCEL (PCW)
            
            // NUM 2 and DOWN ARROW
            {8'h72, 1'b1, 1'b0}: begin
                keys[0][7] <= 1'b1; // NUM 2
                keys[13][4] <= 1'b1; // For 3FFD
            end
            {8'h72, 1'b0, 1'b0}: begin
                keys[0][7] <= 1'b0; // NUM 2
                keys[13][4] <= 1'b0; // For 3FFD
            end
            {8'h72, 1'b1, 1'b1}: keys[10][6] <= 1'b1; // DN ARROW (PC) -> NUM . (PCW)
            {8'h72, 1'b0, 1'b1}: keys[10][6] <= 1'b0; // DN ARROW (PC) -> NUM . (PCW)
            
            // NUM 3
            {8'h7a, 1'b1, 1'bz}: begin
                keys[0][6] <= 1'b1; // NUM 3
                keys[13][3] <= 1'b1; // For 3FFD
            end
            {8'h7a, 1'b0, 1'bz}: begin
                keys[0][6] <= 1'b0; // NUM 3
                keys[13][3] <= 1'b0; // For 3FFD
            end
            
            // NUM 4 and LEFT ARROW
            {8'h6b, 1'b1, 1'b0}: keys[1][5] <= 1'b1; // NUM 4
            {8'h6b, 1'b0, 1'b0}: keys[1][5] <= 1'b0; // NUM 4
            {8'h6b, 1'b1, 1'b1}: keys[1][7] <= 1'b1; // LF ARROW (PC) -> NUM 1 (PCW)
            {8'h6b, 1'b0, 1'b1}: keys[1][7] <= 1'b0; // LF ARROW (PC) -> NUM 1 (PCW)
            
            // NUM 5
            {8'h73, 1'b1, 1'bz}: begin
                keys[1][6] <= 1'b1; // NUM 5
                keys[13][1] <= 1'b1; // For 3FFD
            end
            {8'h73, 1'b0, 1'bz}: begin
                keys[1][6] <= 1'b0; // NUM 5
                keys[13][1] <= 1'b0; // For 3FFD
            end
            
            // NUM 6 and RIGHT ARROW
            {8'h74, 1'b1, 1'b0}: keys[0][5] <= 1'b1; // NUM 6
            {8'h74, 1'b0, 1'b0}: keys[0][5] <= 1'b0; // NUM 6
            {8'h74, 1'b1, 1'b1}: keys[0][6] <= 1'b1; // RT ARROW (PC) -> NUM 3 (PCW)
            {8'h74, 1'b0, 1'b1}: keys[0][6] <= 1'b0; // RT ARROW (PC) -> NUM 3 (PCW)
            
            // NUM 7
            {8'h6c, 1'b1, 1'bz}: keys[2][4] <= 1'b1; // NUM 7
            {8'h6c, 1'b0, 1'bz}: keys[2][4] <= 1'b0; // NUM 7
            
            // NUM 8 and UP ARROW
            {8'h75, 1'b1, 1'b0}: keys[1][4] <= 1'b1; // NUM 8
            {8'h75, 1'b0, 1'b0}: keys[1][4] <= 1'b0; // NUM 8
            {8'h75, 1'b1, 1'b1}: keys[1][6] <= 1'b1; // UP ARROW (PC) -> NUM 5 (PCW)
            {8'h75, 1'b0, 1'b1}: keys[1][6] <= 1'b0; // UP ARROW (PC) -> NUM 5 (PCW)
            
            // NUM 9 and PAGE UP
            {8'h7d, 1'b1, 1'b0}: keys[0][4] <= 1'b1; // NUM 9
            {8'h7d, 1'b0, 1'b0}: keys[0][4] <= 1'b0; // NUM 9
            {8'h7d, 1'b1, 1'b1}: keys[0][3] <= 1'b1; // PAGE UP (PC) -> PASTE (PCW)
            {8'h7d, 1'b0, 1'b1}: keys[0][3] <= 1'b0; // PAGE UP (PC) -> PASTE (PCW)
            
            // NUM . and DELETE
            {8'h71, 1'b1, 1'b0}: begin
                keys[10][6] <= 1'b1; // NUM .
                keys[13][0] <= 1'b1; // For 3FFD
            end
            {8'h71, 1'b0, 1'b0}: begin
                keys[10][6] <= 1'b0; // NUM .
                keys[13][0] <= 1'b0; // For 3FFD
            end
            {8'h71, 1'b1, 1'b1}: keys[2][0] <= 1'b1; // DEL (PC) -> ->DEL (PCW)
            {8'h71, 1'b0, 1'b1}: keys[2][0] <= 1'b0; // DEL (PC) -> ->DEL (PCW)
            
            // ENTER keys
            {8'h5a, 1'b1, 1'b0}: keys[2][2] <= 1'b1; // ENTER
            {8'h5a, 1'b0, 1'b0}: keys[2][2] <= 1'b0; // ENTER
            {8'h5a, 1'b1, 1'b1}: begin
                keys[10][5] <= 1'b1; // NUM Enter
                keys[12][5] <= 1'b1; // For 3FFC
            end
            {8'h5a, 1'b0, 1'b1}: begin
                keys[10][5] <= 1'b0; // NUM Enter
                keys[12][5] <= 1'b0; // For 3FFC
            end
            
            // NUM + and -
            {8'h79, 1'b1, 1'bz}: keys[2][7] <= 1'b1; // NUM [+]
            {8'h79, 1'b0, 1'bz}: keys[2][7] <= 1'b0; // NUM [+]
            {8'h7b, 1'b1, 1'bz}: keys[10][3] <= 1'b1; // NUM [-]
            {8'h7b, 1'b0, 1'bz}: keys[10][3] <= 1'b0; // NUM [-]
            
            // BACKSPACE
            {8'h66, 1'b1, 1'bz}: keys[9][7] <= 1'b1; // BACKSPACE (PC) -> <-DEL (PCW)
            {8'h66, 1'b0, 1'bz}: keys[9][7] <= 1'b0; // BACKSPACE (PC) -> <-DEL (PCW)
            
            // HOME key -> COPY
            {8'h6e, 1'b1, 1'b1}: keys[1][3] <= 1'b1; // HOME (PC) -> COPY (PCW)
            {8'h6e, 1'b0, 1'b1}: keys[1][3] <= 1'b0; // HOME (PC) -> COPY (PCW)
            
            // CAPS LOCK -> SHIFT LOCK
 //           {8'h58, 1'b1, 1'bz}: begin
 //               keys[8][6] <= 1'b1; // CAPS LOCK (PC) -> SHIFT LOCK (PCW)
 //               capslock <= ~capslock; // Toggle capslock state
 //           end
 //           {8'h58, 1'b0, 1'bz}: keys[8][6] <= 1'b0; // CAPS LOCK (PC) -> SHIFT LOCK (PCW)
 
// CAPS LOCK -> SHIFT LOCK - Implementación de toggle adecuado
{8'h58, 1'b1, 1'bz}: begin
    // Solo actualizamos la señal sin afectar al estado de latch todavía
    keys[8][6] <= 1'b1; // CAPS LOCK (PC) -> SHIFT LOCK (PCW)
    if (!caps_pressed_prev) begin
        // Solo togglamos el estado cuando detectamos flanco de subida
        capslock <= ~capslock;
        caps_pressed_prev <= 1'b1;
    end
end
{8'h58, 1'b0, 1'bz}: begin
    keys[8][6] <= 1'b0; // CAPS LOCK (PC) -> SHIFT LOCK (PCW) released
    caps_pressed_prev <= 1'b0; // Resetea el detector de flanco para el próximo ciclo
end
 
            // Other misc keys
            {8'h0e, 1'b1, 1'bz}: keys[8][2] <= 1'b1; // ` (PC) -> STOP (PCW)
            {8'h0e, 1'b0, 1'bz}: keys[8][2] <= 1'b0; // ` (PC) -> STOP (PCW)
            {8'h4a, 1'b1, 1'bz}: begin
                keys[3][6] <= 1'b1; // /
                keys[14][2] <= 1'b1; // For 3FFE
                keys[15][1] <= 1'b1; // For 3FFF
            end
            {8'h4a, 1'b0, 1'bz}: begin
                keys[3][6] <= 1'b0; // /
                keys[14][2] <= 1'b0; // For 3FFE
                keys[15][1] <= 1'b0; // For 3FFF
            end
            {8'h52, 1'b1, 1'bz}: keys[2][6] <= 1'b1; // @
            {8'h52, 1'b0, 1'bz}: keys[2][6] <= 1'b0; // @
            {8'h54, 1'b1, 1'bz}: begin
                keys[3][2] <= 1'b1; // [
                keys[14][2] <= 1'b1; // For 3FFE
                keys[15][2] <= 1'b1; // For 3FFF
            end
            {8'h54, 1'b0, 1'bz}: begin
                keys[3][2] <= 1'b0; // [
                keys[14][2] <= 1'b0; // For 3FFE
                keys[15][2] <= 1'b0; // For 3FFF
            end
            {8'h5b, 1'b1, 1'bz}: begin
                keys[2][1] <= 1'b1; // ]
                keys[14][3] <= 1'b1; // For 3FFE
                keys[15][3] <= 1'b1; // For 3FFF
            end
            {8'h5b, 1'b0, 1'bz}: begin
                keys[2][1] <= 1'b0; // ]
                keys[14][3] <= 1'b0; // For 3FFE
                keys[15][3] <= 1'b0; // For 3FFF
            end
            {8'h4c, 1'b1, 1'bz}: begin
                keys[3][5] <= 1'b1; // ;
                keys[14][3] <= 1'b1; // For 3FFE
                keys[15][0] <= 1'b1; // For 3FFF
            end
            {8'h4c, 1'b0, 1'bz}: begin
                keys[3][5] <= 1'b0; // ;
                keys[14][3] <= 1'b0; // For 3FFE
                keys[15][0] <= 1'b0; // For 3FFF
            end
            {8'h4e, 1'b1, 1'bz}: keys[3][1] <= 1'b1; // -
            {8'h4e, 1'b0, 1'bz}: keys[3][1] <= 1'b0; // -
            {8'h55, 1'b1, 1'bz}: keys[3][0] <= 1'b1; // =
            {8'h55, 1'b0, 1'bz}: keys[3][0] <= 1'b0; // =
            
            // Backslash/half key
            {8'h5d, 1'b1, 1'bz}: begin
                keys[2][6] <= 1'b1; // \ (PC) -> 1/2 (PCW)
                keys[14][3] <= 1'b1; // For 3FFE
                keys[15][1] <= 1'b1; // For 3FFF
            end
            {8'h5d, 1'b0, 1'bz}: begin
                keys[2][6] <= 1'b0; // \ (PC) -> 1/2 (PCW)
                keys[14][3] <= 1'b0; // For 3FFE
                keys[15][1] <= 1'b0; // For 3FFF
            end
            
            // Function keys for color control
            {8'h01, 1'b1, 1'bz}: begin
                up_key <= 1'b1;       // F9 (PC) -> move row up
                repeat_count <= 0;
            end
            {8'h01, 1'b0, 1'bz}: begin
                up_key <= 1'b0;       // F9 (PC) -> move row up released
            end
            
            {8'h09, 1'b1, 1'bz}: begin
                down_key <= 1'b1;     // F10 (PC) -> move row down
                repeat_count <= 0;
            end
            {8'h09, 1'b0, 1'bz}: begin
                down_key <= 1'b0;     // F10 (PC) -> move row down released
            end
            
            {8'h78, 1'b1, 1'bz}: toggle_full <= 1'b1; // F11 (PC) -> toggle full
            {8'h78, 1'b0, 1'bz}: toggle_full <= 1'b0; // F11 (PC) -> toggle full released
            
            // Default case for any unhandled keys
            default: begin
                // No action for unhandled keys
            end
        endcase
	end     // End input strobe

	// Color line handler
	line_down <= 1'b0;
	line_up <= 1'b0;
	
	// Countdown timer for colour line handler
	if(repeat_count > REPEAT_TIME && (up_key | down_key)) begin
		repeat_count <= 0;
		line_up <= up_key;
		line_down <= down_key;
	end
	else if(up_key | down_key) begin 
		repeat_count <= repeat_count + 1;
	end

    // Set special flags and signals regardless of input_strobe
    
    // 3FFD special flags
    keys[13][7] <= ~lk2;
    keys[13][6] <= capslock;
    
    // 3FFE special flags
    keys[14][7] <= lk3;
    keys[14][6] <= lk1;
    
    // 3FFF special flags
    keys[15][7] <= 'b1;             // PCW keyboard transmitting
    keys[15][6] <= input_strobe;    // Update flag

    // Joystick handling
    
    // Joystick Driver 1 - 3FF9
    keys[9][5] <= joy0[5];          // Fire 2
    keys[9][4] <= joy0[4];          // Fire 1
    keys[9][3] <= joy0[0];          // Right
    keys[9][2] <= joy0[1];          // Left
    keys[9][1] <= joy0[2];          // Down
    keys[9][0] <= joy0[3];          // Up

    // Joystick Driver 2 - 3FFB
    keys[11][5] <= joy1[5];          // Fire 2
    keys[11][4] <= joy1[4];          // Fire 1
    keys[11][3] <= joy1[0];          // Right
    keys[11][2] <= joy1[1];          // Left
    keys[11][1] <= joy1[2];          // Down
    keys[11][0] <= joy1[3];          // Up
end

endmodule
