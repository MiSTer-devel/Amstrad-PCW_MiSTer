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
	input wire 		   keymouse,	// Keymouse in use
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
logic		 shiftstate = 0;
logic		 extended = 0;
logic        shifted = 0;
logic        capslock = 0;

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

assign shiftstate = capslock | shifted;
// Translate PC keyboard presses into PCW keyboard presses

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

    // Keyboard processing Main
	if(input_strobe) begin
		case(code)
			8'h12: begin  
                keys[2][5] <= pressed & ~extended; // LEFT SHIFT (PC)
			    keys[1][1] <= pressed & extended; // PRT SCR (PC) -> PTR (PCW)
                shifted <= pressed & ~extended;
            end
			8'h59: begin
                keys[2][5]   <= pressed; // RIGHT SHIFT (PC)
                shifted <= pressed;
            end
			8'h11: keys[10][7]  <= pressed; // ALT
			8'h14: keys[10][1]  <= pressed; // CTRL (PC) -> EXTRA (PCW) 
			8'h05: keys[0][2]   <= pressed; // F1 (PC) -> F1/F2 (PCW)
			8'h06: begin
				keys[0][2]   <= pressed; // F2 (PC) -> F1/F2 (PCW)
				keys[2][5] <= pressed; // LEFT SHIFT (PC)
				shifted <= pressed;
			end
			8'h04: keys[0][0]   <= pressed; // F3 (PC) -> F3/F4 (PCW)
			8'h0C: begin
				keys[0][0]   <= pressed; // F4 (PC) -> F3/F4 (PCW)
				keys[2][5] <= pressed; // LEFT SHIFT (PC)
				shifted <= pressed;
			end				
			8'h03: keys[10][0]  <= pressed; // F5 (PC) -> F5/F6 (PCW)
			8'h0B: begin
				keys[10][0]  <= pressed; // F6 (PC) -> F5/F6 (PCW)
				keys[2][5] <= pressed; // LEFT SHIFT (PC)
				shifted <= pressed;
			end				
			8'h83: keys[10][4]  <= pressed; // F7 (PC) -> F7/F8 (PCW)
			8'h0A: begin
				keys[10][4]  <= pressed; // F8 (PC) -> F7/F9 (PCW)
				keys[2][5] <= pressed; // LEFT SHIFT (PC)
				shifted <= pressed;
			end				
			8'h1c : keys[8][5] <= pressed; // A
			8'h32 : keys[6][6] <= pressed; // B
			8'h21 : keys[7][6] <= pressed; // C
			8'h23 : keys[7][5] <= pressed; // D
			8'h24 : keys[7][2] <= pressed; // E
			8'h2b : keys[6][5] <= pressed; // F
			8'h34 : keys[6][4] <= pressed; // G
			
			8'h33 : keys[5][4] <= pressed; // H
			8'h43 : keys[4][3] <= pressed; // I
			8'h3b : keys[5][5] <= pressed; // J
			8'h42 : keys[4][5] <= pressed; // K
			8'h4b : keys[4][4] <= pressed; // L
			8'h3a : keys[4][6] <= pressed; // M
			8'h31 : keys[5][6] <= pressed; // N
			8'h44 : keys[4][2] <= pressed; // O
			
			8'h4d : keys[3][3] <= pressed; // P
			8'h15 : keys[8][3] <= pressed; // Q
			8'h2d : keys[6][2] <= pressed; // R
			8'h1b : keys[7][4] <= pressed; // S
			8'h2c : keys[6][3] <= pressed; // T
			8'h3c : keys[5][2] <= pressed; // U
			8'h2a : keys[6][7] <= pressed; // V
			8'h1d : keys[7][3] <= pressed; // W
			
			8'h22 : keys[7][7] <= pressed; // X
			8'h35 : keys[5][3] <= pressed; // Y
			8'h1a : keys[8][7] <= pressed; // Z

			8'h16 : keys[8][0] <= pressed; // 1
            8'h1e : keys[8][1] <= pressed; // 2
			8'h26 : keys[7][1] <= pressed; // 3
			8'h25 : keys[7][0] <= pressed; // 4
			8'h2e : keys[6][1] <= pressed; // 5
			8'h36 : keys[6][0] <= pressed; // 6
			8'h3d : keys[5][1] <= pressed; // 7
			8'h3e : keys[5][0] <= pressed; // 8
			8'h46 : keys[4][1] <= pressed; // 9
			8'h45 : keys[4][0] <= pressed; // 0

			8'h41 : begin
                keys[3][4] <= pressed & shiftstate; // <
                keys[4][7] <= pressed & ~shiftstate; // ,
            end
			8'h49 : begin
                keys[2][3] <= pressed & shiftstate; // >
                keys[3][7] <= pressed & ~shiftstate; // .
            end

			8'h0d : keys[8][4] <= pressed; // TAB
			8'h76 : keys[1][0] <= pressed; // ESCAPE (PC) -> EXIT (PCW)
			8'h29 : keys[5][7] <= pressed; // SPACE

			8'h70 : begin
                keys[0][1] <= pressed & ~extended; // NUM 0
			    keys[1][2] <= pressed & extended; // INS (PC) -> CUT (PCW)
            end
			8'h69 : begin
                keys[1][7] <= pressed; // NUM 1
			    keys[10][2] <= pressed & extended; // END (PC) -> CANCEL (PCW)
            end
			8'h72 : begin
                keys[0][7] <= pressed & ~extended; // NUM 2
			    keys[10][6] <= pressed & extended; // DN ARROW (PC) -> NUM . (PCW)
            end
			8'h7a : keys[0][6] <= pressed; // NUM 3
			8'h6b : begin
                keys[1][5] <= pressed & ~extended; // NUM 4
			    keys[1][7] <= pressed & extended; // LF ARROW (PC) -> NUM 1 (PCW)
            end
			8'h73 : keys[1][6] <= pressed; // NUM 5
			8'h74 : begin
                keys[0][5] <= pressed & ~extended; // NUM 6
			    keys[0][6] <= pressed & extended; // RT ARROW (PC) -> NUM 3 (PCW)
            end
			8'h6c : keys[2][4] <= pressed; // NUM 7
			8'h75 : begin
                keys[1][4] <= pressed & ~extended; // NUM 8
			    keys[1][6] <= pressed & extended; // UP ARROW (PC) -> NUM 5 (PCW)
            end
			8'h7d : begin
                keys[0][4] <= pressed & ~extended; // NUM 9
			    keys[0][3] <= pressed & extended; // PAGE UP (PC) -> PASTE (PCW)
            end
			8'h71 : begin
                keys[10][6] <= pressed & ~extended; // NUM .
			    keys[2][0] <= pressed & extended; // DEL (PC) -> ->DEL (PCW)
            end
			8'h5a : begin
                keys[2][2] <= pressed & ~extended; // ENTER
			    keys[10][5] <= pressed & extended; // NUM Enter
            end
			8'h79 : keys[2][7] <= pressed; // NUM [+]
			8'h7b : keys[10][3] <= pressed; // NUM [-]

            // Deletes
			8'h66 : keys[9][7] <= pressed; // BACKSPACE (PC) -> <-DEL (PCW)

            // WP keys
			8'h6e : keys[1][3] <= pressed & extended; // HOME (PC) -> COPY (PCW)
			8'h58 : begin
                keys[8][6] <= pressed; // CAPS LOCK (PC) -> SHIFT LOCK (PCW)
                capslock <= pressed ? ~capslock : capslock;
            end
            
            // Other keys
			8'h0e : keys[8][2] <= pressed; // ` (PC) -> STOP (PCW)
			8'h4a : keys[3][6] <= pressed; // /
            8'h52 : keys[2][6] <= pressed; // @
			8'h54 : keys[3][2] <= pressed; // [
			8'h5b : keys[2][1] <= pressed; // ]
			8'h4c : keys[3][5] <= pressed; // ;
			8'h4e : keys[3][1] <= pressed; // -
			8'h55 : keys[3][0] <= pressed; // =
		endcase

        // Keyboard processing for combination keys in 3FFC
		case(code)
			8'h5a : keys[12][5] <= pressed & extended; // NUM Enter
			8'h59 : keys[12][5] <= pressed & lk2; // RIGHT SHIFT (PC)
            8'h29 : keys[12][4] <= pressed; // SPACE
   			8'h1b : keys[12][4] <= pressed & lk2; // S
			8'h70 : keys[12][3] <= pressed; // NUM 0
			8'h23 : keys[12][3] <= pressed & lk2; // D
			8'h76 : keys[12][2] <= pressed; // ESCAPE (PC) -> EXIT (PCW)
			8'h1c : keys[12][2] <= pressed & lk2; // A
			8'h05 : keys[12][1] <= pressed; // F1 (PC) -> F1/F2 (PCW)
			8'h22 : keys[12][1] <= pressed & lk2; // X
			8'h04 : keys[12][0] <= pressed; // F3 (PC) -> F3/F4 (PCW)
			8'h1d : keys[12][0] <= pressed & lk2; // W
        endcase

        // Keyboard processing for combination keys in 3FFD
		case(code)
            // Shadows into 3FFD
            8'h29 : keys[13][5] <= pressed; // SPACE
			8'h72 : keys[13][4] <= pressed; // NUM 2
			8'h7a : keys[13][3] <= pressed; // NUM 3
			8'h69 : keys[13][2] <= pressed; // NUM 1
			8'h73 : keys[13][1] <= pressed; // NUM 5
			8'h71 : keys[13][0] <= pressed; // NUM .
        endcase

        // Keyboard processing for combination keys in 3FFE
		case(code)
			8'h59 : keys[14][5] <= pressed & lk2; // RIGHT SHIFT (PC)
            8'h29 : keys[14][4] <= pressed; // SPACE
            // Bit 3
			8'h1d : keys[14][3] <= pressed; // W
			8'h2d : keys[14][3] <= pressed; // R
			8'h4d : keys[14][3] <= pressed; // P
			8'h5b : keys[14][3] <= pressed; // ]
			8'h4c : keys[14][3] <= pressed; // ;
			8'h49 : keys[14][3] <= pressed; // > & .
			8'h5d : keys[14][3] <= pressed; // \ (PC) -> 1/2 (PCW)
            // Bit 2
			8'h15 : keys[14][2] <= pressed; // Q
			8'h24 : keys[14][2] <= pressed; // E
			8'h44 : keys[14][2] <= pressed; // O
			8'h54 : keys[14][2] <= pressed; // [
			8'h4b : keys[14][2] <= pressed; // L
			8'h41 : keys[14][2] <= pressed; // < & ,
			8'h4a : keys[14][2] <= pressed; // /
            // Bit 1
			8'h1a : keys[14][1] <= pressed; // Z
			8'h22 : keys[14][1] <= pressed; // X
			8'h21 : keys[14][1] <= pressed; // C
			8'h2a : keys[14][1] <= pressed; // V
			8'h32 : keys[14][1] <= pressed; // B
			8'h31 : keys[14][1] <= pressed; // N
			8'h3a : keys[14][1] <= pressed; // M
            // Bit 0
			8'h1c : keys[14][0] <= pressed; // A
			8'h1b : keys[14][0] <= pressed; // S
			8'h23 : keys[14][0] <= pressed; // D
			8'h2b : keys[14][0] <= pressed; // F
			8'h34 : keys[14][0] <= pressed; // G
			8'h33 : keys[14][0] <= pressed; // H
			8'h3b : keys[14][0] <= pressed; // J
        endcase

        // Keyboard processing for combination keys in 3FFF
		case(code)
			8'h59 : keys[15][5] <= pressed & lk2; // RIGHT SHIFT (PC)
            8'h29 : keys[15][4] <= pressed; // SPACE
            // Bit 3
			8'h1d : keys[15][3] <= pressed; // W
			8'h2d : keys[15][3] <= pressed; // R
			8'h4d : keys[15][3] <= pressed; // P
			8'h5b : keys[15][3] <= pressed; // ]
			8'h1b : keys[15][3] <= pressed; // S
			8'h2b : keys[15][3] <= pressed; // F
			8'h22 : keys[15][3] <= pressed; // X
			8'h2a : keys[15][3] <= pressed; // V
            // Bit 2
			8'h15 : keys[15][2] <= pressed; // Q
			8'h24 : keys[15][2] <= pressed; // E
			8'h44 : keys[15][2] <= pressed; // O
			8'h54 : keys[15][2] <= pressed; // [
			8'h1c : keys[15][2] <= pressed; // A
			8'h23 : keys[15][2] <= pressed; // D
			8'h1a : keys[15][2] <= pressed; // Z
			8'h21 : keys[15][2] <= pressed; // C
            // Bit 1
			8'h32 : keys[15][1] <= pressed; // B
			8'h31 : keys[15][1] <= pressed; // N
			8'h3a : keys[15][1] <= pressed; // M
			8'h41 : begin
                keys[15][1] <= pressed & ~shiftstate; // ,
			    keys[15][0] <= pressed & shiftstate; // < - Bit 0
            end
			8'h49 : begin
                keys[15][1] <= pressed & ~shiftstate; // .
                keys[15][0] <= pressed & shiftstate; // > - Bit 0
            end
			8'h4a : keys[15][1] <= pressed; // /
			8'h5d : keys[15][1] <= pressed; // \ (PC) -> 1/2 (PCW)
            // Bit 0
			8'h33 : keys[15][0] <= pressed; // H
			8'h3b : keys[15][0] <= pressed; // J
			8'h42 : keys[15][0] <= pressed; // K
			8'h4b : keys[15][0] <= pressed; // L
   			8'h4c : keys[15][0] <= pressed; // ;
        endcase
		// Fake colour line control keys
		case(code)
			8'h01 : begin
				up_key <= pressed;  // F9 (PC) -> move row up
				repeat_count <= 0;
			end
			8'h09 : begin
				down_key <= pressed; // F10 (PC) -> move row down
				repeat_count <= 0;
			end
			8'h78 : begin
				//toggle_full <= pressed; // F11 (PC) -> toggle full
				toggle_full <= ~toggle_full;  // Invertir el valor actual de toggle_full
			end			
		endcase
	end     // End input strobe

	line_down <= 1'b0;
	line_up <= 1'b0;
	// Countdown timer for colour line handler
	if(repeat_count > REPEAT_TIME && (up_key | down_key))
	begin
		repeat_count <= 0;
		line_up <= up_key;
		line_down <= down_key;
	end
	else if(up_key | down_key) repeat_count <= repeat_count + 1;

    // Special flags and signals
    // 3FFD
    keys[13][7] <= ~lk2;
    keys[13][6] <= capslock;
    // 3FFE
    keys[14][7] <= lk3;
    keys[14][6] <= lk1;
    // 3FFF
    keys[15][7] <= 'b1;             // PCW keyboard transmitting
    keys[15][6] <= input_strobe;    // Update flag

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
