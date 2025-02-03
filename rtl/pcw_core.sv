//
// PCW Main Core for PCW_MiSTer
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

module pcw_core(
    input wire reset,           // Reset
	input wire clk_sys,         // 64 Mhz System Clock

    output logic [23:0] RGB,    // RGB Output (8-8-8)     
	output logic hsync,         // Horizontal sync
	output logic vsync,         // Vertical sync
	output logic hblank,        // Horizontal blanking
	output logic vblank,        // Vertical blanking
    output logic ce_pix,        // Pixel clock

    output logic LED,           // LED output
    output logic [8:0] audiomix,
    input wire [7:0] joy0,
    input wire [7:0] joy1,
    input wire [2:0] joy_type,
    input wire [10:0] ps2_key,
    input wire [24:0] ps2_mouse,
    input wire [1:0] mouse_type,
    input wire [1:0] disp_color,
    input wire [1:0] overclock,
    input wire ntsc,
    input wire model,
    input wire [1:0] memory_size,
    input wire dktronics,
    input wire [2:0] fake_colour_mode,

    // SDRAM signals
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,
    input         locked,

    input wire dn_clk,
    input wire dn_go,
    input wire dn_wr,
    input wire [24:0] dn_addr,
    input wire [7:0] dn_data,

    input wire [15:0] execute_addr,
    input wire execute_enable,

    input wire [1:0]  img_mounted,
	input wire        img_readonly,
	input wire [31:0] img_size,
    input wire [1:0]  density,

	output logic [31:0] sd_lba,
	output logic [1:0] sd_rd,
	output logic [1:0] sd_wr,
	input  wire        sd_ack,
	input  wire  [8:0] sd_buff_addr,
	input  wire  [7:0] sd_buff_dout,
	output logic [7:0] sd_buff_din,
	input  wire        sd_dout_strobe
    );

    // Joystick types
    localparam JOY_NONE         = 3'b000;
    localparam JOY_KEMPSTON     = 3'b001;
    localparam JOY_SPECTRAVIDEO = 3'b010;
    localparam JOY_CASCADE      = 3'b011;
    localparam JOY_DKTRONICS    = 3'b100;

    localparam MOUSE_NONE       = 2'b00;
    localparam MOUSE_AMX        = 2'b01;
    localparam MOUSE_KEMPSTON   = 2'b10;
    localparam MOUSE_KEYMOUSE   = 2'b11;

    localparam MODEL_8512 = 0;
    localparam MODEL_9512 = 1;

    localparam MEM_256K = 0;
    localparam MEM_512K = 1;
    localparam MEM_1M = 2;
    localparam MEM_2M = 3;

    logic fake_colour;
    assign fake_colour = (fake_colour_mode != 3'b000);
	logic colorin;
	assign colorin = (fake_colour_mode == 3'b101);

    // Audio channels
    logic [7:0] ch_a;
    logic [7:0] ch_b;
    logic [7:0] ch_c;
    logic [9:0] audio;
    logic speaker_enable = 1'b0;

    // dpram addressing
    logic [16:0] ram_a_addr;
    logic [20:0] ram_b_addr/* synthesis keep */;
    logic [7:0] ram_a_dout;
    logic [7:0] ram_b_dout/* synthesis keep */;
    
    // cpu control
    logic [15:0] cpua;
    logic [7:0] cpudo;
    logic [7:0] cpudi;
    logic cpuwr,cpurd,cpumreq,cpuiorq,cpum1;
    logic cpuclk, cpuclk_r;
    logic romrd,ramrd,ramwr;
    logic ior,iow,memr,memw;
    logic reset_conditional;
    // Generate fractional CPU clock
    reg [15:0] cnt;
    always @(posedge clk_sys)
    begin
        case(overclock)
            2'b00: {cpuclk, cnt} <= cnt + 16'h1000;  // 1x4Mhz - divide by 16: (2^16)/16   = 0x1000
            2'b01: {cpuclk, cnt} <= cnt + 16'h2000;  // 2x4Mhz - divide by  8: (2^16)/8    = 0x2000
            2'b10: {cpuclk, cnt} <= cnt + 16'h4000;  // 4x4Mhz - divide by  4: (2^16)/4    = 0x4000
            2'b11: {cpuclk, cnt} <= cnt + 16'h8000;  // 8x4Mhz - divide by  2: (2^16)/2    = 0x8000
        endcase
    end

    // Generate fractional Disk clock, capped at 4x
    reg [15:0] dcnt;
    logic disk_clk;
    always @(posedge clk_sys)
    begin
        case(overclock)
            2'b00: {disk_clk, dcnt} <= dcnt + 16'h1000;  // 1x4Mhz - divide by 16: (2^16)/16   = 0x1000
            2'b01: {disk_clk, dcnt} <= dcnt + 16'h2000;  // 2x4Mhz - divide by  8: (2^16)/8    = 0x2000
            2'b10: {disk_clk, dcnt} <= dcnt + 16'h4000;  // 4x4Mhz - divide by  4: (2^16)/4    = 0x4000
            2'b11: {disk_clk, dcnt} <= dcnt + 16'h8000;  // 8x4Mhz - divide by  4: (2^16)/2    = 0x4000
        endcase
    end    

    // Generate 1mhz clock for ay-3-8912 sound
    reg [15:0] scnt;
    logic snd_clk;
    always @(posedge clk_sys)
    begin
        {snd_clk, scnt} <= scnt + 16'h0400;  // divide by 64 = 1Mhz
    end 

    // Gated clock which can be disabled during ROM download
    logic GCLK;
    assign GCLK = dn_go ? 1'b0 : cpuclk;

    // CPU register debugging for Signal Tap
    logic [15:0] PC /* synthesis keep */; 
    logic [15:0] SP /* synthesis keep */;
    logic [7:0]  AC /* synthesis keep */;
    logic [15:0] BC /* synthesis keep */; 
    logic [15:0] DE /* synthesis keep */;
    logic [15:0] HL /* synthesis keep */;
    logic [15:0] IX /* synthesis keep */;
    logic [15:0] IY /* synthesis keep */;
    logic Z /* synthesis keep */;
    logic N /* synthesis keep */;
    logic P  /* synthesis keep */;
    logic C /* synthesis keep */;

    // Used for CPU debugging in SignalTap
    z80_debugger debugger(
        .*,
        .ce(GCLK),
        .m1_n(cpum1),
        .REG_in(cpu_reg)
    ); 

    // Used to jump to address 0 on reset after ROM loads
    z80_regset z80_regset(
        .*,
        .dir_set(cpu_reg_set),
        .dir_out(cpu_reg_out)
    );

    // Generate CPU positive and negative edges (delayed 1 clk_sys)
    //logic cpu_pe, cpu_ne;
    //edge_det cpu_edge_det(.clk_sys(clk_sys), .signal(cpuclk), .pos_edge(cpu_pe), .neg_edge(cpu_ne));

    // CPU / memory access flags
    assign ior = cpurd | cpuiorq | ~cpum1;
    assign iow = cpuwr | cpuiorq | ~cpum1;
    assign memr = cpurd | cpumreq;
    assign memw = cpuwr | cpumreq;
    logic kbd_sel/* synthesis keep */;
    assign kbd_sel = ram_b_addr[20:4]==17'b00000111111111111 && memr==1'b0 ? 1'b1 : 1'b0;
    logic daisy_sel;
    assign daisy_sel = ((cpua[7:0]==8'hfc || cpua[7:0]==8'hfd) & model) && (~ior | ~iow)? 1'b1 : 1'b0;
	//logic color_256in;
	//assign color_256in =1'b0;

    wire WAIT_n = sdram_access ? ram_ready : 1'b1;
	// Create processor instance
    T80pa cpu(
       	.RESET_n(~reset),
        .CLK(clk_sys),
        .CEN_p(GCLK),
        .CEN_n(1'b1),
        .M1_n(cpum1),
        .WAIT_n(WAIT_n),
        .MREQ_n(cpumreq),
        .IORQ_n(cpuiorq),
        .NMI_n(nmi_sig),
        .INT_n(int_sig),
        .RD_n(cpurd),
        .WR_n(cpuwr),
        .A(cpua),
        .DI(cpudi),
        .DO(cpudo),
		.REG(cpu_reg),
        .DIR(cpu_reg_out),
        .DIRSet(cpu_reg_set)  
    );

    // Interrupt enable flag for timer interrupt check
    logic iff1/* synthesis keep */;
    assign iff1 = cpu_reg[210]/* synthesis keep */;
    logic [3:0] timer_misses;
    logic motor = 0;          // Motor on off register
    logic disk_to_nmi = 0;  // if 1, disk generates nmi
    logic disk_to_int = 0;  // if 1, disk generates int
    logic tc = 0;           // TC signal to reset disk
    logic [7:0] portF0 /*synthesis noprune*/;     // 0x0000-0x3fff page map
    logic [7:0] portF1 /*synthesis noprune*/;     // 0x4000-0x7fff page map
    logic [7:0] portF2 /*synthesis noprune*/;     // 0x8000-0xbfff page map
    logic [7:0] portF3 /*synthesis noprune*/;     // 0xc000-0xffff page map
    logic [7:0] portF4 /*synthesis noprune*/;     // Memory read lock register (CPC only)
    logic [7:0] portF5 /*synthesis noprune*/;     // Roller RAM address
    logic [7:0] portF6 /*synthesis noprune*/;     // Y scroll
    logic [7:0] portF7 /*synthesis noprune*/;     // Inverse / Disable
    logic [7:0] portF8 /*synthesis noprune*/;     // Ntsc / Flyback (read)
	logic [7:0] port80 /*modes pcw+  */;
	logic [7:0] port81 /*bites pcw+ */;
	logic pwc_allow_videomode_change =1;
    logic pcw_last_restart_component =0;
    logic [3:0] pcw_last_index_color_change;
    logic [1:0] pcw_last_index_color_change_component;
    logic [3:0] pcw_video_mode;
    logic [3:0] indice_a_color;
    logic [1:0] componente; 
    logic [23:0] valor_a_cambiar;
    logic [23:0] mascara_quitar;
    logic [23:0] valor_aplicar;
    logic [4:0] rotaciones;
    logic [3:0] temp_cpudo; // Variable temporal para cpudo
	logic [23:0] color_table [33:0];
   logic [23:0] color_256;  
	// Señal para detectar el flanco de bajada de iow (escritura válida)
reg iow_prev;
wire iow_falling_edge = (iow_prev == 1'b0) && (iow == 1'b1);
    initial begin
        // Inicializar la tabla de color_256es con 16 color_256es en formato 24'h
        color_table[0]  = 24'h000000; // Black
        color_table[1]  = 24'h41FF00; // Green
        color_table[2]  = 24'h000000; // Black
        color_table[3]  = 24'h00AA00; // Green
        color_table[4]  = 24'hAA0000; // Red
        color_table[5]  = 24'hAA5500; // Orange
        color_table[6]  = 24'h000000; // Black
        color_table[7]  = 24'h55FF55; // Light Green
        color_table[8]  = 24'hFF5555; // Light Red
        color_table[9]  = 24'hFFFF55; // Yellow
        color_table[10] = 24'h000000; // Black
        color_table[11] = 24'h00AAAA; // Cyan
        color_table[12] = 24'hAA00AA; // Magenta
        color_table[13] = 24'hAAAAAA; // Light Gray
        color_table[14] = 24'h000000; // Black
        color_table[15] = 24'h55FFFF; // Light Cyan
        color_table[16] = 24'hFF55FF; // Light Magenta
        color_table[17] = 24'hFFFFFF; // White
        color_table[18] = 24'h000000; // Black
        color_table[19] = 24'h0000AA; // Dark Blue
        color_table[20] = 24'h00AA00; // Dark Green
        color_table[21] = 24'h00AAAA; // Cyan
        color_table[22] = 24'hAA0000; // Dark Red
        color_table[23] = 24'hAA00AA; // Magenta
        color_table[24] = 24'hAA5500; // Brown
        color_table[25] = 24'hAAAAAA; // Light Gray
        color_table[26] = 24'h555555; // Dark Gray
        color_table[27] = 24'h5555FF; // Light Blue
        color_table[28] = 24'h55FF55; // Light Green
        color_table[29] = 24'h55FFFF; // Light Cyan
        color_table[30] = 24'hFF5555; // Light Red
        color_table[31] = 24'hFF55FF; // Light Magenta
        color_table[32] = 24'hFFFF55; // Yellow
        color_table[33] = 24'hFFFFFF; // White
        iow_prev = 1;

    end
    // Set CPU data in
    always_comb
    begin
        if(~ior)
        begin
            if(cpua[15:0]==16'h01fc) cpudi = model ? daisy_dout : 8'hff; 
            else begin		
                casez(cpua[7:0])
                        8'hf8: cpudi = portF8;
                        8'hf4: cpudi = portF8;      // Timer interrupt counter will also clear
                        8'hfc: cpudi = model ? daisy_dout : 8'hf8;       // Printer Controller
                        8'hfd: cpudi = model ? daisy_dout : 8'hc8;       // Printer Controller
                        8'he0: begin                // Joystick or CPS
                            case(joy_type)
                                JOY_SPECTRAVIDEO: cpudi = {3'b0,joy0[0],joy0[3],joy0[1],joy0[4],joy0[2]}; // Right,Up,Left,Fire,Down
                                JOY_CASCADE: cpudi = {~joy0[4],2'b0,~joy0[3],1'b0,~joy0[2],~joy0[0],~joy0[1]}; // Fire,Up,Down,Right,Left
                                default: cpudi = 8'h00;       // Dart and CPS
                            endcase
                        end
                        // Kempston Mouse
                        8'b110100??, 8'hd4: cpudi = kempston_dout;
                        // AMX Mouse
                        8'b10?000??: cpudi = amx_dout;
                        // DK Tronics sound and joystick controller
                        8'ha9: cpudi = dktronics ? dk_out : 8'hff;
                        // Kempston Joystick
                        8'h9f: cpudi = (joy_type==JOY_KEMPSTON) ? {3'b0,joy0[4:0]} : 8'hff; // Fire,Up,Down,Left,Right
                        // Floppy controller
                        8'b0000000?: cpudi = fdc_dout;    // Floppy read or write
						8'h80 : begin
                             cpudi =port80;
                             //reset_conditional <= 1'b1;  //parece que esto no lo lee 
                             end
						8'h81 :
                         begin 
                                cpudi =port81;
                               // reset_conditional <= 1'b1; //parece que esto no lo lee 
                         end 
                        default: cpudi = 8'hff; 
						// Pcw + dectect¿?

                endcase
            end
        end
        else begin
            cpudi = kbd_sel ? kbd_data : ram_b_dout;
        end
    end

    assign portF8 = {1'b0,vblank,fdc_status_latch,~ntsc,timer_misses};

    logic int_mode_change = 1'b0;

	always @(posedge clk_sys)
	begin
		if(reset  || reset_conditional) begin
			port80 <= 8'h00;
			port81 <= 8'h00;
			pcw_last_index_color_change <= 4'h0;
			pcw_last_index_color_change_component <= 2'h0;
			pcw_video_mode <= 4'h0;
			portF0 <= 8'h80;
			portF1 <= 8'h81;
			portF2 <= 8'h82;
			portF3 <= 8'h83;
			portF4 <= 8'hf1;
			portF5 <= 8'h00;
			portF6 <= 8'h00;
			portF7 <= 8'h80;
            pcw_last_restart_component <= 1'b0;
			disk_to_nmi <= 1'b0;
			disk_to_int <= 1'b0;
			tc <= 1'b0;
			motor <= 1'b0;
			speaker_enable <= 1'b0;
			reset_conditional <= 1'b0;
            iow_prev <= 1;
            pcw_video_mode <= 0;
		end
        iow_prev <= iow;
		int_mode_change <= 1'b0;
		if(~iow  && cpua[7:0]==8'h80 && colorin) begin
			port80 <= cpudo;
			//ver si algo escribe en el 80 algo
			 //pwc_allow_videomode_change <= 1;
			if (cpudo >= 8'h10) begin
				//reset_conditional <= 1'b1;
				pcw_last_index_color_change <= 0;
				pcw_last_index_color_change_component <=0;
			end
		end
		if(iow_falling_edge && cpua[7:0]==8'h81 && colorin) begin
            port81 <= cpudo;
            if (port80 & 8'h20) begin
                // Cambio color_256 paleta mediante índice a color_256es
                indice_a_color = pcw_last_index_color_change;
                if (indice_a_color > 4'hF) indice_a_color = 4'h0;
                case (pcw_video_mode)
					0: valor_a_cambiar = indice_a_color % 2;
					1: valor_a_cambiar = (indice_a_color % 4) + 1;
					2: valor_a_cambiar = (indice_a_color % 16) + 17;
					3: reset_conditional <= 1'b1;
				endcase
				                  case (port81)
            8'd0:    color_256 = 24'h000000;
            8'd1:    color_256 = 24'h000055;
            8'd2:    color_256 = 24'h0000AA;
            8'd3:    color_256 = 24'h0000FF;
            8'd4:    color_256 = 24'h002400;
            8'd5:    color_256 = 24'h002455;
            8'd6:    color_256 = 24'h0024AA;
            8'd7:    color_256 = 24'h0024FF;
            8'd8:    color_256 = 24'h004900;
            8'd9:    color_256 = 24'h004955;
            8'd10:   color_256 = 24'h0049AA;
            8'd11:   color_256 = 24'h0049FF;
            8'd12:   color_256 = 24'h006D00;
            8'd13:   color_256 = 24'h006D55;
            8'd14:   color_256 = 24'h006DAA;
            8'd15:   color_256 = 24'h006DFF;
            8'd16:   color_256 = 24'h009200;
            8'd17:   color_256 = 24'h009255;
            8'd18:   color_256 = 24'h0092AA;
            8'd19:   color_256 = 24'h0092FF;
            8'd20:   color_256 = 24'h00B600;
            8'd21:   color_256 = 24'h00B655;
            8'd22:   color_256 = 24'h00B6AA;
            8'd23:   color_256 = 24'h00B6FF;
            8'd24:   color_256 = 24'h00DB00;
            8'd25:   color_256 = 24'h00DB55;
            8'd26:   color_256 = 24'h00DBAA;
            8'd27:   color_256 = 24'h00DBFF;
            8'd28:   color_256 = 24'h00FF00;
            8'd29:   color_256 = 24'h00FF55;
            8'd30:   color_256 = 24'h00FFAA;
            8'd31:   color_256 = 24'h00FFFF;
            8'd32:   color_256 = 24'h240000;
            8'd33:   color_256 = 24'h240055;
            8'd34:   color_256 = 24'h2400AA;
            8'd35:   color_256 = 24'h2400FF;
            8'd36:   color_256 = 24'h242400;
            8'd37:   color_256 = 24'h242455;
            8'd38:   color_256 = 24'h2424AA;
            8'd39:   color_256 = 24'h2424FF;
            8'd40:   color_256 = 24'h244900;
            8'd41:   color_256 = 24'h244955;
            8'd42:   color_256 = 24'h2449AA;
            8'd43:   color_256 = 24'h2449FF;
            8'd44:   color_256 = 24'h246D00;
            8'd45:   color_256 = 24'h246D55;
            8'd46:   color_256 = 24'h246DAA;
            8'd47:   color_256 = 24'h246DFF;
            8'd48:   color_256 = 24'h249200;
            8'd49:   color_256 = 24'h249255;
            8'd50:   color_256 = 24'h2492AA;
            8'd51:   color_256 = 24'h2492FF;
            8'd52:   color_256 = 24'h24B600;
            8'd53:   color_256 = 24'h24B655;
            8'd54:   color_256 = 24'h24B6AA;
            8'd55:   color_256 = 24'h24B6FF;
            8'd56:   color_256 = 24'h24DB00;
            8'd57:   color_256 = 24'h24DB55;
            8'd58:   color_256 = 24'h24DBAA;
            8'd59:   color_256 = 24'h24DBFF;
            8'd60:   color_256 = 24'h24FF00;
            8'd61:   color_256 = 24'h24FF55;
            8'd62:   color_256 = 24'h24FFAA;
            8'd63:   color_256 = 24'h24FFFF;
            8'd64:   color_256 = 24'h490000;
            8'd65:   color_256 = 24'h490055;
            8'd66:   color_256 = 24'h4900AA;
            8'd67:   color_256 = 24'h4900FF;
            8'd68:   color_256 = 24'h492400;
            8'd69:   color_256 = 24'h492455;
            8'd70:   color_256 = 24'h4924AA;
            8'd71:   color_256 = 24'h4924FF;
            8'd72:   color_256 = 24'h494900;
            8'd73:   color_256 = 24'h494955;
            8'd74:   color_256 = 24'h4949AA;
            8'd75:   color_256 = 24'h4949FF;
            8'd76:   color_256 = 24'h496D00;
            8'd77:   color_256 = 24'h496D55;
            8'd78:   color_256 = 24'h496DAA;
            8'd79:   color_256 = 24'h496DFF;
            8'd80:   color_256 = 24'h499200;
            8'd81:   color_256 = 24'h499255;
            8'd82:   color_256 = 24'h4992AA;
            8'd83:   color_256 = 24'h4992FF;
            8'd84:   color_256 = 24'h49B600;
            8'd85:   color_256 = 24'h49B655;
            8'd86:   color_256 = 24'h49B6AA;
            8'd87:   color_256 = 24'h49B6FF;
            8'd88:   color_256 = 24'h49DB00;
            8'd89:   color_256 = 24'h49DB55;
            8'd90:   color_256 = 24'h49DBAA;
            8'd91:   color_256 = 24'h49DBFF;
            8'd92:   color_256 = 24'h49FF00;
            8'd93:   color_256 = 24'h49FF55;
            8'd94:   color_256 = 24'h49FFAA;
            8'd95:   color_256 = 24'h49FFFF;
            8'd96:   color_256 = 24'h6D0000;
            8'd97:   color_256 = 24'h6D0055;
            8'd98:   color_256 = 24'h6D00AA;
            8'd99:   color_256 = 24'h6D00FF;
            8'd100:  color_256 = 24'h6D2400;
            8'd101:  color_256 = 24'h6D2455;
            8'd102:  color_256 = 24'h6D24AA;
            8'd103:  color_256 = 24'h6D24FF;
            8'd104:  color_256 = 24'h6D4900;
            8'd105:  color_256 = 24'h6D4955;
            8'd106:  color_256 = 24'h6D49AA;
            8'd107:  color_256 = 24'h6D49FF;
            8'd108:  color_256 = 24'h6D6D00;
            8'd109:  color_256 = 24'h6D6D55;
            8'd110:  color_256 = 24'h6D6DAA;
            8'd111:  color_256 = 24'h6D6DFF;
            8'd112:  color_256 = 24'h6D9200;
            8'd113:  color_256 = 24'h6D9255;
            8'd114:  color_256 = 24'h6D92AA;
            8'd115:  color_256 = 24'h6D92FF;
            8'd116:  color_256 = 24'h6DB600;
            8'd117:  color_256 = 24'h6DB655;
            8'd118:  color_256 = 24'h6DB6AA;
            8'd119:  color_256 = 24'h6DB6FF;
            8'd120:  color_256 = 24'h6DDB00;
            8'd121:  color_256 = 24'h6DDB55;
            8'd122:  color_256 = 24'h6DDBAA;
            8'd123:  color_256 = 24'h6DDBFF;
            8'd124:  color_256 = 24'h6DFF00;
            8'd125:  color_256 = 24'h6DFF55;
            8'd126:  color_256 = 24'h6DFFAA;
            8'd127:  color_256 = 24'h6DFFFF;
            8'd128:  color_256 = 24'h920000;
            8'd129:  color_256 = 24'h920055;
            8'd130:  color_256 = 24'h9200AA;
            8'd131:  color_256 = 24'h9200FF;
            8'd132:  color_256 = 24'h922400;
            8'd133:  color_256 = 24'h922455;
            8'd134:  color_256 = 24'h9224AA;
            8'd135:  color_256 = 24'h9224FF;
            8'd136:  color_256 = 24'h924900;
            8'd137:  color_256 = 24'h924955;
            8'd138:  color_256 = 24'h9249AA;
            8'd139:  color_256 = 24'h9249FF;
            8'd140:  color_256 = 24'h926D00;
            8'd141:  color_256 = 24'h926D55;
            8'd142:  color_256 = 24'h926DAA;
            8'd143:  color_256 = 24'h926DFF;
            8'd144:  color_256 = 24'h929200;
            8'd145:  color_256 = 24'h929255;
            8'd146:  color_256 = 24'h9292AA;
            8'd147:  color_256 = 24'h9292FF;
            8'd148:  color_256 = 24'h92B600;
            8'd149:  color_256 = 24'h92B655;
            8'd150:  color_256 = 24'h92B6AA;
            8'd151:  color_256 = 24'h92B6FF;
            8'd152:  color_256 = 24'h92DB00;
            8'd153:  color_256 = 24'h92DB55;
            8'd154:  color_256 = 24'h92DBAA;
            8'd155:  color_256 = 24'h92DBFF;
            8'd156:  color_256 = 24'h92FF00;
            8'd157:  color_256 = 24'h92FF55;
            8'd158:  color_256 = 24'h92FFAA;
            8'd159:  color_256 = 24'h92FFFF;
            8'd160:  color_256 = 24'hB60000;
            8'd161:  color_256 = 24'hB60055;
            8'd162:  color_256 = 24'hB600AA;
            8'd163:  color_256 = 24'hB600FF;
            8'd164:  color_256 = 24'hB62400;
            8'd165:  color_256 = 24'hB62455;
            8'd166:  color_256 = 24'hB624AA;
            8'd167:  color_256 = 24'hB624FF;
            8'd168:  color_256 = 24'hB64900;
            8'd169:  color_256 = 24'hB64955;
            8'd170:  color_256 = 24'hB649AA;
            8'd171:  color_256 = 24'hB649FF;
            8'd172:  color_256 = 24'hB66D00;
            8'd173:  color_256 = 24'hB66D55;
            8'd174:  color_256 = 24'hB66DAA;
            8'd175:  color_256 = 24'hB66DFF;
            8'd176:  color_256 = 24'hB69200;
            8'd177:  color_256 = 24'hB69255;
            8'd178:  color_256 = 24'hB692AA;
            8'd179:  color_256 = 24'hB692FF;
            8'd180:  color_256 = 24'hB6B600;
            8'd181:  color_256 = 24'hB6B655;
            8'd182:  color_256 = 24'hB6B6AA;
            8'd183:  color_256 = 24'hB6B6FF;
            8'd184:  color_256 = 24'hB6DB00;
            8'd185:  color_256 = 24'hB6DB55;
            8'd186:  color_256 = 24'hB6DBAA;
            8'd187:  color_256 = 24'hB6DBFF;
            8'd188:  color_256 = 24'hB6FF00;
            8'd189:  color_256 = 24'hB6FF55;
            8'd190:  color_256 = 24'hB6FFAA;
            8'd191:  color_256 = 24'hB6FFFF;
            8'd192:  color_256 = 24'hDB0000;
            8'd193:  color_256 = 24'hDB0055;
            8'd194:  color_256 = 24'hDB00AA;
            8'd195:  color_256 = 24'hDB00FF;
            8'd196:  color_256 = 24'hDB2400;
            8'd197:  color_256 = 24'hDB2455;
            8'd198:  color_256 = 24'hDB24AA;
            8'd199:  color_256 = 24'hDB24FF;
            8'd200:  color_256 = 24'hDB4900;
            8'd201:  color_256 = 24'hDB4955;
            8'd202:  color_256 = 24'hDB49AA;
            8'd203:  color_256 = 24'hDB49FF;
            8'd204:  color_256 = 24'hDB6D00;
            8'd205:  color_256 = 24'hDB6D55;
            8'd206:  color_256 = 24'hDB6DAA;
            8'd207:  color_256 = 24'hDB6DFF;
            8'd208:  color_256 = 24'hDB9200;
            8'd209:  color_256 = 24'hDB9255;
            8'd210:  color_256 = 24'hDB92AA;
            8'd211:  color_256 = 24'hDB92FF;
            8'd212:  color_256 = 24'hDBB600;
            8'd213:  color_256 = 24'hDBB655;
            8'd214:  color_256 = 24'hDBB6AA;
            8'd215:  color_256 = 24'hDBB6FF;
            8'd216:  color_256 = 24'hDBDB00;
            8'd217:  color_256 = 24'hDBDB55;
            8'd218:  color_256 = 24'hDBDBAA;
            8'd219:  color_256 = 24'hDBDBFF;
            8'd220:  color_256 = 24'hDBFF00;
            8'd221:  color_256 = 24'hDBFF55;
            8'd222:  color_256 = 24'hDBFFAA;
            8'd223:  color_256 = 24'hDBFFFF;
            8'd224:  color_256 = 24'hFF0000;
            8'd225:  color_256 = 24'hFF0055;
            8'd226:  color_256 = 24'hFF00AA;
            8'd227:  color_256 = 24'hFF00FF;
            8'd228:  color_256 = 24'hFF2400;
            8'd229:  color_256 = 24'hFF2455;
            8'd230:  color_256 = 24'hFF24AA;
            8'd231:  color_256 = 24'hFF24FF;
            8'd232:  color_256 = 24'hFF4900;
            8'd233:  color_256 = 24'hFF4955;
            8'd234:  color_256 = 24'hFF49AA;
            8'd235:  color_256 = 24'hFF49FF;
            8'd236:  color_256 = 24'hFF6D00;
            8'd237:  color_256 = 24'hFF6D55;
            8'd238:  color_256 = 24'hFF6DAA;
            8'd239:  color_256 = 24'hFF6DFF;
            8'd240:  color_256 = 24'hFF9200;
            8'd241:  color_256 = 24'hFF9255;
            8'd242:  color_256 = 24'hFF92AA;
            8'd243:  color_256 = 24'hFF92FF;
            8'd244:  color_256 = 24'hFFB600;
            8'd245:  color_256 = 24'hFFB655;
            8'd246:  color_256 = 24'hFFB6AA;
            8'd247:  color_256 = 24'hFFB6FF;
            8'd248:  color_256 = 24'hFFDB00;
            8'd249:  color_256 = 24'hFFDB55;
            8'd250:  color_256 = 24'hFFDBAA;
            8'd251:  color_256 = 24'hFFDBFF;
            8'd252:  color_256 = 24'hFFFF00;
            8'd253:  color_256 = 24'hFFFF55;
            8'd254:  color_256 = 24'hFFFFAA;
            8'd255:  color_256 = 24'hFFFFFF;
            default: color_256 = 24'h000000; // Por defecto, negro
        endcase
		               color_table[valor_a_cambiar]  =color_256;
                pcw_last_index_color_change <= pcw_last_index_color_change + 1;
               // if (pcw_last_index_color_change >= 4'h10) pcw_last_index_color_change <= 4'h0;
            end else     if (port80 & 8'h10) begin
				// Cambio color paleta mediante RGB
				indice_a_color = pcw_last_index_color_change ; // Adjust index if needed
				case (pcw_video_mode)
					0: valor_a_cambiar = indice_a_color % 2;
					1: valor_a_cambiar = (indice_a_color % 4) + 2;
					2: valor_a_cambiar = (indice_a_color % 16) + 18;
					3: reset_conditional <= 1'b1;
				endcase
               


				componente = pcw_last_index_color_change_component;

				// Calculate the mask and shift value for the current component
				rotaciones = componente * 8; // 0, 8, or 16 for R, G, B
				mascara_quitar = ~(24'hFF << rotaciones); // Mask to clear the current component
				valor_aplicar = cpudo << rotaciones; // Shift the new value to the correct position

				// Update only the current component in the color_256 table
				color_table[valor_a_cambiar] = (color_table[valor_a_cambiar] & mascara_quitar) | valor_aplicar;

				// Increment the component counter
				if (pcw_last_index_color_change_component >= 2'h2) begin
					pcw_last_index_color_change_component <= 2'h0; // Wrap around after 2
					pcw_last_index_color_change <= pcw_last_index_color_change + 1;
					if (pcw_last_index_color_change >= 4'h0f) pcw_last_index_color_change <= 4'h0;
				end else begin
					pcw_last_index_color_change_component <= pcw_last_index_color_change_component + 1;
				end
            end else begin
                // Cambio modo
                temp_cpudo = cpudo[3:0]; // Usar una variable temporal
                if (temp_cpudo >= 4'h4) temp_cpudo = 4'h0;
                pcw_video_mode <= temp_cpudo;
/*                 if (temp_cpudo == 4'h2) begin
                    reset_conditional <= 1'b1; // llega exito
                end */
            end
        end 
        if(~iow && cpua[7:0]==8'hf0) portF0 <= cpudo;
        if(~iow && cpua[7:0]==8'hf1) portF1 <= cpudo;
        if(~iow && cpua[7:0]==8'hf2) portF2 <= cpudo;
        if(~iow && cpua[7:0]==8'hf3) portF3 <= cpudo;
        if(~iow && cpua[7:0]==8'hf4) portF4 <= cpudo;
        if(~iow && cpua[7:0]==8'hf5) portF5 <= cpudo;
        if(~iow && cpua[7:0]==8'hf6) portF6 <= cpudo;
        if(~iow && cpua[7:0]==8'hf7) portF7 <= cpudo;
        if(~iow && cpua[7:0]==8'hf8) 
        begin
            // decode command for System Control Register
            if(cpudo[3:0] == 4'd0) ; // Terminate bootstrap (do nothing)
           // else if(cpudo[3:0] == 4'd1) reset <= 1'b1;  // System Reset
            else if(cpudo[3:0] == 4'd2) begin           // Disk to NMI
                disk_to_nmi <= 1'b1;
                disk_to_int <= 1'b0;
                //int_mode_change <= 1'b1;
            end
            else if(cpudo[3:0] == 4'd3) begin           // Disk to INT
                disk_to_int <= 1'b1;
                disk_to_nmi <= 1'b0;
                int_mode_change <= 1'b1;
            end
            else if(cpudo[3:0] == 4'd4) begin           // Disconnect Disk Int/NMI
                disk_to_int <= 1'b0;
                disk_to_nmi <= 1'b0;
                int_mode_change <= 1'b1;
            end
            else if(cpudo[3:0] == 4'd5) begin           // Set FDC TC
                tc <= 1'b1;
            end            
            else if(cpudo[3:0] == 4'd6) begin           // Clear FDC TC
                tc <= 1'b0;
            end            
            else if(cpudo[3:0] == 4'd9) motor <= 1'b1;
            else if(cpudo[3:0] == 4'd10) motor <= 1'b0;
            else if(cpudo[3:0] == 4'd11) speaker_enable <= 1'b1;
            else if(cpudo[3:0] == 4'd12) speaker_enable <= 1'b0;
            //if(img_mounted) motor <= 0; // Reset on new image mounted
        end



    end

    // logic old_GCLK, old_ior;
    // always @(posedge clk_sys)
    // begin
    //     old_GCLK <= GCLK;
    //     if(~old_GCLK & GCLK)
    //     begin
    //         old_ior <= ior;
    //         if(old_ior & ~ior & (cpua[7:0]==8'h00 || cpua[7:0]==8'h01)) waitio <= 1'b1;
    //         else waitio <= 1'b0;
    //     end
    // end

    // detect fdc interrupt edge
    logic fdc_pe, fdc_ne;
    edge_det fdc_edge_det(.clk_sys(clk_sys), .signal(fdc_int), .pos_edge(fdc_pe), .neg_edge(fdc_ne));
    //  Drive FDC status latch (portF8) and NMI flag
    logic fdc_status_latch = 1'b0;
//    logic clear_fdc_status = 1'b0;
    logic clear_nmi_flag = 1'b0;
    logic nmi_flag = 1'b0;
    always @(posedge clk_sys)
    begin
        if(fdc_pe) 
        begin
            fdc_status_latch <= 1'b1;
            if(disk_to_nmi) nmi_flag <= 1'b1;
        end
        else if(fdc_ne) fdc_status_latch <= 1'b0;
//        if(clear_fdc_status) fdc_status_latch <= 1'b0;
        if(clear_nmi_flag) nmi_flag <= 1'b0;
    end

    // // Detect interrupts being re-enabled by the cpu via iff1
    // logic iff1_pe;
    // edge_det int_enable_edge_det(.clk_sys(clk_sys), .signal(iff1), .pos_edge(iff1_pe));

    // Detect timer interrupt firing from video controller (300 hz)
    logic timer_pe;
    logic vid_timer;
    edge_det timer_edge_det(.clk_sys(clk_sys), .signal(vid_timer), .pos_edge(timer_pe));
     // Detect int_mode_change edge
    logic int_mode_pe, int_mode_ne;
    edge_det int_mode_edge_det(.clk_sys(clk_sys), .signal(int_mode_change), .pos_edge(int_mode_pe), .neg_edge(int_mode_ne));

    logic timer_line = 1'b0;
    logic int_line = 1'b0;
    logic nmi_line = 1'b0;
    logic clear_timer = 1'b0;
    logic [4:0] clear_timer_count = 'b0;   // 32 count cycle to clear timer after read
    // Timer flag and interrupt flag drivers
    always @(posedge clk_sys)
    begin
        if(timer_pe) 
        begin
            // Timer count and int line processing
            if(!(&timer_misses)) timer_misses <= timer_misses + 4'b1;
            if(~iff1) timer_line <= 1'b0;
            else timer_line <= 1'b1;
            // NMI line processing occurs on timer
            if(nmi_flag) nmi_line <= 1'b1;  // Trigger or hold NMI high
            else nmi_line <= 1'b0;
            // INT line processing
            if(disk_to_int & fdc_status_latch & iff1) int_line <= 1'b1;
            else int_line <= 1'b0;
        end
        // Detect clear timer start
        if(~ior & cpua[7:0]==8'hf4) 
        begin
            clear_timer <= 1'b1; // Clear timer
            clear_timer_count <= 'b0;
        end

		
        // Clear timer processing
        if(clear_timer)
        begin
            if(&clear_timer_count)
            begin
                // Reached top so clear flag
                clear_timer <= 1'b0;
                timer_line <= 1'b0;
                timer_misses <= 'b0;
            end
            else clear_timer_count <= clear_timer_count + 4'd1;
        end
        // Clear interrupts
        if(int_mode_pe)
        begin
            if(disk_to_nmi) int_line <= 1'b0;
            else if(disk_to_int) clear_nmi_flag <= 1'b1;    // Disk to int clears nmi
            else begin
                clear_nmi_flag <= 1'b1;
                int_line <= 1'b0;      // Else clear both
            end
        end 
        else begin
            clear_nmi_flag <= 1'b0;
        end
    end

	logic nmi_sig/* synthesis keep */, int_sig/* synthesis keep */;
    assign nmi_sig = ~nmi_line;
    // Disk int and timer int combined
    assign int_sig = nmi_line ? 1'b1 : (~int_line & ~timer_line);   // Don't fire if NMI outstanding

    // Video control registers
    logic [7:0] roller_ptr;
    logic [7:0] yscroll;
    logic inverse;
    logic disable_vid;
    assign roller_ptr = portF5;
    assign yscroll = portF6;
    assign inverse = portF7[7];
    assign disable_vid = ~portF7[6]; // & ~portF8[3];

    // Ram B address for various paging modes
    logic [20:0] pcw_ram_b_addr/* synthesis keep */;
    logic [17:0] cpc_read_ram_b_addr/* synthesis keep */;
    logic [17:0] cpc_write_ram_b_addr/* synthesis keep */;

    // Memory size adjusted ports
    logic [6:0] mportF0,mportF1,mportF2,mportF3;
    always_comb
    begin 
        case(memory_size)
            MEM_256K: begin
                mportF0 = {3'b0,portF0[3:0]};
                mportF1 = {3'b0,portF1[3:0]};
                mportF2 = {3'b0,portF2[3:0]};
                mportF3 = {3'b0,portF3[3:0]};
            end
            MEM_512K: begin
                mportF0 = {2'b0,portF0[4:0]};
                mportF1 = {2'b0,portF1[4:0]};
                mportF2 = {2'b0,portF2[4:0]};
                mportF3 = {2'b0,portF3[4:0]};
            end
            MEM_1M: begin
                mportF0 = {1'b0,portF0[5:0]};
                mportF1 = {1'b0,portF1[5:0]};
                mportF2 = {1'b0,portF2[5:0]};
                mportF3 = {1'b0,portF3[5:0]};
            end
            MEM_2M: begin
                mportF0 = portF0[6:0];
                mportF1 = portF1[6:0];
                mportF2 = portF2[6:0];
                mportF3 = portF3[6:0];
            end
       endcase
    end

    // PCW Paged memory support for read and writes
    always_comb
    begin
        case(cpua[15:14])
            2'b00: pcw_ram_b_addr = {mportF0,cpua[13:0]};
            2'b01: pcw_ram_b_addr = {mportF1,cpua[13:0]};
            2'b10: pcw_ram_b_addr = {mportF2,cpua[13:0]};
            2'b11: pcw_ram_b_addr = {mportF3,cpua[13:0]};
        endcase
    end

    // CPC Paged memory support for reads
    always_comb
    begin
        case(cpua[15:14])
            2'b00: cpc_read_ram_b_addr = portF4[4] ? {1'b0,portF0[2:0],cpua[13:0]} : {1'b0,portF0[6:4],cpua[13:0]};
            2'b01: cpc_read_ram_b_addr = portF4[5] ? {1'b0,portF1[2:0],cpua[13:0]} : {1'b0,portF1[6:4],cpua[13:0]};
            2'b10: cpc_read_ram_b_addr = portF4[6] ? {1'b0,portF2[2:0],cpua[13:0]} : {1'b0,portF2[6:4],cpua[13:0]};
            2'b11: cpc_read_ram_b_addr = portF4[7] ? {1'b0,portF3[2:0],cpua[13:0]} : {1'b0,portF3[6:4],cpua[13:0]};
        endcase
    end

    // CPC Paged memory support for writes
    always_comb
    begin
        case(cpua[15:14])
            2'b00: cpc_write_ram_b_addr = {1'b0,portF0[2:0],cpua[13:0]};
            2'b01: cpc_write_ram_b_addr = {1'b0,portF1[2:0],cpua[13:0]};
            2'b10: cpc_write_ram_b_addr = {1'b0,portF2[2:0],cpua[13:0]};
            2'b11: cpc_write_ram_b_addr = {1'b0,portF3[2:0],cpua[13:0]};
        endcase
    end

    // Finally memory address based upon above page modes
    always_comb
    begin
        case(cpua[15:14])
            2'b00: ram_b_addr = portF0[7] ? pcw_ram_b_addr : ~memw ? {3'b0,cpc_write_ram_b_addr} : {3'b0,cpc_read_ram_b_addr}; 
            2'b01: ram_b_addr = portF1[7] ? pcw_ram_b_addr : ~memw ? {3'b0,cpc_write_ram_b_addr} : {3'b0,cpc_read_ram_b_addr}; 
            2'b10: ram_b_addr = portF2[7] ? pcw_ram_b_addr : ~memw ? {3'b0,cpc_write_ram_b_addr} : {3'b0,cpc_read_ram_b_addr}; 
            2'b11: ram_b_addr = portF3[7] ? pcw_ram_b_addr : ~memw ? {3'b0,cpc_write_ram_b_addr} : {3'b0,cpc_read_ram_b_addr}; 
        endcase
    end

    // DPRAM - Only support 256K of memory while using dpram
    // Addresses are made up of 4 bits of page number and 14 bits of offset
    // Port A is used for display memory access but can only access 128k
    logic [7:0] dpram_b_dout;
    dpram #(.DATA(8), .ADDR(17)) main_mem(
        // Port A is used for display memory access
        .a_clk(clk_sys),
        .a_wr(1'b0),        // Video never writes to display memory
        .a_addr({1'b0,ram_a_addr}),
        .a_din('b0),
        .a_dout(ram_a_dout),

        // Port B - used for CPU and download access
        .b_clk(clk_sys),
        .b_wr(dn_go ? dn_wr : ~memw & ~|ram_b_addr[20:17]),
        .b_addr(dn_go ? dn_addr[16:0] : ram_b_addr[16:0]),
        .b_din(dn_go ? dn_data : cpudo),
        .b_dout(dpram_b_dout)
    );

    logic ram_ready;
    logic [7:0] sdram_b_dout;
    // Extended SDRAM for memory above 256K.  2MB in size, but first 256K will not be used
    sdram sdram
    (
        .*,
        .init(~locked),
        .clk(clk_sys),
        .dout(sdram_b_dout),
        .din (cpudo),
        .addr(ram_b_addr),
        .we(~memw & sdram_access), 
        .rd(~memr & sdram_access),
        .ready(ram_ready)
    );

    //wire sdram_access = |ram_b_addr[20:18] && memory_size > MEM_256K;
	wire sdram_access = |ram_b_addr[20:17];// && memory_size > MEM_256K;
    assign ram_b_dout = sdram_access ? sdram_b_dout : dpram_b_dout;

    // Edge detectors for moving fake pixel line using F9 and F10 keys
    logic line_up_pe, line_down_pe, toggle_pe;
    edge_det line_up_edge_det(.clk_sys(clk_sys), .signal(line_up), .pos_edge(line_up_pe));
    edge_det line_down_edge_det(.clk_sys(clk_sys), .signal(line_down), .pos_edge(line_down_pe));
    edge_det toggle_full_edge_det(.clk_sys(clk_sys), .signal(toggle_full), .pos_edge(toggle_pe));
    // Line position of fake colour line
    logic [7:0] fake_end;
        always @(posedge clk_sys)
    begin
        if(reset) fake_end <= 8'd0;
        else begin
            if(line_up_pe && fake_end > 0) fake_end <= fake_end - 8'd1;
            if(line_down_pe && fake_end < 255) fake_end <= fake_end + 8'd1;
            if(toggle_pe) begin
                if(fake_end==8'd255) fake_end <= 8'd0;
                else if(fake_end==8'd0) fake_end <= 8'd255;
                else fake_end <= 8'd0;
            end
            // Writen to via a write to port FF
            if(~iow && cpua[7:0]==8'hff) fake_end <= cpudo;
        end
    end

    logic [3:0] colour;

    logic [23:0] rgb_white;
    logic [23:0] rgb_green;
    logic [23:0] rgb_amber;

    logic cpu_reg_set = 1'b0;
    logic [211:0] cpu_reg = 'b0;
    logic [211:0] cpu_reg_out;
    logic [7:0] ypos;

    // Video output controller
    video_controller video(
        .reset(reset),
        .clk_sys(clk_sys),
        .roller_ptr(roller_ptr),
        .yscroll(yscroll),
        .inverse(inverse),
        .disable_vid(disable_vid),
        .ntsc(ntsc),
        .fake_colour(fake_colour),
        .pcw_video_mode(pcw_video_mode),
        .fake_end(fake_end),
        .ypos(ypos),

        .vid_addr(ram_a_addr),
        .din(ram_a_dout),

        .colour(colour),
        .ce_pix(ce_pix),
        .hsync(hsync),
        .vsync(vsync),
        .hb(hblank),
        .vb(vblank),
        .timer_int(vid_timer)
    );

    // Head over heals duplicate writes to 0xc000-c1ff for debugging
    // Clone SD writes into sd_debug for memory debugging using in system member debugger in Quartus
    // logic [7:0] debug_vid /*synthesis noprune*/;
    // logic c000_range;
    // assign c000_range = (ram_b_addr >= 18'hc000 && ram_b_addr < 18'hcfff) & ~memw & GCLK;
    // sd_debug vid_debug(
    //     .clock(clk_sys),
    //     .address(ram_b_addr[11:0]),
    //     .data(cpudo),
    //     .wren(c000_range),
    //     .q(debug_vid)
    // );

    // Video colour processing
    always_comb begin
        rgb_white = 24'hAAAAAA;
        if(colour==4'b0000) rgb_white = 24'h000000;
        else if(colour==4'b1111) rgb_white = 24'hFFFFFF;
    end

    always_comb begin
        rgb_green = 24'h00aa00;
        if(colour==4'b0000) rgb_green = 24'h000000;
        else if(colour==4'b1111) rgb_green = 24'h00aa00;
    end

    always_comb begin
        rgb_amber = 24'hff5500;
        if(colour==4'b0000) rgb_amber = 24'h000000;
        else if(colour==4'b1111) rgb_amber = 24'hff5500;
    end

    logic [23:0] mono_colour;
    always_comb begin
        if(disp_color==2'b00) mono_colour = rgb_white;
        else if(disp_color==2'b01) mono_colour = rgb_green;
        else if(disp_color==2'b10) mono_colour= rgb_amber;
        else mono_colour = rgb_white;
    end

    always_comb begin
        RGB = mono_colour;
        if(fake_colour && ypos < fake_end) begin
            case(fake_colour_mode)
                3'b000: RGB = mono_colour;
                3'b001: begin    // CGA Palette 0 Low
                    case(colour[3:2])
                        //2'b00: RGB =  24'h000000;   // Black
					    //2'b01: RGB =  24'h00aa00;   // Green
                        //2'b10: RGB =  24'haa0000;   // Red
						//2'b11: RGB =  24'haa5500;   // Brown
						2'b00: RGB =  color_table[2];   // Black
					    2'b01: RGB =  color_table[3];   // Green
                        2'b10: RGB =  color_table[4];   // Red
						2'b11: RGB =  color_table[5];   // Brown
                    endcase                    
                end
                3'b010: begin    // CGA Palette 0 High
                    case(colour[3:2])
                        //2'b00: RGB =  24'h000000;   // Black
                        //2'b01: RGB =  24'h55ff55;   // Light Green
                        //2'b10: RGB =  24'hff5555;   // Light Red
						//2'b11: RGB =  24'hffff55;   // Yellow
                        2'b00: RGB =  color_table[6];   // Black
                        2'b01: RGB =  color_table[7];   // Light Green
                        2'b10: RGB =  color_table[8];   // Light Red
						2'b11: RGB =  color_table[9];   // Yellow
                    endcase                    
                end
                3'b011: begin    // CGA Palette 1 low
                    case(colour[3:2])
                        //2'b00: RGB =  24'h000000;   // Black
                        //2'b01: RGB =  24'h00aaaa;   // Cyan
                        //2'b10: RGB =  24'haa00aa;   // Magenta
                        //2'b11: RGB =  24'haaaaaa;   // light grey
								2'b00: RGB =  color_table[10];   // Black
                        2'b01: RGB =  color_table[11];   // Cyan
                        2'b10: RGB =  color_table[12];   // Magenta
                        2'b11: RGB =  color_table[13];   // light grey
                    endcase 
						end 
					3'b100: begin    // CGA Palette 1 High
                    case(colour[3:2])
                        2'b00: RGB =  24'h000000;   // Black
                        //2'b01: RGB =  24'h55ffff;   // Light Cyan
                        //2'b10: RGB =  24'hff55ff;   // Light Magenta
                        //2'b11: RGB =  24'hffffff;   // White
								2'b00: RGB = color_table[14];   // Black
                        2'b01: RGB =  color_table[15];   // Light Cyan
                        2'b10: RGB =  color_table[16];   // Light Magenta
                        2'b11: RGB =  color_table[17];   // White
                    endcase 
					 end 
					 3'b101: begin   // color_256in	
							if (pcw_video_mode ==0) RGB = mono_colour;
							else if (pcw_video_mode ==1) begin 
								case(colour[3:2])
									2'b00: RGB =  color_table[2];   
									2'b01: RGB =  color_table[3];
									2'b10: RGB =  color_table[4];
									2'b11: RGB =  color_table[5];
								endcase
							end else if (pcw_video_mode ==2) begin
								case(colour[3:0])                            
 									4'b0000: RGB =  color_table[18];
									4'b0001: RGB =  color_table[19];
									4'b0010: RGB =  color_table[20];
									4'b0011: RGB =  color_table[21];
									4'b0100: RGB =  color_table[22];
									4'b0101: RGB =  color_table[23];
									4'b0110: RGB =  color_table[24];
									4'b0111: RGB =  color_table[25];
									4'b1000: RGB =  color_table[26];
									4'b1001: RGB =  color_table[27];
									4'b1010: RGB =  color_table[28];
									4'b1011: RGB =  color_table[29];
									4'b1100: RGB =  color_table[30];
									4'b1101: RGB =  color_table[31];
									4'b1110: RGB =  color_table[32];
									4'b1111: RGB =  color_table[33];
  /*                    4'b0000: RGB =  24'h000000;   // Black
                        4'b0001: RGB =  24'h0000aa;   // Blue
                        4'b0010: RGB =  24'h00aa00;   // Green
                        4'b0011: RGB =  24'h00aaaa;   // Cyan
                        4'b0100: RGB =  24'haa0000;   // Red
                        4'b0101: RGB =  24'haa00aa;   // Magenta
                        4'b0110: RGB =  24'haa5500;   // Yellow Brown
                        4'b0111: RGB =  24'haaaaaa;   // White / gray
                        4'b1000: RGB =  24'h555555;   // dark gray
                        4'b1001: RGB =  24'h5555ff;   // Light blue
                        4'b1010: RGB =  24'h55ff55;   // Light green
                        4'b1011: RGB =  24'h55ffff;   // light cyan
                        4'b1100: RGB =  24'hff5555;   // light red
                        4'b1101: RGB =  24'hff55ff;   // Light magenta
                        4'b1110: RGB =  24'hffff55;   // Light yellow
                        4'b1111: RGB =  24'hffffff;   // bright white */
								endcase 
							end
						end
            endcase
        end
    end

    logic [7:0] daisy_dout;
    // Fake daisywheel printer interface
    fake_daisy daisy(
        .reset(reset),
        .clk_sys(clk_sys),
        .ce(cpuclk),
        .sel(daisy_sel),
        .address({cpua[8],cpua[0]}),
        .wr(~iow),
        .din(cpudo),
        .dout(daisy_dout)
    );

    // Mouse emulation
    logic mouse_left, mouse_middle, mouse_right;
    logic signed [8:0] mouse_x, mouse_y;
    mouse mouse(
        .*
    );

    // AMX mouse driver
    logic [7:0] amx_dout;
    wire amx_sel = ~ior && (cpua[7:2]==6'b101000 || cpua[7:2]==6'b100000) && mouse_type==MOUSE_AMX;
    amx_mouse amx_mouse(
        .sel(amx_sel),
        .addr(cpua[1:0]),
        .dout(amx_dout),
        .*
    );

    // Kempston mouse driver
    logic [7:0] kempston_dout;
    wire kempston_sel = ~ior && (cpua[7:0] ==? 8'b110100?? || cpua[7:0]==8'hd4) && mouse_type==MOUSE_KEMPSTON;
    kempston_mouse kempston_mouse(
        .sel(kempston_sel),
        .addr(cpua[2:0]),
        .dout(kempston_dout),
        .input_pulse(ps2_mouse[24]),
        .*
    );

    // Keyboard / Joystick controller
    logic line_up, line_down;   // line up and down signals for moving fake colour
    logic toggle_full;          // Toggle full screen colour on / off
    logic [7:0] kbd_data;
    key_joystick keyjoy(
        .reset(reset),
        .clk_sys(clk_sys),
        .ps2_key(ps2_key),
        .joy0(joy0),
        .joy1(joy1),
        .lk1(1'b0),
        .lk2(1'b0),
        .lk3(1'b0),
        .addr(cpua[3:0]),
        .key_data(kbd_data),
        .keymouse(mouse_type==MOUSE_KEYMOUSE),
        .mouse_pulse(ps2_mouse[24]),
        .line_up(line_up),
        .line_down(line_down),
        .toggle_full(toggle_full),
        .*          // Mouse inputs
    ); 

    // DKtronics sound and joystick interface
    // {3'b0,joy0[4:0]} : 8'hff; // Fire,Up,Down,Left,Right
    logic [7:0] dkjoy_io;
    assign dkjoy_io = {1'b1,~joy0[4],~joy0[3],~joy0[2],~joy0[0],~joy0[1],2'b11};

    logic dk_busdir, dk_bc;
    always_comb
    begin
        if(~ior & cpua[7:0]==8'ha9) {dk_busdir,dk_bc} <= 2'b01;         // Port A9 - Read Register
        else if(~iow & cpua[7:0]==8'haa) {dk_busdir,dk_bc} <= 2'b11;    // Port AA - Write Address
        else if(~iow & cpua[7:0]==8'hab) {dk_busdir,dk_bc} <= 2'b10;    // Port AB - Write Register
        else {dk_busdir,dk_bc} <= 2'b00;
    end 

    logic [7:0] dk_out;
    // Audio processing
    ym2149 soundchip(
        .DI(cpudo),
        .DO(dk_out),
		.BDIR(dk_busdir),
        .BC(dk_bc),
        .SEL(1'b0),
        .MODE(1'b0),
	    .CHANNEL_A(ch_a),
        .CHANNEL_B(ch_b),
        .CHANNEL_C(ch_c),
	      .IOA_in(dkjoy_io),
        .CE(snd_clk & dktronics),
        .RESET(reset),
        .CLK(clk_sys)

    ); 

    // Bleeper audio
    bleeper bleeper(
        .clk_sys(clk_sys),
        .ce(speaker_enable),
        .speaker(speaker_out)
    );

    logic [7:0] speaker = 'b0;
    logic speaker_out;
    assign speaker = {speaker_out, 6'b0};
    assign audio = {2'b00,ch_a} + {2'b00,ch_b} + {2'b00,ch_c} + {2'b00,speaker};
    assign audiomix = audio[9:1];


    // Floppy disk controller logic and control
    wire fdc_sel = {~cpua[7]};
    
    //wire [7:0] u765_dout;
    wire [7:0] fdc_dout;// = (fdc_sel & ~ior) ? u765_dout : 8'hFF;

    reg  [1:0] u765_ready = 0;
    always @(posedge clk_sys) if(img_mounted[0]) u765_ready[0] <= |img_size;
    always @(posedge clk_sys) if(img_mounted[1]) u765_ready[1] <= |img_size;

    logic fdc_int;
    u765 u765
    (
        .reset(reset),

        .clk_sys(clk_sys),
        .ce(disk_clk),
        .a0(cpua[0]),
        .ready(u765_ready),
        .motor({motor,motor}),
        .available(2'b11),
        .nRD(~fdc_sel | ior), 
        .nWR(~fdc_sel | iow),
        .din(cpudo),
        .dout(fdc_dout),
        .int_out(fdc_int),
        .tc(tc),
        .density(density),
        .activity_led(LED),

        .img_mounted(img_mounted),
        .img_size(img_size[31:0]),
        .img_wp(img_readonly),
        .sd_lba(sd_lba),
        .sd_rd(sd_rd),
        .sd_wr(sd_wr),
        .sd_ack(sd_ack),
        .sd_buff_addr(sd_buff_addr),
        .sd_buff_dout(sd_buff_dout),
        .sd_buff_din(sd_buff_din),
        .sd_buff_wr(sd_dout_strobe)
    );


endmodule

