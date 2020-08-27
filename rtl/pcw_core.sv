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

    output logic [17:0] RGB,    // RGB Output (6-6-6)     
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
//    logic waitio = 1'b0;

    logic [3:0] rgbi;

    logic [7:0] speaker = 'b0;
    logic [17:0] rgb_white;
    logic [17:0] rgb_green;
    logic [17:0] rgb_amber;

    logic cpu_reg_set = 1'b0;
    logic [211:0] cpu_reg = 'b0;
    logic [211:0] cpu_reg_out;
    

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
                        8'ha9: cpudi = dk_out;      
                        // Kempston Joystick
                        8'h9f: cpudi = (joy_type==JOY_KEMPSTON) ? {3'b0,joy0[4:0]} : 8'hff; // Fire,Up,Down,Left,Right
                        // Floppy controller
                        8'b0000000?: cpudi = fdc_dout;    // Floppy read or write
                        default: cpudi = 8'hff;             
                endcase
            end
        end
        else begin
            cpudi = kbd_sel ? kbd_data : ram_b_dout;
        end
    end

    assign portF8 = {1'b0,vblank,fdc_status_latch,~ntsc,timer_misses};

    logic int_mode_change = 1'b0;
    // IO writing to register ports
    always @(posedge clk_sys)
    begin
        if(reset)
        begin
            portF0 <= 8'h80;
            portF1 <= 8'h81;
            portF2 <= 8'h82;
            portF3 <= 8'h83;
            portF4 <= 8'hf1;
            portF5 <= 8'h00;
            portF6 <= 8'h00;
            portF7 <= 8'h80;
            disk_to_nmi <= 1'b0;
            disk_to_int <= 1'b0;
            tc <= 1'b0;
            motor <= 1'b0;
            speaker_enable <= 1'b0;
        end
        int_mode_change <= 1'b0;

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
    dpram #(.DATA(8), .ADDR(18)) main_mem(
        // Port A is used for display memory access
        .a_clk(clk_sys),
        .a_wr(1'b0),        // Video never writes to display memory
        .a_addr({1'b0,ram_a_addr}),
        .a_din('b0),
        .a_dout(ram_a_dout),

        // Port B - used for CPU and download access
        .b_clk(clk_sys),
        .b_wr(dn_go ? dn_wr : ~memw & ~|ram_b_addr[20:18]),
        .b_addr(dn_go ? dn_addr[17:0] : ram_b_addr[17:0]),
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

    wire sdram_access = |ram_b_addr[20:18] && memory_size > MEM_256K;
    assign ram_b_dout = sdram_access ? sdram_b_dout : dpram_b_dout;

    // Video output controller
    video_controller video(
        .reset(reset),
        .clk_sys(clk_sys),
        .roller_ptr(roller_ptr),
        .yscroll(yscroll),
        .inverse(inverse),
        .disable_vid(disable_vid),
        .ntsc(ntsc),

        .vid_addr(ram_a_addr),
        .din(ram_a_dout),

        .rgbi(rgbi),
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
        rgb_white = 18'b111110111110111110;
        if(rgbi==4'b0000) rgb_white = 18'b000000000000000000;
        else if(rgbi==4'b1000) rgb_white = 18'b110111111111111111;
    end

    always_comb begin
        rgb_green = 18'b000000111110000000;
        if(rgbi==4'b0000) rgb_green = 18'b000000000000000000;
        else if(rgbi==4'b1000) rgb_green = 18'b011001111111011001;
    end

    always_comb begin
        rgb_amber = 18'b000000011111111110;
        if(rgbi==4'b0000) rgb_amber = 18'b000000000000000000;
        else if(rgbi==4'b1000) rgb_amber = 18'b000000101100111111;
    end

    always_comb begin
        RGB = 18'b111110111110111110;
        if(disp_color==2'b00) RGB = rgb_white;
        else if(disp_color==2'b01) RGB = rgb_green;
        else if(disp_color==2'b10) RGB = rgb_amber;
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

        .CE(snd_clk),
        .RESET(reset),
        .CLK(clk_sys)
    ); 

    // Bleeper audio
    bleeper bleeper(
        .clk_sys(clk_sys),
        .ce(speaker_enable),
        .speaker(speaker_out)
    );

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

