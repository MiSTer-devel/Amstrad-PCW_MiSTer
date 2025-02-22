//
// Video Controller for PCW_MiSTer
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

module video_controller(
    input wire reset,               // Reset
	input wire clk_sys,             // 64 Mhz System Clock, Need to divide by 4 for 16Mhz pixel clock
    input wire [7:0] roller_ptr,    // Port F5 io register - Roller Ram Ptr
    input wire [7:0] yscroll,       // Port F6 y scroll register
    input wire inverse,             // Port F7 inverse video
    input wire disable_vid,         // Port F7 / F8 video disable
    input wire ntsc,                // Port F8 NTSC video flag
    input wire [1:0] fake_colour_mode,         // Fake colour mode enabled
	input wire [1:0] pcw_video_mode,     // Fake colour mode EGA enabled
    input wire [7:0] fake_end,      // Fake colour end row
    output logic [7:0] ypos,        // Current yposition for fake colour comparison logic
	output logic [16:0] vid_addr,   // Address Bus out for reading pixel data & roller ram
	input wire [7:0] din,           // Data in for pixel data and roller ram
    input  [3:0] VShift,
    input  [3:0] HShift,
	output logic [3:0] colour,
	output logic ce_pix,
	output logic hsync,
	output logic vsync,
	output logic hb,
	output logic vb,
    output logic timer_int

    );

    // generate a 16mhz pixel clock based on clk_sys being 64mhz
    reg [15:0] cnt;
    reg pix_stb;
    always @(posedge clk_sys)
        {pix_stb, cnt} <= cnt + 16'h4000;  // divide by 4: (2^16)/2 = 0x4000
    assign ce_pix = pix_stb;

    logic [10:0] x;  // current pixel x position: 10-bit value: 0-1023
    logic [8:0] y;  // current pixel y position:  9-bit value: 0-511
    logic active;   // screen area action
    logic screen_start;   // Positive for one pixel at the very end of frame
    logic line_start;   // Start of horizontal sync line

    video_sync display (
        .i_clk(clk_sys),
        .i_pix_stb(pix_stb),
        .i_rst(reset),
        .i_ntsc(ntsc),
        .o_hs(hsync), 
        .o_vs(vsync), 
        .o_hblank(hb), 
        .o_vblank(vb), 
        .o_x(x), 
        .o_y(y),
        .VShift(VShift),
        .HShift(HShift),
        .o_active(active),
        .o_screenstart(screen_start),
        .o_linestart(line_start),
        .o_timer(timer_int)
    );

    assign ypos = y;
    // lookup_addr and line_addr driver
    logic [16:0] line_addr = 17'd0;
    logic [15:0] roller_bits; 
    typedef enum bit [1:0] {IDLE, GET_MSB, GET_LSB, SETUP} roller_states;
    roller_states roller_state;
    logic video_lookup;         // memory override bit to get roller ram address
    logic [16:0] lookup_addr;   // address in memory to get roller ram lsb and msb
    always @ (posedge clk_sys)
    begin

        logic old_ls = 1'b0;

        if (reset)
        begin
            roller_state <= IDLE;
            video_lookup <= 1'b1;
            roller_bits <= 'b0;
            line_addr <= 'b0;
            lookup_addr <= 'b0;
        end else
        begin
            if(ce_pix & ~vb)
            begin
                old_ls <= line_start;
                if(~old_ls & line_start) begin
                    roller_state <= GET_LSB;
                    video_lookup <= 1'b1;
                    // Read MSB of address
                    lookup_addr <= {roller_ptr[7:0],9'b0} + (((y + yscroll) & 8'hff) << 1);
                end else begin
                    case(roller_state)
                        IDLE: begin
                            video_lookup <= 1'b0;
                            // Normal logic for reading a pixel
                            lookup_addr <= 'b0;
                        end
                        GET_LSB: begin
                            video_lookup <= 1'b1;
                            // Read LSB of address
                            lookup_addr <= {roller_ptr[7:0],9'b0} + (((y + yscroll) & 8'hff) << 1) + 1;
                            // din should equal LSB from previous step
                            roller_bits[7:0] <= din;
                            roller_state <= GET_MSB;
                        end
                        GET_MSB: begin
                            video_lookup <= 1'b0;
                            lookup_addr <= 'b0;
                            // din should equal MSB from previous transition step
                            roller_bits[15:8] <= din;
                            roller_state <= SETUP;
                        end
                        SETUP: begin
                            video_lookup <= 1'b0;
                            lookup_addr <= 'b0;
                            // Set line address for future pixel reads
                            line_addr <= {roller_bits[15:3],1'b0,roller_bits[2:0]};
                            roller_state <= IDLE;
                        end
                    endcase
                end
            end
        end
    end

    // Pixel memory lookup address controller
    logic [16:0] pixel_addr /* synthesis keep */;
    //assign pixel_addr = active ? (pcw_video_mode == 3 ? line_addr + (x[10:4] << 3 ) : line_addr + (x[10:3] << 3 )) : 'b0;
    //assign pixel_addr = active ? (pcw_video_mode == 3 ? line_addr + (x[10:3] << 4) : line_addr + (x[10:3] << 3)) : 'b0;
    assign pixel_addr = active ? line_addr + (x[10:3] << 3 ) : 'b0;

    // Address controller for vid_addr
    assign vid_addr = video_lookup ? lookup_addr : pixel_addr;

    logic [15:0] pixel_reg = 'b0;
    logic [3:0] pixel;
    logic [7:0] attr_reg = 'b0;  //Atrubute register for PCWPlus 3 mode
    logic [7:0] attr_shift_reg = 'b0;  // Displaced attribute register for PCWPlus 3 mode
    always @ (posedge clk_sys)
    begin
        if(ce_pix)
        begin
            if (pcw_video_mode == 3) begin  // PCWPlus 3 mode
                if (x[3:0] == 4'b0000 && active) begin
                    attr_reg <= din;
                end else if (x[3:0] == 4'b1000 && active) begin
                    pixel_reg <= {{2{din[7]}}, {2{din[6]}}, {2{din[5]}}, {2{din[4]}}, {2{din[3]}}, {2{din[2]}}, {2{din[1]}}, {2{din[0]}}};  // Duplicamos los datos de píxeles;
                     attr_shift_reg <= attr_reg; 
                end else begin
                    pixel_reg <= {pixel_reg[14:0], 1'b0}; 	 		
                end
            end else begin 
            // Every 8 pixels load shift reg
            if(x[2:0]==3'b000 && active) pixel_reg <={din, din};
             // else shift pixel register left
            else begin
                   // Shift every other pixel in fake colour mode
               if (fake_colour_mode > 2'b00 && y <= fake_end) begin
                    if(fake_colour_mode == 2'b10) begin
                        if (pcw_video_mode ==2) pixel_reg <=  (x[1:0] ==2'b00) ? {pixel_reg[11:0],4'b0} : pixel_reg;
                        else if (pcw_video_mode ==1) pixel_reg <=  ~x[0] ?  {pixel_reg[5:0], 2'b0,pixel_reg[13:8], 2'b0} : pixel_reg;                     
                        else  pixel_reg <= {pixel_reg[14:0], 1'b0};
                    end else pixel_reg <=  ~x[0] ? {pixel_reg[5:0], 2'b0,pixel_reg[13:8], 2'b0} : pixel_reg;                     
				end else pixel_reg <= {pixel_reg[14:0], 1'b0}; 					
            end
		end	
            // Load pixel register
            pixel <= (fake_colour_mode >2'b00 && y <= fake_end ) ? pixel_reg[15:12] : {pixel_reg[15], pixel_reg[15],pixel_reg[15], pixel_reg[15]};
        end
    end
    // Screen on and pixel to draw
    always_comb
    begin
        if (pcw_video_mode == 3) begin  //PCWplus mode 3
            if (!disable_vid && active) begin
                if (pixel_reg[15]) colour = attr_shift_reg[3:0];
                else colour =  attr_shift_reg[7:4];
            end else colour = 4'b0000; 
        end else begin 
            if(inverse) begin
                if(!disable_vid && active) colour = ~pixel;
                else colour = 4'b1111;  // Disabled inverse video 
            end else begin
                if(!disable_vid && active) colour = pixel;
                else colour = 4'b0000;
            end
        end
	end
    
endmodule