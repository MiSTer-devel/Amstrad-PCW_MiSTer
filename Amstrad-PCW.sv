//============================================================================
//  HT1080Z port to MiSTer
//  Renamed to TRS-80 after Cassette and CMD loading support
//  
//  Copyright (c) 2019 Alan Steremberg - alanswx
//
//
//============================================================================

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [45:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	output  [7:0] VIDEO_ARX,
	output  [7:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input		  CLK_AUDIO,
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S, // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)
	input         TAPE_IN,
	//ADC
	inout   [3:0] ADC_BUS,

	// SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
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

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,


	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

assign VGA_F1=0;

assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign USER_OUT = '1;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0;
assign ADC_BUS  = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;

assign BUTTONS = 0;

// aspect ratio including all border space is  4:3
// aspect ratio iwith partial border space is 20:17
// aspect ratio of only displayed area is     11:10
assign VIDEO_ARX = 4; //status[13] ? 4 : (status[12] ? 20 : 11);
assign VIDEO_ARY = 3; //status[13] ? 3 : (status[12] ? 17 : 10);

assign AUDIO_S = 0;
assign AUDIO_MIX = 0;

assign LED_DISK  = LED;				/* later add disk motor on/off */
assign LED_POWER = 0;
assign LED_USER  = ioctl_download;

localparam BOOT_ROM_END = 16'd275;	// Length of boot rom

`include "build_id.v"
localparam CONF_STR = {
	"Amstrad PCW;;",
	"S0,DSK,Mount A:;",
	"S1,DSK,Mount B:;",
	"-;",
	"O4,System Model,8256/8512,9256/9512+;",
	"OFG,Memory Size,256K,512K,1MB,2MB;",
	"O89,Clockspeed (MHz),4.00(1x),8.00(2x),16.00(4x),32.00(x8);",
	"-;",	
	"O56,Screen Color,White,Green,Amber;",
	"O7,Video System,PAL,NTSC;",
	"OIJ,Fake Colour,None,Palette 1, Palette 2, Palette 3;",
	"O13,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"-;",
//	"O4,Kbd Layout,PCW,PC;",
	"OAC,Joystick Type,None,Kempston,Spectravideo,Cascade,DKTronics;",
	"ODE,Mouse Type,None,AMX,Kempston,Keymouse;",
	"OH,DKTronics I/F,Disabled,Enabled;",
	"-;",
	"R0,Reset;",
	"J,Fire 1,Fire 2;",
	"V,v",`BUILD_DATE
};

logic locked;
(* preserve *) wire clk_sys;
pll pll
(
	.refclk   (CLK_50M),
	.rst      (0),
	.outclk_0 (clk_sys), // 32 MHz
	.locked	  (locked)
);

wire [31:0] status;
wire  [1:0] buttons;
wire        ioctl_download;
wire        ioctl_wr;
wire [15:0] ioctl_addr;
wire  [7:0] ioctl_data;
wire  [7:0] ioctl_index;
wire		ioctl_wait;
wire [31:0] sd_lba;
wire  [1:0] sd_rd;
wire  [1:0] sd_wr;
wire        sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din;
wire        sd_buff_wr;
wire  [1:0] img_mounted;
wire        img_readonly;
wire [63:0] img_size;

wire        forced_scandoubler;
wire [10:0] ps2_key;
wire [24:0] ps2_mouse;

wire [21:0] gamma_bus;

wire [15:0] joystick_0, joystick_1;
wire LED;

hps_io #(.STRLEN(($size(CONF_STR)>>3) ), .WIDE(0), .VDNUM(2)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),

	.conf_str(CONF_STR),

	.ps2_key(ps2_key),
	.ps2_mouse(ps2_mouse),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.buttons(buttons),
	.forced_scandoubler(forced_scandoubler),
	.gamma_bus(gamma_bus),

	.status(status),

	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_data),
	.ioctl_wait(ioctl_wait),
	.ioctl_index(ioctl_index),

	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),

	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size)
);

wire rom_download = ioctl_download && ioctl_index==0;
wire reset = RESET | status[0] | buttons[1] | rom_download;

// signals from loader
logic loader_wr;		
logic loader_download;
reg [15:0] loader_addr;
reg [7:0] loader_data;
reg [15:0] execute_addr;
logic execute_enable;
logic loader_wait;

// Boot loader to kickstart system on a reset
// Required because the ROM is overwritten and needs to be reloaded every reset
// First detect end of reset pulse to kickstart download
logic reset_ne;
logic first_byte;
edge_det reset_edge_det(.clk_sys(clk_sys), .signal(reset), .neg_edge(reset_ne));

logic [15:0] read_addr;
logic [7:0] read_data;
always @(posedge clk_sys)
begin
	if(reset_ne)
	begin
		read_addr <= 'b0;
		loader_addr <= 'b0;
		loader_wr <= 1'b0;
		execute_enable <= 1'b0;
		loader_download <= 1'b1;
		execute_addr <= 'b0;
	end
	else begin
		if(loader_download) 
		begin
			if(~loader_wr) 
			begin
				// Transfer loaded byte to loader
				loader_data <= read_data;
				loader_wr <= 1'b1;
			end
			else begin
				loader_wr <= 1'b0;
				loader_addr <= loader_addr + 16'd1;
				read_addr <= read_addr + 16'd1;
				if(read_addr >= BOOT_ROM_END)
				begin
					loader_download <= 1'b0;
					execute_enable <= 1'b1;
				end
			end
		end		
		if(execute_enable) execute_enable <= 1'b0;
	end
end

// Rom containing boot rom code to transfer to address 0
boot_loader boot_loader
(
	.address(read_addr),
	.model(status[4]),
	.data(read_data)
);

pcw_core pcw_core
(
	.reset(reset),
	.clk_sys(clk_sys),

	.joy0(joystick_0),
	.joy1(joystick_1),
	.joy_type(status[12:10]),
	.ps2_key(ps2_key),
	.ps2_mouse(ps2_mouse),
	.mouse_type(status[14:13]),

	.RGB(RGB),
	.hsync(HSync),
	.vsync(VSync),
	.hblank(HBlank),
	.vblank(VBlank),
	.ce_pix(ce_pix),

	.LED(LED),
	.audiomix(audiomix),

	.disp_color(status[6:5]),
	.ntsc(status[7]),
	.overclock(status[9:8]),
	.model(status[4]),
	.memory_size(status[16:15]),
	.dktronics(status[17]),
	.fake_colour_mode(status[19:18]),

	.dn_clk(clk_sys),
	.dn_go(loader_download),
	.dn_wr(loader_wr),
	.dn_addr(loader_addr),			// CPU = 0000-FFFF; cassette = 10000-1FFFF
	.dn_data(loader_data),

	.execute_addr(execute_addr),
	.execute_enable(execute_enable),

	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),
	.density({1'b1, status[4]}),		// 8256/512 = A=SD, 9512+ A=DD

	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_dout_strobe(sd_buff_wr),
	// SD RAM signals not explicitly named
	.locked(locked),
	.*	
);

///////////////////////////////////////////////////
wire        ce_pix;
wire [17:0] RGB;
wire        HSync,VSync,HBlank,VBlank;

wire  [2:0] scale = status[3:1];
wire  [2:0] sl = scale > 1'd1 ? scale - 1'd1 : 3'd0;

assign CLK_VIDEO = clk_sys;
assign VGA_SL = sl[1:0];

video_mixer #(.LINE_LENGTH(1024), .GAMMA(1)) video_mixer
(
	.*,

	.clk_vid(clk_sys),
	.ce_pix_out(CE_PIXEL),

	.scanlines(0),
	.scandoubler(scale || forced_scandoubler),
	.hq2x(scale==3'b001),

	.mono(0),

	.B({RGB[5:0],RGB[5:4]}),
	.G({RGB[11:6],RGB[11:10]}),
	.R({RGB[17:12],RGB[17:16]})
);

wire  [8:0] audiomix;

assign AUDIO_L={audiomix,7'b0000000};
assign AUDIO_R=AUDIO_L;

endmodule
