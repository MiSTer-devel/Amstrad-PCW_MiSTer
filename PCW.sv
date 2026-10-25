//============================================================================
//  Amstrad PCW port to MiSTer
//  Copyright (c) 2020 Stephen Eddy
//  PCWPLUS Color modes modes Copyright (c) 2020 habisoft
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);
assign VGA_F1=0;
assign VGA_SCALER = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

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
//assign VIDEO_ARX = 4; //status[13] ? 4 : (status[12] ? 20 : 11);
//assign VIDEO_ARY = 3; //status[13] ? 3 : (status[12] ? 17 : 10);

assign AUDIO_S = 0;
assign AUDIO_MIX = 0;

assign LED_DISK  = LED;				/* later add disk motor on/off */
assign LED_POWER = 0;
assign LED_USER  = ioctl_download;

wire [1:0] ar = status[21:20];
video_freak video_freak
(
	.*,
	.VGA_DE_IN(VGA_DE),
	.VGA_DE(),
	.ARX((!ar) ? 12'd4 : (ar - 1'd1)),
	.ARY((!ar) ? 12'd3 : 12'd0),
	.CROP_SIZE(0),
	.CROP_OFF(0),
	.SCALE(status[23:22])
);

//	"O89,Clockspeed (MHz),4.00(1x),8.00(2x),16.00(4x),32.00(x8);",
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
	"P1,Video;",
	"P1-;",	
	"P1O56,Screen Color,White,Green,Amber;",
	"P1O7,Video System,PAL,NTSC;",
	"P1F3,gbp,Load Palette;",
	"P1OQR,Fake Colour,None,Loaded palette, PCWPLUS, EGA;",
	"P1oAD,CRT H-Sync Adjust,0,1,2,3,4,5,6,7,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1oEH,CRT V-Sync Adjust,0,1,2,3,4,5,6,7,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1OKL,Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P1O13,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%, CRT 75%;",
	"P1OMN,Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"-;",
	"OAC,Joystick Type,None,Kempston,Spectravideo,Cascade,DKTronics;",
	"ODE,Mouse Type,None,AMX,Kempston,Keymouse;",
	"OH,DKTronics I/F,Disabled,Enabled;",
	"-;",
	"R0,Reset;",
	"J,Fire 1,Fire 2;",
	"V,v",`BUILD_DATE
};

wire locked;
(* preserve *) wire clk_sys;
pll pll
(
	.refclk   (CLK_50M),
	.rst      (0),
	.outclk_0 (clk_sys), // 32 MHz
	.locked	  (locked)
);

wire [63:0] status;
wire  [1:0] buttons;
wire        ioctl_download;
wire        ioctl_wr;
wire [15:0] ioctl_addr;
wire  [7:0] ioctl_data;
wire  [7:0] ioctl_index;
wire		ioctl_wait = 1'b0;
wire [31:0] sd_lba[2];
wire [31:0] sd_lba_0;
wire  [1:0] sd_rd;
wire  [1:0] sd_wr;
wire  [1:0] sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din_0;
wire  [7:0] sd_buff_din[2];
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
wire        freeze_sync;

hps_io #(.CONF_STR(CONF_STR), .WIDE(0), .VDNUM(2)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),

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

assign sd_buff_din[0]=sd_buff_din_0;
assign sd_buff_din[1]=sd_buff_din_0;
assign sd_lba[0]=sd_lba_0;
assign sd_lba[1]=sd_lba_0;


wire palette_download = ioctl_download && (ioctl_index == 3);
wire reset = RESET | ~locked | status[0] | buttons[1];

// Palette download
reg [127:0] palette = 128'h00000032cd320000ff00ffff00000000;

always @(posedge clk_sys) begin
	if (palette_download & ioctl_wr) begin
			palette[127:0] <= {palette[119:0], ioctl_data[7:0]};
	end
end
pcw_core pcw_core
(
	.reset(reset),
	.clk_sys(clk_sys),
	.palette(palette),
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
	.HShift(status[45:42]),
	.VShift(status[49:46]),
	.LED(LED),
	.audiomix_l(audiomix_l),
	.audiomix_r(audiomix_r),

	.disp_color(status[6:5]),
	.ntsc(status[7]),
	.overclock(status[9:8]),
	.model(status[4]),
	.memory_size(status[16:15]),
	.dktronics(status[17]),
	.fake_colour_mode(status[27:26]),
	//.dn_clk(clk_sys),
	//.dn_go(loader_download),
	//.dn_wr(loader_wr),
	//.dn_addr(loader_addr),			// CPU = 0000-FFFF; cassette = 10000-1FFFF
	//.dn_data(loader_data),

	//.execute_addr(execute_addr),
	//.execute_enable(execute_enable),

	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),
	.density({1'b1, status[4]}),		// 8256/512 = A=SD, 9512+ A=DD

	.sd_lba(sd_lba_0),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din_0),
	.sd_dout_strobe(sd_buff_wr),
	// SD RAM signals not explicitly named
	.locked(locked),
	.*	
);

///////////////////////////////////////////////////
wire        ce_pix;
wire [23:0] RGB;
wire        HSync,VSync,HBlank,VBlank;

wire  [2:0] scale = status[3:1];
wire  [2:0] sl = scale > 1'd1 ? scale - 1'd1 : 3'd0;

assign CLK_VIDEO = clk_sys;
assign VGA_SL = sl[1:0];

video_mixer #( .GAMMA(1)) video_mixer
(
	.*,


	.freeze_sync(),
	.scandoubler(scale || forced_scandoubler),
	.hq2x(scale==3'b001),


	.B({RGB[7:0]}),
	.G({RGB[15:8]}),
	.R({RGB[23:16]})
);

wire  [13:0] audiomix_l, audiomix_r;
// Unsigned samples, as in the original core: the whole 14-bit range fits shifted by two,
// so there is no overflow and no 6 dB loss. The framework removes the DC offset.
assign AUDIO_L={audiomix_l,2'b00};
assign AUDIO_R={audiomix_r,2'b00};

endmodule
