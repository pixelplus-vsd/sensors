// SPDX-License-Identifier: GPL-2.0
/*
 * This is compatible with the PI6008(ISP) + PKA210(CIS) designed by PIXELPLUS.
 * The ISP can transmit either Bayer data (10-bit or 12-bit) or YUV422 streaming data.

 * Unless otherwise specified, the I2C slave address is 0x1A (0b001_1010).
 * It uses 0x34(0b0011_0100) for writing, and 0x35(0b0011_0101) for reading.
 
 * There are two key points to be mindful of:
 *  1. For the GPIO reset pin, if it is set to low,
 *     neither video data nor I2C communication will work,
 *     even if the image sensor is powered.
  
 * Based on dummy camera driver by Raspberry Pi (Trading) Ltd.
 * Copyright (C) 2024, PIXELPLUS Co., Ltd.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>

// for test function
#define V4L2_CID_PG7130_CUSTOM_SETTING	(V4L2_CID_LASTP1 + 0x01)
void TestRegs(struct device *dev, struct v4l2_subdev *sd);

u32 mbus_codes[] = {
	// MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_Y8_1X8,
};
#define NUM_MBUS_CODES ARRAY_SIZE(mbus_codes)

#define MIN_WIDTH	16
#define MAX_WIDTH	16383
#define MIN_HEIGHT	16
#define MAX_HEIGHT	16383

#define DEFAULT_WIDTH	640U
#define DEFAULT_HEIGHT	480U
/* Default format will be the first entry in mbus_codes */

#define PG7130_NATIVE_WIDTH		640U
#define PG7130_NATIVE_HEIGHT		480U

#define PG7130_PIXEL_ARRAY_LEFT		0U
#define PG7130_PIXEL_ARRAY_TOP		0U
#define PG7130_PIXEL_ARRAY_WIDTH		640U
#define PG7130_PIXEL_ARRAY_HEIGHT	480U

#define PG7130_HORIZONTAL_TOTAL_SIZE	1600U		// pixel domain 2475U
#define PG7130_VERTICAL_TOTAL_SIZE	500U		// pixel domain 1000U

/* Pixel rate is fixed for all the modes */
//#define PG7130_PIXEL_RATE			74250000	//74.25MHz
//#define PG7130_DEFAULT_LINK_FREQ	297000000	//297MHz
#define PG7130_PIXEL_RATE			74250000/2
#define PG7130_DEFAULT_LINK_FREQ	192000000

#define PG7130_VBLANK_MIN		0
#define PG7130_VTS_MAX			32767
#define PG7130_HTS_MAX			0x1fff

static const s64 PG7130_link_freq_menu[] = {
	PG7130_DEFAULT_LINK_FREQ,
};

#define PG7130_REG_MIRROR		0xF070016CUL
enum PG7130_mirror {
	PG7130_MIRROR_H = 0,
	PG7130_MIRROR_V
};

/* regulator supplies */
static const char * const PG7130_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define PG7130_NUM_SUPPLIES ARRAY_SIZE(PG7130_supply_names)

struct regval_list {
	u32 addr;
	u32 data;
};

struct PG7130_mode {
	struct v4l2_mbus_framefmt	format;
	struct v4l2_rect		crop;
	u64				pixel_rate;
	int				hts;
	int				vts;
	const struct regval_list	*reg_list;
	unsigned int			num_regs;
};

struct PG7130 {
	struct v4l2_subdev		sd;
	struct media_pad		pad;
	struct mutex			lock;
	struct clk			*xclk;
	struct gpio_desc		*pwdn;
	struct regulator_bulk_data supplies[PG7130_NUM_SUPPLIES];
	bool				clock_ncont;
	struct v4l2_ctrl_handler	ctrls;
	struct PG7130_mode		*mode;
	struct v4l2_ctrl		*pixel_rate;
	struct v4l2_ctrl		*link_freq;
	struct v4l2_ctrl		*hblank;
	struct v4l2_ctrl		*vblank;
	struct v4l2_ctrl		*exposure;
	struct v4l2_ctrl		*hflip;
	struct v4l2_ctrl		*vflip;
	struct v4l2_ctrl		*rw_control;
	struct v4l2_ctrl		*reg_control;
	struct v4l2_ctrl		*custom_ctrl;
	bool				streaming;
	u8				lanes;
};

static inline struct PG7130 *to_sensor(struct v4l2_subdev *sd)
{
	return container_of(sd, struct PG7130, sd);
}



static struct regval_list PG7130_640x480p_Y8[] = {
	{ 0xf0802040, 0x7130ac5f },
	{ 0xf08020dc, 0xaca17130 },
	{ 0xf0802090, 0x03178ecf },

	{ 0xf0700174, 0x00000001 },
	// { 0xF0801010, 0x00000040 },  // [5]0:MIPI DPHY global power down disable, 1:power down, [4]0:LP&HS mode(0x40), 1: HS mode only(0x50)
	
	// { 0xf0801070, 0x00000005 },  // Tdata and clk lpx 100ns, 1UI 20.8333333ns
	
	// { 0xf0801078, 0x00000008 },  // Tdata hs prepare 166.67ns
	// { 0xf080107c, 0x0000000f },  // Tdata hs zero 312.5ns
	// { 0xf0801080, 0x0000000f },  // Tdata hs hs trail 312.5ns
	// { 0xf0801098, 0x00000006 },  // Tdata hs exit 166.67ns(automatic)
	
	// { 0xf0801074, 0x00000003 },  // Tclk_prepare 62.5ns
	// { 0xf0801084, 0x00000023 },  // Tclk_zero 729.167
	// { 0xf0801088, 0x0000000f },  // Tclk_trail 312.5
	// { 0xf080108c, 0x0000000f },  // Tclk_pre 312.5
	// { 0xf0801090, 0x00000042 },  // Tclk_post 1375
	// { 0xf080109c, 0x000000a0 },  // Tclk_hsexit 3333.333
};
static const struct PG7130_mode PG7130_modes[] = {

	{
		.format = { 
			.code = MEDIA_BUS_FMT_Y8_1X8,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= PG7130_PIXEL_ARRAY_WIDTH,
			.height		= PG7130_PIXEL_ARRAY_HEIGHT
		},
		.crop = {
			.left		= 0,
			.top		= 0,
			.width		= PG7130_PIXEL_ARRAY_WIDTH,
			.height		= PG7130_PIXEL_ARRAY_HEIGHT
		},
		.pixel_rate	= PG7130_PIXEL_RATE,
		.hts		= PG7130_HORIZONTAL_TOTAL_SIZE,
		.vts		= PG7130_VERTICAL_TOTAL_SIZE,
		.reg_list	= PG7130_640x480p_Y8,
		.num_regs	= ARRAY_SIZE(PG7130_640x480p_Y8)
	},
};

#define PG7130_DEFAULT_MODE (&PG7130_modes[0])
#define PG7130_DEFAULT_FORMAT (PG7130_modes[0].format)
