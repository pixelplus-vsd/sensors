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
#define V4L2_CID_PX6130_CUSTOM_SETTING	(V4L2_CID_LASTP1 + 0x01)
void TestRegs(struct device *dev, struct v4l2_subdev *sd);

u32 mbus_codes[] = {
	MEDIA_BUS_FMT_Y10_1X10,
	MEDIA_BUS_FMT_Y12_1X12,
	MEDIA_BUS_FMT_YUYV8_1X16,
	MEDIA_BUS_FMT_UYVY8_1X16,
	MEDIA_BUS_FMT_YVYU8_1X16,
	MEDIA_BUS_FMT_VYUY8_1X16
};
#define NUM_MBUS_CODES ARRAY_SIZE(mbus_codes)

#define MIN_WIDTH	16
#define MAX_WIDTH	16383
#define MIN_HEIGHT	16
#define MAX_HEIGHT	16383

#define DEFAULT_WIDTH	1280U
#define DEFAULT_HEIGHT	960U
/* Default format will be the first entry in mbus_codes */

#define PX6130_NATIVE_WIDTH		1280U
#define PX6130_NATIVE_HEIGHT		960U

#define PX6130_PIXEL_ARRAY_LEFT		0U
#define PX6130_PIXEL_ARRAY_TOP		0U
#define PX6130_PIXEL_ARRAY_WIDTH		1280U
#define PX6130_PIXEL_ARRAY_HEIGHT	960U

#define PX6130_HORIZONTAL_TOTAL_SIZE	1600U		// pixel domain 2475U
#define PX6130_VERTICAL_TOTAL_SIZE	1000U		// pixel domain 1000U

/* Pixel rate is fixed for all the modes */
//#define PX6130_PIXEL_RATE			74250000	//74.25MHz
//#define PX6130_DEFAULT_LINK_FREQ	297000000	//297MHz
#define PX6130_PIXEL_RATE			74250000
#define PX6130_DEFAULT_LINK_FREQ	192000000

#define PX6130_VBLANK_MIN		0
#define PX6130_VTS_MAX			32767
#define PX6130_HTS_MAX			0x1fff

static const s64 PX6130_link_freq_menu[] = {
	PX6130_DEFAULT_LINK_FREQ,
};

#define PX6130_REG_MIRROR		0xF070016CUL
enum PX6130_mirror {
	PX6130_MIRROR_H = 0,
	PX6130_MIRROR_V
};

/* regulator supplies */
static const char * const PX6130_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define PX6130_NUM_SUPPLIES ARRAY_SIZE(PX6130_supply_names)

struct regval_list {
	u32 addr;
	u32 data;
};

struct PX6130_mode {
	struct v4l2_mbus_framefmt	format;
	struct v4l2_rect		crop;
	u64				pixel_rate;
	int				hts;
	int				vts;
	const struct regval_list	*reg_list;
	unsigned int			num_regs;
};

struct PX6130 {
	struct v4l2_subdev		sd;
	struct media_pad		pad;
	struct mutex			lock;
	struct clk			*xclk;
	struct gpio_desc		*pwdn;
	struct regulator_bulk_data supplies[PX6130_NUM_SUPPLIES];
	bool				clock_ncont;
	struct v4l2_ctrl_handler	ctrls;
	struct PX6130_mode		*mode;
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

static inline struct PX6130 *to_sensor(struct v4l2_subdev *sd)
{
	return container_of(sd, struct PX6130, sd);
}



static struct regval_list PX6130_960p_YUV422[] = {
	{ 0xF0000004, 0x68340C28 },  // [CLOCK]
	{ 0xF0000008, 0x2C050000 },  //
	{ 0xF000000C, 0x40004000 },  //
	{ 0xF0000000, 0x00000000 },  //
	{ 0xF0000014, 0x54942C28 },  //
	{ 0xF0000018, 0x2C050000 },  //
	{ 0xF000001C, 0x80008000 },  //
	{ 0xF0000010, 0x00000000 },  //
	{ 0xF0000020, 0x01020002 },  //
	{ 0xF0000024, 0x00000001 },  //
	{ 0xF0000028, 0x00030300 },  //
	{ 0xF000002C, 0x08080300 },  //[CLOCK End]
	/*
	[PAD IO]
	*/
	{ 0xF0001040, 0x20202020 },  //[PAD IO]
	{ 0xF0001044, 0x20202020 },  //
	{ 0xF0001048, 0x00002020 },  //
	{ 0xF000104C, 0x02000021 },  //
	{ 0xF0001050, 0x03030101 },  //
	{ 0xF0001054, 0x00000303 },  //
	{ 0xF0001058, 0x00020000 },  //
	{ 0xF0001060, 0x00000015 },  //
	{ 0xF0001064, 0x00000043 },  //[PAD IO End]
	/*
	[SAFETY OPTION]
	*/
	{ 0x01007A00, 0x01010101 },  //
	{ 0x01007A04, 0x05010101 },  //
	{ 0x01007A08, 0x04010105 },  //
	{ 0x01007A0C, 0x01010101 },  //
	{ 0x01007A10, 0x01010101 },  //
	{ 0x01007A14, 0x01010101 },  //
	{ 0x01007A18, 0x01010101 },  //
	{ 0x01007A1C, 0x01010101 },  //
	{ 0x01007A20, 0x00000100 },  //[SAFETY OPTION End]
	/*
	[VO]
	*/
	{ 0xF0800040, 0x09030E81 },  //
	{ 0xF0800044, 0x01400500 },  //
	{ 0xF0800048, 0x000403C0 },  //
	{ 0xF080004C, 0x00000027 },  //
	{ 0xF0800080, 0x11000000 },  //
	{ 0xF0800084, 0x000000A0 },  //
	{ 0xF0800090, 0xC0000000 },  //
	{ 0xF0800094, 0x00000500 },  //
	{ 0xF0800098, 0x00000A00 },  //
	{ 0xF08000C0, 0x000603C0 },  //
	{ 0xF08000C4, 0x00000FFF },  //
	{ 0xF080010C, 0x00000000 },  //
	{ 0xF0800110, 0x00000000 },  //
	{ 0xF0800114, 0x00000000 },  //
	{ 0xF0800118, 0x00000000 },  //
	{ 0xF080011C, 0x00000000 },  //
	{ 0xF0800120, 0x00000000 },  //
	{ 0xF0800124, 0x00000000 },  //
	{ 0xF0800128, 0x00000000 },  //
	{ 0xF080012C, 0x00000000 },  //
	{ 0xF0800130, 0x00000000 },  //
	{ 0xF0800134, 0x00000000 },  //
	{ 0xF0800138, 0x00000000 },  //
	{ 0xF080013C, 0x00000000 },  //
	{ 0xF0800140, 0x00000000 },  //
	{ 0xF0800144, 0x00000000 },  //
	{ 0xF0800148, 0x00000000 },  //
	{ 0xF080014C, 0x00000000 },  //
	{ 0xF0800150, 0x14002880 },  //
	{ 0xF0800154, 0x00002052 },  //
	{ 0xF0801014, 0x00000004 },  // MIPI CKP/CKN pad state control [3:2]1:normal(0x04), 2:Phase inverted(0x08)
	{ 0xF0801018, 0x00000000 },  //
	{ 0xF080101C, 0x000000AA },  //
	{ 0xF0801020, 0x00000005 },  //
	{ 0xF0801070, 0x00000002 },  //
	{ 0xF0801074, 0x00000002 },  //
	{ 0xF0801078, 0x00000003 },  //
	{ 0xF080107C, 0x00000005 },  //
	{ 0xF0801080, 0x00000005 },  //
	{ 0xF0801084, 0x0000000C },  //
	{ 0xF0801088, 0x00000003 },  //
	{ 0xF080108C, 0x00000001 },  //
	{ 0xF0801090, 0x00000009 },  //
	{ 0xF0801094, 0x0000002A },  //
	{ 0xF0801098, 0x00000006 },  //
	{ 0xF080109C, 0x00000006 },  //
	{ 0xF0801118, 0x0000001E },  //
	{ 0xF08010D8, 0x0000000A },  //
	{ 0xF08010DC, 0x00000000 },  //
	{ 0xF0801010, 0x00000050 },  //
	{ 0xF0800200, 0x27100000 },  //
	{ 0xF0800204, 0x41F848EA },  //
	{ 0xF0800208, 0x41F848EA },  //
	{ 0xF0700070, 0x00000002 },  //[VO End]
};
static const struct PX6130_mode PX6130_modes[] = {

	{
		.format = { 
			.code = MEDIA_BUS_FMT_UYVY8_1X16,
			//.code = MEDIA_BUS_FMT_YUYV8_1X16,
			.colorspace	= V4L2_COLORSPACE_SMPTE170M,
			//.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= PX6130_PIXEL_ARRAY_WIDTH,
			.height		= PX6130_PIXEL_ARRAY_HEIGHT
		},
		.crop = {
			.left		= 0,
			.top		= 0,
			.width		= PX6130_PIXEL_ARRAY_WIDTH,
			.height		= PX6130_PIXEL_ARRAY_HEIGHT
		},
		.pixel_rate	= PX6130_PIXEL_RATE,
		.hts		= PX6130_HORIZONTAL_TOTAL_SIZE,
		.vts		= PX6130_VERTICAL_TOTAL_SIZE,
		.reg_list	= PX6130_960p_YUV422,
		.num_regs	= ARRAY_SIZE(PX6130_960p_YUV422)
	},
};

#define PX6130_DEFAULT_MODE (&PX6130_modes[0])
#define PX6130_DEFAULT_FORMAT (PX6130_modes[0].format)
