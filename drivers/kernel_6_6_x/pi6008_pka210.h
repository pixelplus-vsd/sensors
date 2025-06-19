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
#define V4L2_CID_PI6008_PKA210_CUSTOM_SETTING	(V4L2_CID_LASTP1 + 0x01)
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

#define DEFAULT_WIDTH	1920U
#define DEFAULT_HEIGHT	1080U
/* Default format will be the first entry in mbus_codes */

#define PI6008_PKA210_NATIVE_WIDTH		1920U
#define PI6008_PKA210_NATIVE_HEIGHT		1080U

#define PI6008_PKA210_PIXEL_ARRAY_LEFT		0U
#define PI6008_PKA210_PIXEL_ARRAY_TOP		0U
#define PI6008_PKA210_PIXEL_ARRAY_WIDTH		1920U
#define PI6008_PKA210_PIXEL_ARRAY_HEIGHT	1080U

#define PI6008_PKA210_HORIZONTAL_TOTAL_SIZE	2200U		// pixel domain 2475U
#define PI6008_PKA210_VERTICAL_TOTAL_SIZE	1125U		// pixel domain 1000U

/* Pixel rate is fixed for all the modes */
//#define PI6008_PKA210_PIXEL_RATE			74250000	//74.25MHz
//#define PI6008_PKA210_DEFAULT_LINK_FREQ	297000000	//297MHz
#define PI6008_PKA210_PIXEL_RATE			74250000
#define PI6008_PKA210_DEFAULT_LINK_FREQ	148500000

#define PI6008_PKA210_VBLANK_MIN		0
#define PI6008_PKA210_VTS_MAX			32767
#define PI6008_PKA210_HTS_MAX			0x1fff

static const s64 PI6008_PKA210_link_freq_menu[] = {
	PI6008_PKA210_DEFAULT_LINK_FREQ,
};

#define PI6008_PKA210_REG_MIRROR		0xF070016CUL
enum PI6008_PKA210_mirror {
	PI6008_PKA210_MIRROR_H = 0,
	PI6008_PKA210_MIRROR_V
};

/* regulator supplies */
static const char * const PI6008_PKA210_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define PI6008_PKA210_NUM_SUPPLIES ARRAY_SIZE(PI6008_PKA210_supply_names)

struct regval_list {
	u32 addr;
	u32 data;
};

struct PI6008_PKA210_mode {
	struct v4l2_mbus_framefmt	format;
	struct v4l2_rect		crop;
	u64				pixel_rate;
	int				hts;
	int				vts;
	const struct regval_list	*reg_list;
	unsigned int			num_regs;
};

struct PI6008_PKA210 {
	struct v4l2_subdev		sd;
	struct media_pad		pad;
	struct mutex			lock;
	struct clk			*xclk;
	struct gpio_desc		*pwdn;
	struct regulator_bulk_data supplies[PI6008_PKA210_NUM_SUPPLIES];
	bool				clock_ncont;
	struct v4l2_ctrl_handler	ctrls;
	struct PI6008_PKA210_mode		*mode;
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

static inline struct PI6008_PKA210 *to_sensor(struct v4l2_subdev *sd)
{
	return container_of(sd, struct PI6008_PKA210, sd);
}


static struct regval_list PI6008_PKA210_FHD_YUV422[] = {
	{ 0xF070016C, 0x00000000 }, // [0] horizontal mirror, [1] vertical mirror
};

static const struct PI6008_PKA210_mode PI6008_PKA210_modes[] = {

	{
		.format = { 
			.code = MEDIA_BUS_FMT_UYVY8_1X16,
			//.code = MEDIA_BUS_FMT_YUYV8_1X16,
			.colorspace	= V4L2_COLORSPACE_SMPTE170M,
			//.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= PI6008_PKA210_PIXEL_ARRAY_WIDTH,
			.height		= PI6008_PKA210_PIXEL_ARRAY_HEIGHT
		},
		.crop = {
			.left		= 0,
			.top		= 0,
			.width		= PI6008_PKA210_PIXEL_ARRAY_WIDTH,
			.height		= PI6008_PKA210_PIXEL_ARRAY_HEIGHT
		},
		.pixel_rate	= PI6008_PKA210_PIXEL_RATE,
		.hts		= PI6008_PKA210_HORIZONTAL_TOTAL_SIZE,
		.vts		= PI6008_PKA210_VERTICAL_TOTAL_SIZE,
		.reg_list	= PI6008_PKA210_FHD_YUV422,
		.num_regs	= ARRAY_SIZE(PI6008_PKA210_FHD_YUV422)
	},
};

#define PI6008_PKA210_DEFAULT_MODE (&PI6008_PKA210_modes[0])
#define PI6008_PKA210_DEFAULT_FORMAT (PI6008_PKA210_modes[0].format)

