#include "pi6008_pka210.c"

// static int PI6008_PKA210_write32(struct v4l2_subdev *sd, u32 reg, u8 val)
// {
// 	unsigned char data[5] = {
// 		(reg >> 24) & 0xff,
// 		(reg >> 16) & 0xff,
// 		(reg >> 8)  & 0xff,
// 		reg & 0xff,
// 		val
// 	};

// 	struct i2c_client *client = v4l2_get_subdevdata(sd);
// 	int ret;

// 	ret = i2c_master_send(client, data, 5);

// 	if (ret == 5) {
// 		ret = 0;
// 	} else {
// 		dev_err(&client->dev, "%s: i2c write error, reg: 0x%08x\n",
// 			__func__, reg);
// 		return ret;
// 	}

// 	return 0;
// }

static int PI6008_PKA210_write32(struct v4l2_subdev *sd, u32 reg, u32 val)
{
	unsigned char data[8] = {
		(reg >> 24) & 0xff, (reg >> 16) & 0xff, (reg >> 8) & 0xff, reg & 0xff,
		(val >> 24) & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff
	};
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	ret = i2c_master_send(client, data, 8);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 8) {
		ret = 0;
	} else {
		dev_err(&client->dev, "%s: i2c write error, reg: %x\n",
			__func__, reg);
		return ret;
	}

	return 0;
}

static int PI6008_PKA210_write(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	unsigned char data[3] = { reg >> 8, reg & 0xff, val};
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = i2c_master_send(client, data, 3);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 3) {
		ret = 0;
	} else {
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return 0;
}

// static int PI6008_PKA210_read32(struct v4l2_subdev *sd, u32 reg, u8 *val)
// {
// 	struct i2c_client *client = v4l2_get_subdevdata(sd);
// 	u8 reg_buf[4];   // 4-byte register address
// 	u8 val_buf;      // 1-byte read value
// 	struct i2c_msg msg[2];
// 	int ret;

// 	/* Prepare 4-byte register address */
// 	reg_buf[0] = (reg >> 24) & 0xff;
// 	reg_buf[1] = (reg >> 16) & 0xff;
// 	reg_buf[2] = (reg >> 8) & 0xff;
// 	reg_buf[3] = reg & 0xff;

// 	/* First message: write the register address */
// 	msg[0].addr  = client->addr;
// 	msg[0].flags = 0;  // Write
// 	msg[0].len   = 4;
// 	msg[0].buf   = reg_buf;

// 	/* Second message: read 1 byte from the register */
// 	msg[1].addr  = client->addr;
// 	msg[1].flags = I2C_M_RD;
// 	msg[1].len   = 1;
// 	msg[1].buf   = &val_buf;

// 	/* Perform I2C transfer */
// 	ret = i2c_transfer(client->adapter, msg, 2);
// 	if (ret != 2) {
// 		dev_err(&client->dev, "%s: i2c read error, reg: 0x%08x, ret: %d\n",
// 			__func__, reg, ret);
// 		return ret >= 0 ? -EINVAL : ret;
// 	}

// 	*val = val_buf;
// 	return 0;
// }

static int PI6008_PKA210_read32(struct v4l2_subdev *sd, u32 reg, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 buf[4];
	struct i2c_msg msg[2];
	int ret;

	/* 4-byte register address */
	buf[0] = (reg >> 24) & 0xff;
	buf[1] = (reg >> 16) & 0xff;
	buf[2] = (reg >> 8) & 0xff;
	buf[3] = reg & 0xff;

	/* Write the register address */
	msg[0].addr = client->addr;
	msg[0].flags = client->flags; /* Write operation */
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	/* Read the 4-byte value */
	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD; /* Read operation */
	msg[1].buf = buf;
	msg[1].len = 4;

	/* Perform the I2C transaction */
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		dev_err(&client->dev, "%s: i2c slave(0x%X) read error, reg: %x = %d\n",
			__func__, client->addr, reg, ret);
		return ret >= 0 ? -EINVAL : ret;
	}

	/* Combine the 4-byte result into a single 32-bit value */
	*val = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];

	return 0;
}


static int PI6008_PKA210_read(struct v4l2_subdev *sd, u16 reg, u8 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msg[2];
	int ret;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		dev_err(&client->dev, "%s: i2c read error, reg: %x = %d\n",
			__func__, reg, ret);
		return ret >= 0 ? -EINVAL : ret;
	}

	*val = buf[0];

	return 0;
}

static int PI6008_PKA210_write32_array(struct v4l2_subdev *sd,
			      const struct regval_list *regs, int array_size)
{
	int i, ret;
	for (i = 0; i < array_size; i++) {
		ret = PI6008_PKA210_write32(sd, regs[i].addr, regs[i].data);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int PI6008_PKA210_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *try_img_fmt =
		v4l2_subdev_get_try_format(sd, fh->state, 0);
	struct v4l2_rect *try_crop;

	*try_img_fmt = PI6008_PKA210_DEFAULT_FORMAT;

	/* Initialize try_crop rectangle. */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, 0);
	try_crop->top = 0;
	try_crop->left = 0;
	try_crop->width = try_img_fmt->width;
	try_crop->height = try_img_fmt->height;

	return 0;
}

static int PI6008_PKA210_set_virtual_channel(struct v4l2_subdev *sd, int channel)
{
	u8 channel_id = 0;
	int ret = 0;
	channel_id &= ~(3 << 6);
	return ret;
}

static int PI6008_PKA210_init_regs(struct v4l2_subdev *sd)
{
	struct PI6008_PKA210 *sensor = to_sensor(sd);
	
	// write PI6008_PKA210 initial register values via i2c
	int ret = PI6008_PKA210_write32_array(sd, sensor->mode->reg_list, sensor->mode->num_regs);
	if (ret < 0)
		return ret;

	return 0;
}

static int PI6008_PKA210_stream_on(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (ret) {
		dev_err(&client->dev, "Failed to program sensor mode: %d\n", ret);
		return ret;
	}

	/* Apply customized values from user when stream starts. */
	ret =  __v4l2_ctrl_handler_setup(sd->ctrl_handler);
	dev_info(&client->dev, "%s(%d): ret %d\n", __func__, __LINE__, ret);
	if (ret)
		return ret;

	return 0;
}

static int PI6008_PKA210_stream_off(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_info(&client->dev, "%s(%d)\n", __func__, __LINE__);
	return 0;
}

static int PI6008_PKA210_power_on(struct device *dev)
{
	struct PI6008_PKA210 *sensor = dev_get_drvdata(dev);
	int ret;

	dev_info(dev, "PI6008_PKA210 power on\n");
	ret = regulator_bulk_enable(PI6008_PKA210_NUM_SUPPLIES, sensor->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		return ret;
	}

	if (sensor->pwdn) {
		gpiod_set_value_cansleep(sensor->pwdn, 0);
		dev_info(dev, "PWDN GPIO %d(%d)\n", gpiod_get_value(sensor->pwdn), __LINE__);
		msleep(20000);
	}

	// i don't know whether it can be applied or not
	
	ret = clk_prepare_enable(sensor->xclk);
	if (ret < 0) {
		dev_err(dev, "clk prepare enable failed\n");
		goto error_pwdn;
	}
	

	return 0;

error_pwdn:
	gpiod_set_value_cansleep(sensor->pwdn, 1);
	regulator_bulk_disable(PI6008_PKA210_NUM_SUPPLIES, sensor->supplies);

	return ret;
}

static int PI6008_PKA210_power_off(struct device *dev)
{
	struct PI6008_PKA210 *sensor = dev_get_drvdata(dev);

	dev_info(dev, "PI6008_PKA210 power off\n");

	clk_disable_unprepare(sensor->xclk); // i don't know if it is used or not
	gpiod_set_value_cansleep(sensor->pwdn, 1);
	regulator_bulk_disable(PI6008_PKA210_NUM_SUPPLIES, sensor->supplies);
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int PI6008_PKA210_sensor_get_register(struct v4l2_subdev *sd,
				      struct v4l2_dbg_register *reg)
{
	int ret;
	u32 val;

	ret = PI6008_PKA210_read32(sd, reg->reg & 0xffffffff, &val);
	if (ret < 0)
		return ret;

	pr_info("%s(%d): (%ul, %ul)\n", __func__, __LINE__, reg->reg, reg->val);
	reg->val = val;
	reg->size = 4;

	return 0;
}

static int PI6008_PKA210_sensor_set_register(struct v4l2_subdev *sd,
				      const struct v4l2_dbg_register *reg)
{
	pr_info("%s(%d): (%ul, %ul)\n", __func__, __LINE__, reg->reg, reg->val);
	return PI6008_PKA210_write32(sd, reg->reg & 0xffffffff, reg->val & 0xffffffff);
}
#endif

static const struct v4l2_rect *
__PI6008_PKA210_get_pad_crop(struct PI6008_PKA210 *PI6008_PKA210,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&PI6008_PKA210->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &PI6008_PKA210->mode->crop;
	}

	return NULL;
}

static int PI6008_PKA210_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct PI6008_PKA210 *sensor = to_sensor(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);
	if (sensor->streaming == enable) {
		mutex_unlock(&sensor->lock);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev); // PI6008_PKA210_power_on func call
		if (ret < 0) {
			goto error_unlock;
		}

		ret = PI6008_PKA210_stream_on(sd);
		if (ret < 0) {
			goto error_pm;
		}
	} else {
		ret = PI6008_PKA210_stream_off(sd);
		if (ret < 0) {
			goto error_pm;
		}
		pm_runtime_put(&client->dev);  // PI6008_PKA210_power_off func call
	}

	sensor->streaming = enable;
	mutex_unlock(&sensor->lock);
	
	return 0;

error_pm:
	pm_runtime_put(&client->dev);
error_unlock:
	mutex_unlock(&sensor->lock);

	return ret;
}

static int PI6008_PKA210_g_frame_interval(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_frame_interval *interval)
{
	/*
	interval->interval.numerator = 1;
	interval->interval.denominator = 30;
	*/
	return 0;
}

static int PI6008_PKA210_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= NUM_MBUS_CODES)
		return -EINVAL;
	if (code->pad)
		return -EINVAL;

	code->code = mbus_codes[code->index];
	return 0;
}

static int PI6008_PKA210_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sPI6008_PKA210_get_mbus_coded_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index)
		return -EINVAL;
	if (fse->pad)
		return -EINVAL;

	fse->min_width = MIN_WIDTH;
	fse->max_width = MAX_WIDTH;
	fse->min_height = MIN_HEIGHT;
	fse->max_height = MAX_HEIGHT;

	return 0;
}

static int PI6008_PKA210_get_pad_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct PI6008_PKA210 *sensor = to_sensor(sd);

	if (fmt->pad)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&sensor->sd, sd_state,
						   fmt->pad);
		fmt->format = *try_fmt;
	} else {
		fmt->format = sensor->mode->format;
	}

	return 0;
}

static int PI6008_PKA210_set_pad_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct PI6008_PKA210 *sensor = to_sensor(sd);
	const struct PI6008_PKA210_mode *mode;

	if (format->pad)
		return -EINVAL;

	mode = v4l2_find_nearest_size(PI6008_PKA210_modes, ARRAY_SIZE(PI6008_PKA210_modes),
				      format.width, format.height,
				      fmt->width, fmt->height);

	/* Update the sensor mode and apply at it at streamon time. */
	mutex_lock(&sensor->lock);
	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_format(sd, sd_state, format->pad) = mode->format;
	} else {
		// we don't need to control for exposure now
		
		int exposure_max, exposure_def;
		int hblank, vblank;

		sensor->mode = mode;
		__v4l2_ctrl_modify_range(sensor->pixel_rate, mode->pixel_rate,
					 mode->pixel_rate, 1, mode->pixel_rate);

		hblank = mode->hts - mode->format.width;
		__v4l2_ctrl_modify_range(sensor->hblank, hblank,
					 PI6008_PKA210_HTS_MAX - mode->format.width, 1,
					 hblank);

		vblank = mode->vts - mode->format.height;
		__v4l2_ctrl_modify_range(sensor->vblank, PI6008_PKA210_VBLANK_MIN,
					 PI6008_PKA210_VTS_MAX - mode->format.height,
					 1, vblank);
		__v4l2_ctrl_s_ctrl(sensor->vblank, vblank);

		/*
		exposure_max = mode->vts - 4;
		exposure_def = min(exposure_max, PI6008_PKA210_EXPOSURE_DEFAULT);
		__v4l2_ctrl_modify_range(sensor->exposure,
					 sensor->exposure->minimum,
					 exposure_max, sensor->exposure->step,
					 exposure_def);
		 */
	}
	*fmt = mode->format;

	int i = 0;
	for (i = 0; i < NUM_MBUS_CODES; i++)
		if (mbus_codes[i] == format->format.code)
			break;
	if (i >= NUM_MBUS_CODES)
		i = 0;
	
	fmt->code = mbus_codes[i];
	mutex_unlock(&sensor->lock);

	return 0;
}

static int PI6008_PKA210_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct PI6008_PKA210 *sensor = to_sensor(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = sensor->mode->format.width;
		sel->r.height = sensor->mode->format.height;

		return 0;
	}

	return -EINVAL;
}

/* Subdev core operations registration */
static const struct v4l2_subdev_core_ops PI6008_PKA210_subdev_core_ops = {
	.subscribe_event	= v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event	= v4l2_event_subdev_unsubscribe,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= PI6008_PKA210_sensor_get_register,
	.s_register		= PI6008_PKA210_sensor_set_register,
#endif
};

static const struct v4l2_subdev_video_ops PI6008_PKA210_subdev_video_ops = {
	.s_stream =		PI6008_PKA210_s_stream,
	.g_frame_interval = PI6008_PKA210_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops PI6008_PKA210_subdev_pad_ops = {
	.enum_mbus_code		= PI6008_PKA210_enum_mbus_code,
	.get_fmt		= PI6008_PKA210_get_pad_fmt,
	.set_fmt		= PI6008_PKA210_set_pad_fmt,
	.get_selection		= PI6008_PKA210_get_selection,
	.enum_frame_size	= PI6008_PKA210_enum_frame_size,
};

static const struct v4l2_subdev_ops PI6008_PKA210_subdev_ops = {
	.core		= &PI6008_PKA210_subdev_core_ops,
	.video		= &PI6008_PKA210_subdev_video_ops,
	.pad		= &PI6008_PKA210_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops PI6008_PKA210_subdev_internal_ops = {
	.open = PI6008_PKA210_open,
};

static int PI6008_PKA210_detect(struct v4l2_subdev *sd)
{
	int ret = 0;

	return ret;
}

static int PI6008_PKA210_s_mirror( struct v4l2_subdev *sd, u16 reg, u8 ctrl_val, int mirror_mode)
{
	int ret;
	u32 val;
	ret = PI6008_PKA210_read32(sd, PI6008_PKA210_REG_MIRROR, &val);
	switch (mirror_mode) {
		case PI6008_PKA210_MIRROR_H:
			val = (val & 0x02) | (ctrl_val & 0x01);
		break;
		case PI6008_PKA210_MIRROR_V:
			val = (val & 0x01) | ((ctrl_val & 0x01) << 1);
		break;
	}
	
	ret = PI6008_PKA210_write32(sd, PI6008_PKA210_REG_MIRROR, val);

	return ret;
}

static int PI6008_PKA210_s_data_lane_ctrl(struct v4l2_subdev *sd, u8 val)
{
	int ret = 0;
	u8 reg_val = 0x00;
	u32 reg_addr = 0xf0801018;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	// 하위 네비트는 data 0 line, 상위 4비트는 data 1 line
	switch (val) {
	case 0: // normal operation
		reg_val = 0x00;
		dev_info(&client->dev, "Data normal mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 1: // LP-11
		reg_val = 0x44;
		dev_info(&client->dev, "Data LP-11 mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 2: // LP-01
		reg_val = 0x22;
		dev_info(&client->dev, "Data LP-01 mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 3: // LP-00
		reg_val = 0x11;
		dev_info(&client->dev, "Data LP-00 mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 4: // HS-1
		reg_val = 0x66;
		dev_info(&client->dev, "Data HS-1(P-high, N-low) mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 5: // HS-0
		reg_val = 0x55;
		dev_info(&client->dev, "Data HS-0(P-low, N-high) mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 6: // LP-10
		reg_val = 0x33;
		dev_info(&client->dev, "Data HS-0(P-low, N-high) mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 7: // Hi-z
		reg_val = 0x77;
		dev_info(&client->dev, "Data Hi-z(High impedence) mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 8: // ULP
		reg_val = 0x88;
		dev_info(&client->dev, "Data Ultra Low Power mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 9: // power down
		reg_val = 0xAA;
		dev_info(&client->dev, "Data power down mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	default:
		dev_info(&client->dev, "Not available custom control set to: 0x%x\n", val);
		break;
	}
	return ret;
}


static int PI6008_PKA210_s_clk_lane_ctrl(struct v4l2_subdev *sd, u32 reg_val, u8 val)
{
	int ret = 0;
	u32 upper = 0;
	u32 reg_addr = 0xf0801004;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	// 하위 네비트는 그대로 가고, 상위 4비트로 모드를 바꿔 준다
	switch (val) {
	case 0:
		dev_info(&client->dev, "For initialization\n");
		break;
	case 1: // normal operation
		upper = 0x00;
		reg_val = (reg_val & 0x0F) | (upper << 4);
		dev_info(&client->dev, "Clk normal mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 2: // LP-11
		upper = 0x04;
		reg_val = (reg_val & 0x0F) | (upper << 4);
		dev_info(&client->dev, "Clk LP-11 mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 3: // LP-01
		upper = 0x02;
		reg_val = (reg_val & 0x0F) | (upper << 4);
		dev_info(&client->dev, "Clk LP-01 mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 4: // LP-00
		upper = 0x01;
		reg_val = (reg_val & 0x0F) | (upper << 4);
		dev_info(&client->dev, "Clk LP-00 mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 5: // HS-1
		upper = 0x06;
		reg_val = (reg_val & 0x0F) | (upper << 4);
		dev_info(&client->dev, "Clk HS-1(P-high, N-low) mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 6: // HS-0
		upper = 0x05;
		reg_val = (reg_val & 0x0F) | (upper << 4);
		dev_info(&client->dev, "Clk HS-0(P-low, N-high) mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 7: // LP-10
		upper = 0x03;
		reg_val = (reg_val & 0x0F) | (upper << 4);
		dev_info(&client->dev, "Clk LP-10(P-low, N-high) mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 8: // Hi-z
		upper = 0x07;
		reg_val = (reg_val & 0x0F) | (upper << 4);
		dev_info(&client->dev, "Clk Hi-z(High impedence) mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 9: // ULP
		upper = 0x08;
		reg_val = (reg_val & 0x0F) | (upper << 4);
		dev_info(&client->dev, "Clk Ultra Low Power mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	case 10: // Serialize test mode
		upper = 0x09;
		reg_val = (reg_val & 0x0F) | (upper << 4);
		dev_info(&client->dev, "Serialize test mode: 0x%x\n", reg_val);
		ret = PI6008_PKA210_write32(sd, reg_addr, reg_val);
		break;
	default:
		dev_info(&client->dev, "Not available custom control set to: 0x%x\n", val);
		break;
	}
	return ret;
}

static int PI6008_PKA210_s_custom(struct v4l2_subdev *sd, u8 val)
{
	int ret = 0;
	u32 reg_val = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	
	ret = PI6008_PKA210_read32(sd, 0xf0801004, &reg_val); // mipi clock lanes control
	dev_info(&client->dev, "MIPI clock reg value is: 0x%x\n", reg_val);

	// for test
	static u32 device_val = 0x00;
	if(val < 11)
	{
		ret = PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, val);
	}
	else
	{
		switch (val) {
		case 11:
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 2); // LP-11
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 3); // LP-01
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 4); // LP-00
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 5); // HS-1
			break;
		case 12:
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 2); // LP-11
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 3); // LP-01
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 4); // LP-00
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 6); // HS-0
			break;
		case 13:
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 2); // LP-11
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 3); // LP-01
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 4); // LP-00
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 5); // HS-1
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 1); // Normal operation
			break;
		case 14:
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 2); // LP-11
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 3); // LP-01
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 4); // LP-00
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 6); // HS-0
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 1); // Normal operation
			break;
		case 15:
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 2); // LP-11
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 3); // LP-01
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 4); // LP-00
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 5); // HS-1
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 6); // HS-0
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 1); // Normal operation
			break;
		case 16: // doesn't work
			PI6008_PKA210_s_data_lane_ctrl(sd, 1); // LP-11
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 2); // LP-11
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 3); // LP-01
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 4); // LP-00
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 5); // HS-1
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 6); // HS-0
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 1); // Normal operation
			PI6008_PKA210_s_data_lane_ctrl(sd, 0); // normal operation
			break;
		case 17:
			ret = PI6008_PKA210_write32(sd, 0xF0801010, 0x10); // mipi disable
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 2); // LP-11
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 3); // LP-01
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 4); // LP-00
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 5); // HS-1
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 6); // HS-0
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 1); // Normal operation
			ret = PI6008_PKA210_write32(sd, 0xF0801010, 0x50); // mipi enable
			break;
		case 20: // doesn't work
			PI6008_PKA210_s_data_lane_ctrl(sd, 1); // LP-11
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 2); // LP-11
			PI6008_PKA210_s_clk_lane_ctrl(sd, reg_val, 1); // Normal operation
			PI6008_PKA210_s_data_lane_ctrl(sd, 0); // normal operation
			break;
		case 30:
			dev_info(&client->dev, "None-continouse mode: 0x%x\n", 0x40);
			ret = PI6008_PKA210_write32(sd, 0xF0801010, 0x40); // mipi none-continouse mode
			break;
		case 31:
			dev_info(&client->dev, "Continouse mode: 0x%x\n", 0x50);
			ret = PI6008_PKA210_write32(sd, 0xF0801010, 0x50); // mipi continouse mode
			break;
		case 40:
			ret = PI6008_PKA210_read32(sd, 0xf050001c, &device_val); // PI6008 device ID
			dev_info(&client->dev, "PI6008 device ID(0x%x): 0x%x\n", 0xf050001c, device_val);  // expected 0x20171211
			break;
		case 41:
			ret = PI6008_PKA210_read32(sd, 0xf0801000, &device_val);
			dev_info(&client->dev, "MIPI MIPI Controller Status(0x%x): 0x%x\n", 0xf0801000, device_val);
			break;
		case 42:
			ret = PI6008_PKA210_write32(sd, 0xf0801000, 0xc1); //0100 0001
			dev_info(&client->dev, "MIPI MIPI Controller test mode enable(0x%x): 0x%x\n", 0xf0801000, device_val);
			break;
		case 43:
			ret = PI6008_PKA210_write32(sd, 0xf0801000, 0xc0); //0100 0000
			dev_info(&client->dev, "MIPI MIPI Controller test mode disable(0x%x): 0x%x\n", 0xf0801000, device_val);
			break;
		default:
			break;
		}
	}
	
	return ret;
}

static int PI6008_PKA210_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct PI6008_PKA210 *sensor = container_of(ctrl->handler,
					    struct PI6008_PKA210, ctrls);
	struct v4l2_subdev *sd = &sensor->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	//dev_info(&client->dev, "dbg_info: %s, line: %d, id: %d\n", __func__, __LINE__, ctrl->id);
	
	switch (ctrl->id) {
	case V4L2_CID_PIXEL_RATE:
		/* Read-only, but we adjust it based on mode. */
		break;
	case V4L2_CID_HFLIP:
		PI6008_PKA210_s_mirror(sd, PI6008_PKA210_REG_MIRROR, ctrl->val, PI6008_PKA210_MIRROR_H);
		break;
	case V4L2_CID_VFLIP:
		PI6008_PKA210_s_mirror(sd, PI6008_PKA210_REG_MIRROR, ctrl->val, PI6008_PKA210_MIRROR_V);
		break;
	case V4L2_CID_HBLANK:
		break;
	case V4L2_CID_VBLANK:
		break;
	case V4L2_CID_LINK_FREQ:
		break;
	case V4L2_CID_PI6008_PKA210_CUSTOM_SETTING:
		dev_info(&client->dev, "Custom control set to: %d\n", ctrl->val);
		if (ctrl->val != 0) {
			PI6008_PKA210_s_custom(sd, ctrl->val);
		}
		break;
	default:
		dev_info(&client->dev,
			 "Control (id:0x%x, val:0x%x) not supported\n",
			 ctrl->id, ctrl->val);
		return -EINVAL;
	};
	
	return ret;
}

static const struct v4l2_ctrl_ops PI6008_PKA210_ctrl_ops = {
	.s_ctrl = PI6008_PKA210_s_ctrl,
};

static const int PI6008_PKA210_configure_regulators(struct device *dev, struct PI6008_PKA210 *sensor)
{
	unsigned int i = 0;
	
	for (i = 0;i < PI6008_PKA210_NUM_SUPPLIES; i++)
	{
		sensor->supplies[i].supply = PI6008_PKA210_supply_names[i];
	}
	
	int nRet = devm_regulator_bulk_get(dev, PI6008_PKA210_NUM_SUPPLIES, sensor->supplies);
	return nRet;
}

static const struct v4l2_ctrl_config PI6008_PKA210_custom_ctrl_cfg = {
	.ops = &PI6008_PKA210_ctrl_ops,
	.id = V4L2_CID_PI6008_PKA210_CUSTOM_SETTING,
	.name = "PI6008_PKA210_custom_set",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 128,
	.step = 1,
	.def = 0,
	.flags = 0,
};

static int PI6008_PKA210_init_controls(struct PI6008_PKA210 *sensor, struct device *dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	int hblank, vblank, exposure_max, exposure_def;
	struct v4l2_fwnode_device_properties props;

	int ret = v4l2_ctrl_handler_init(&sensor->ctrls, 8);
	if (ret) {
		dev_err(dev, "v4l2_ctrl_handler_init failed: %d\n", ret);
		return ret;
	}	

	/*
	v4l2_ctrl_new_std(&sensor->ctrls, &PI6008_PKA210_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, 0);
	
	v4l2_ctrl_new_std(&sensor->ctrls, &PI6008_PKA210_ctrl_ops,
			  V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 0);
	
	v4l2_ctrl_new_std_menu(&sensor->ctrls, &PI6008_PKA210_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL,
			       0, V4L2_EXPOSURE_MANUAL);
	
	exposure_max = sensor->mode->vts - 4;
	exposure_def = min(exposure_max, PI6008_PKA210_EXPOSURE_DEFAULT);
	sensor->exposure = v4l2_ctrl_new_std(&sensor->ctrls, &PI6008_PKA210_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     PI6008_PKA210_EXPOSURE_MIN,
					     exposure_max, PI6008_PKA210_EXPOSURE_STEP,
					     exposure_def);

	// min: 16 = 1.0x; max (10 bits); default: 32 = 2.0x.
	v4l2_ctrl_new_std(&sensor->ctrls, &PI6008_PKA210_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 16, 1023, 1, 32);
	*/
	
	// By default, PIXEL_RATE is read only, but it does change per mode
	sensor->pixel_rate = v4l2_ctrl_new_std(&sensor->ctrls, &PI6008_PKA210_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       sensor->mode->pixel_rate,
					       sensor->mode->pixel_rate, 1,
					       sensor->mode->pixel_rate);
	if (sensor->ctrls.error)
		dev_err(dev, "Pixel rate init failed (%d)\n", sensor->ctrls.error);

	sensor->link_freq =
		v4l2_ctrl_new_int_menu(&sensor->ctrls, &PI6008_PKA210_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(PI6008_PKA210_link_freq_menu) - 1, 0,
				       PI6008_PKA210_link_freq_menu);
	if (sensor->ctrls.error)
		dev_err(dev, "Link freq init failed (%d)\n", sensor->ctrls.error);
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	
	hblank = sensor->mode->hts - sensor->mode->format.width;
	sensor->hblank = v4l2_ctrl_new_std(&sensor->ctrls, &PI6008_PKA210_ctrl_ops,
					   V4L2_CID_HBLANK, hblank,
					   PI6008_PKA210_HTS_MAX -
					   sensor->mode->format.width, 1,
					   hblank);
	if (sensor->ctrls.error)
		dev_err(dev, "H blank control init failed (%d)\n", sensor->ctrls.error);

	vblank = sensor->mode->vts - sensor->mode->format.height;
	sensor->vblank = v4l2_ctrl_new_std(&sensor->ctrls, &PI6008_PKA210_ctrl_ops,
					   V4L2_CID_VBLANK, PI6008_PKA210_VBLANK_MIN,
					   PI6008_PKA210_VTS_MAX -
					   sensor->mode->format.height, 1,
					   vblank);
	if (sensor->ctrls.error)
		dev_err(dev, "V blank control init failed (%d)\n", sensor->ctrls.error);
					   
	/*
	v4l2_ctrl_new_std_menu_items(&sensor->ctrls, &PI6008_PKA210_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(PI6008_PKA210_test_pattern_menu) - 1,
				     0, 0, PI6008_PKA210_test_pattern_menu);
	 */
	
	sensor->hflip = v4l2_ctrl_new_std(&sensor->ctrls, &PI6008_PKA210_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (sensor->ctrls.error)
		dev_err(dev, "H flip control init failed (%d)\n", sensor->ctrls.error);
	if (sensor->hflip)
		sensor->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	sensor->vflip = v4l2_ctrl_new_std(&sensor->ctrls, &PI6008_PKA210_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);	
	if (sensor->ctrls.error)
		dev_err(dev, "V flip control init failed (%d)\n", sensor->ctrls.error);
	if (sensor->vflip)
		sensor->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	sensor->custom_ctrl = v4l2_ctrl_new_custom(&sensor->ctrls,
		&PI6008_PKA210_custom_ctrl_cfg,
		NULL);
	// sensor->custom_ctrl->flags |= V4L2_CTRL_FLAG_DISABLED;
	dev_info(dev, "sensor->custom_ctrl = 0x%x, error = %d\n",
		sensor->custom_ctrl, sensor->ctrls.error);
	
	v4l2_fwnode_device_parse(dev, &props);

	v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &PI6008_PKA210_ctrl_ops,
					&props);
	
	if (sensor->ctrls.error)
		goto handler_free;


	sensor->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->sd.ctrl_handler = &sensor->ctrls;
	
	return 0;

handler_free:
	dev_err(&client->dev, "%s Controls initialization failed (%d)\n",
		__func__, sensor->ctrls.error);
	v4l2_ctrl_handler_free(&sensor->ctrls);

	return sensor->ctrls.error;
}


static int PI6008_PKA210_parse_dt(struct PI6008_PKA210 *sensor, struct device_node *np, struct device *dev)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct device_node *ep;
	int ret = 0;

	ep = of_graph_get_next_endpoint(np, NULL);
	if (!ep)
		return -EINVAL;

	//ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &bus_cfg);
	ret = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(ep), &bus_cfg);
	if (ret)
		goto out;

	sensor->clock_ncont = bus_cfg.bus.mipi_csi2.flags &
			      V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK;
	dev_info(dev, "camera none_continuous %d\n", sensor->clock_ncont);
	sensor->lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
	dev_info(dev, "camera num data lanes %d\n", sensor->lanes);
	
	dev_info(dev, "camera num link freq %d\n", bus_cfg.nr_of_link_frequencies);
	if (bus_cfg.nr_of_link_frequencies != 0)
	{
		dev_info(dev, "camera link freq %lld\n", bus_cfg.link_frequencies[0]);
		if (bus_cfg.nr_of_link_frequencies != 1 ||
			(bus_cfg.link_frequencies[0] != PI6008_PKA210_DEFAULT_LINK_FREQ)) {
			dev_err(dev, "Link frequency not supported: %lld\n",
				bus_cfg.link_frequencies[0]);
			//goto out;
		}
		
		dev_info(dev, "MIPI %d lanes, link_freq %lld, none_continuous clock %d\n",
			sensor->lanes, bus_cfg.link_frequencies[0], sensor->clock_ncont);
	}
	return ret;

out:
	of_node_put(ep);

	return -EINVAL;
}

static int PI6008_PKA210_probe(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct device *dev = &client->dev;
	struct PI6008_PKA210 *sensor;
	struct v4l2_subdev *sd;
	u32 xclk_freq;
	int ret;

	dev_info(dev, "PIXELPLUS PI6008_PKA210 camera driver starts\n");

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_OF) && np) {
		ret = PI6008_PKA210_parse_dt(sensor, np, dev);
		if (ret) {
			dev_err(dev, "DT parsing error: %d\n", ret);
			return ret;
		}
	}

	sensor->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "could not get xclk");
		return PTR_ERR(sensor->xclk);
	}

	xclk_freq = clk_get_rate(sensor->xclk);
	if (xclk_freq != 25000000) {
		dev_err(dev, "Unsupported clock frequency: %u\n", xclk_freq);
		return -EINVAL;
	}

	// CIS doesn't get power from the rpi board, later we will change that rpi board supplies power to CIS
	/* Request the power down GPIO asserted. */
	sensor->pwdn = devm_gpiod_get_optional(dev, "pwdn", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->pwdn)) {
		dev_err(dev, "Failed to get 'pwdn' gpio\n");
		return -EINVAL;
	}

	ret = PI6008_PKA210_configure_regulators(dev, sensor);
	if (ret) {
		dev_err(dev, "Failed to get power regulators(%d)\n", ret);
		return ret;
	}

	mutex_init(&sensor->lock);

	sensor->mode = PI6008_PKA210_DEFAULT_MODE;
	dev_info(dev, "PIXELPLUS PI6008_PKA210 camera init controls\n");
	ret = PI6008_PKA210_init_controls(sensor, dev);
	if (ret)
		goto mutex_destroy;

	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &PI6008_PKA210_subdev_ops);
	sd->internal_ops = &PI6008_PKA210_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE; //MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret < 0)
		goto ctrl_handler_free;

	// power from an external power supply, we don't need it
	/*
	ret = PI6008_PKA210_power_on(dev);
	if (ret)
		goto entity_cleanup;
	

	ret = PI6008_PKA210_detect(sd);
	if (ret < 0)
		goto power_off;
	*/

	ret = PI6008_PKA210_power_on(dev);
	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret < 0)
	{
		dev_err(dev, "dbg_info: %s, line: %d, ret: %d\n", __func__, __LINE__, ret);
		goto power_off;
	}
	
	// ret = PI6008_PKA210_init_regs(sd);
	if (ret) {
		dev_err(dev, "Failed to initialize PI6008_PKA210 registers(%d)\n", ret);
		//return ret;
	}

	// power from an external power supply, we don't need it
	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
	
	dev_info(dev, "PIXELPLUS PI6008_PKA210 camera driver probed\n");

	TestRegs(dev, sd);

	return 0;

power_off:
	PI6008_PKA210_power_off(dev);
entity_cleanup:
	media_entity_cleanup(&sd->entity);
ctrl_handler_free:
	v4l2_ctrl_handler_free(&sensor->ctrls);
mutex_destroy:
	mutex_destroy(&sensor->lock);

	return ret;
}

void TestRegs(struct device *dev, struct v4l2_subdev *sd)
{
	int ret = 0;
	// below are for test
	u32 nReg = 0xf0600004;
	u32 nVal = 0x00;
	
	// ret = PI6008_PKA210_write32(sd, 0xF0801010, 0x40); // mipi none-continouse mode

	ret = PI6008_PKA210_read32(sd, nReg, &nVal); // check the sensor id
	dev_info(dev, "I2C Sensor ID success (0x%X, 0x%X)\n", nReg, nVal);

	nReg = 0xf0801000;
	ret = PI6008_PKA210_read32(sd, nReg, &nVal); // check the sensor id
	dev_info(dev, "I2C MIPI enable success (0x%X, 0x%X)\n", nReg, nVal);
}

static void PI6008_PKA210_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct PI6008_PKA210 *sensor = to_sensor(sd);

	dev_info(&client->dev, "PIXELPLUS PI6008_PKA210 driver is removed\n");

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);

	v4l2_ctrl_handler_free(&sensor->ctrls);
	v4l2_device_unregister_subdev(sd);

	pm_runtime_disable(&client->dev);

	mutex_destroy(&sensor->lock);
}

static const struct dev_pm_ops PI6008_PKA210_pm_ops = {
	SET_RUNTIME_PM_OPS(PI6008_PKA210_power_off, PI6008_PKA210_power_on, NULL)
};

static const struct i2c_device_id PI6008_PKA210_id[] = {
	{ "px6008_pka210", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, PI6008_PKA210_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id PI6008_PKA210_of_match[] = {
	{ .compatible = "pixelplus,pi6008_pka210" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, PI6008_PKA210_of_match);
#endif

static struct i2c_driver PI6008_PKA210_driver = {
	.driver = {
		.of_match_table = of_match_ptr(PI6008_PKA210_of_match),
		.name	= "PI6008_PKA210",
		.pm	= &PI6008_PKA210_pm_ops,
	},
	.probe		= PI6008_PKA210_probe,
	.remove		= PI6008_PKA210_remove,
	.id_table	= PI6008_PKA210_id,
};

module_i2c_driver(PI6008_PKA210_driver);

MODULE_AUTHOR("Sanghyeon Kang <info@pixelplus.com>");
MODULE_DESCRIPTION("MIPI-CSI2 V4L2 driver for PIXELPLUS PI6008_PKA210 sensors");
MODULE_LICENSE("GPL v2");

