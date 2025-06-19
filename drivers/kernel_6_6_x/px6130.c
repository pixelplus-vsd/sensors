#include "px6130.h"

static int PX6130_write32(struct v4l2_subdev *sd, u32 reg, u32 val)
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

static int PX6130_read32(struct v4l2_subdev *sd, u32 reg, u32 *val)
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

static int PX6130_write32_array(struct v4l2_subdev *sd,
			      const struct regval_list *regs, int array_size)
{
	int i, ret;
	for (i = 0; i < array_size; i++) {
		ret = PX6130_write32(sd, regs[i].addr, regs[i].data);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int PX6130_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *try_img_fmt =
		v4l2_subdev_get_try_format(sd, fh->state, 0);
	struct v4l2_rect *try_crop;

	*try_img_fmt = PX6130_DEFAULT_FORMAT;

	/* Initialize try_crop rectangle. */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, 0);
	try_crop->top = 0;
	try_crop->left = 0;
	try_crop->width = try_img_fmt->width;
	try_crop->height = try_img_fmt->height;

	return 0;
}

static int PX6130_set_virtual_channel(struct v4l2_subdev *sd, int channel)
{
	u8 channel_id = 0;
	int ret = 0;
	channel_id &= ~(3 << 6);
	return ret;
}

static int PX6130_init_regs(struct v4l2_subdev *sd)
{
	struct PX6130 *sensor = to_sensor(sd);
	
	// write PX6130 initial register values via i2c
	int ret = PX6130_write32_array(sd, sensor->mode->reg_list, sensor->mode->num_regs);
	if (ret < 0)
		return ret;

	return 0;
}

static int PX6130_stream_on(struct v4l2_subdev *sd)
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

static int PX6130_stream_off(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_info(&client->dev, "%s(%d)\n", __func__, __LINE__);
	return 0;
}

static int PX6130_power_on(struct device *dev)
{
	struct PX6130 *sensor = dev_get_drvdata(dev);
	int ret;

	dev_info(dev, "PX6130 power on\n");
	ret = regulator_bulk_enable(PX6130_NUM_SUPPLIES, sensor->supplies);
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
	regulator_bulk_disable(PX6130_NUM_SUPPLIES, sensor->supplies);

	return ret;
}

static int PX6130_power_off(struct device *dev)
{
	struct PX6130 *sensor = dev_get_drvdata(dev);

	dev_info(dev, "PX6130 power off\n");

	clk_disable_unprepare(sensor->xclk); // i don't know if it is used or not
	gpiod_set_value_cansleep(sensor->pwdn, 1);
	regulator_bulk_disable(PX6130_NUM_SUPPLIES, sensor->supplies);
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int PX6130_sensor_get_register(struct v4l2_subdev *sd,
				      struct v4l2_dbg_register *reg)
{
	int ret;
	u32 val;

	ret = PX6130_read32(sd, reg->reg & 0xffffffff, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = 4;

	return 0;
}

static int PX6130_sensor_set_register(struct v4l2_subdev *sd,
				      const struct v4l2_dbg_register *reg)
{
	return PX6130_write32(sd, reg->reg & 0xffffffff, reg->val & 0xffffffff);
}
#endif

static const struct v4l2_rect *
__PX6130_get_pad_crop(struct PX6130 *PX6130,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&PX6130->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &PX6130->mode->crop;
	}

	return NULL;
}

static int PX6130_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct PX6130 *sensor = to_sensor(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);
	if (sensor->streaming == enable) {
		mutex_unlock(&sensor->lock);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev); // PX6130_power_on func call
		if (ret < 0) {
			goto error_unlock;
		}

		ret = PX6130_stream_on(sd);
		if (ret < 0) {
			goto error_pm;
		}
	} else {
		ret = PX6130_stream_off(sd);
		if (ret < 0) {
			goto error_pm;
		}
		pm_runtime_put(&client->dev);  // PX6130_power_off func call
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

static int PX6130_g_frame_interval(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_frame_interval *interval)
{
	/*
	interval->interval.numerator = 1;
	interval->interval.denominator = 30;
	*/
	return 0;
}

static int PX6130_enum_mbus_code(struct v4l2_subdev *sd,
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

static int PX6130_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sPX6130_get_mbus_coded_state,
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

static int PX6130_get_pad_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct PX6130 *sensor = to_sensor(sd);

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

static int PX6130_set_pad_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct PX6130 *sensor = to_sensor(sd);
	const struct PX6130_mode *mode;

	if (format->pad)
		return -EINVAL;

	mode = v4l2_find_nearest_size(PX6130_modes, ARRAY_SIZE(PX6130_modes),
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
					 PX6130_HTS_MAX - mode->format.width, 1,
					 hblank);

		vblank = mode->vts - mode->format.height;
		__v4l2_ctrl_modify_range(sensor->vblank, PX6130_VBLANK_MIN,
					 PX6130_VTS_MAX - mode->format.height,
					 1, vblank);
		__v4l2_ctrl_s_ctrl(sensor->vblank, vblank);

		/*
		exposure_max = mode->vts - 4;
		exposure_def = min(exposure_max, PX6130_EXPOSURE_DEFAULT);
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

static int PX6130_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct PX6130 *sensor = to_sensor(sd);

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
static const struct v4l2_subdev_core_ops PX6130_subdev_core_ops = {
	.subscribe_event	= v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event	= v4l2_event_subdev_unsubscribe,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= PX6130_sensor_get_register,
	.s_register		= PX6130_sensor_set_register,
#endif
};

static const struct v4l2_subdev_video_ops PX6130_subdev_video_ops = {
	.s_stream =		PX6130_s_stream,
	.g_frame_interval = PX6130_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops PX6130_subdev_pad_ops = {
	.enum_mbus_code		= PX6130_enum_mbus_code,
	.get_fmt		= PX6130_get_pad_fmt,
	.set_fmt		= PX6130_set_pad_fmt,
	.get_selection		= PX6130_get_selection,
	.enum_frame_size	= PX6130_enum_frame_size,
};

static const struct v4l2_subdev_ops PX6130_subdev_ops = {
	.core		= &PX6130_subdev_core_ops,
	.video		= &PX6130_subdev_video_ops,
	.pad		= &PX6130_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops PX6130_subdev_internal_ops = {
	.open = PX6130_open,
};

static int PX6130_detect(struct v4l2_subdev *sd)
{
	int ret = 0;

	return ret;
}

static int PX6130_s_mirror( struct v4l2_subdev *sd, u16 reg, u8 ctrl_val, int mirror_mode)
{
	int ret;
	u32 val;
	ret = PX6130_read32(sd, PX6130_REG_MIRROR, &val);
	switch (mirror_mode) {
		case PX6130_MIRROR_H:
			val = (val & 0x02) | (ctrl_val & 0x01);
		break;
		case PX6130_MIRROR_V:
			val = (val & 0x01) | ((ctrl_val & 0x01) << 1);
		break;
	}
	
	ret = PX6130_write32(sd, PX6130_REG_MIRROR, val);

	return ret;
}

static int PX6130_s_custom(struct v4l2_subdev *sd, u8 val)
{
	int ret = 0;
	u32 reg_val = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	
	ret = PX6130_read32(sd, 0xf0801004, &reg_val); // mipi clock lanes control
	dev_info(&client->dev, "MIPI clock reg value is: 0x%x\n", reg_val);

	// for test
	static u32 device_val = 0x00;
	switch (val) {
		case 0:
			dev_info(&client->dev, "Nothing to do: DEFAULT\n");
			break;
		case 1:
			ret = PX6130_write32(sd, 0xF0801070, val); 
			dev_info(&client->dev, "MIPI Time LPX: 0xF0801070, 0x%x\n", val);
			break;
		case 2:
			ret = PX6130_write32(sd, 0xF0801074, val); 
			dev_info(&client->dev, "MIPI Time Clock Prepare: 0xF0801074, 0x%x\n", val);
			break;
		case 3:
			ret = PX6130_write32(sd, 0xF0801084, val); 
			dev_info(&client->dev, "MIPI Time Clock Zero: 0xF0801084, 0x%x\n", val);
			break;
		case 4:
			ret = PX6130_write32(sd, 0xF0801088, val); 
			dev_info(&client->dev, "MIPI Time Clock Trail: 0xF0801088, 0x%x\n", val);
			break;
		case 5:
			ret = PX6130_write32(sd, 0xF080108C, val); 
			dev_info(&client->dev, "MIPI Time Clock Pre: 0xF080108C, 0x%x\n", val);
			break;
		case 6:
			ret = PX6130_write32(sd, 0xF0801090, val); 
			dev_info(&client->dev, "MIPI Time Clock Post: 0xF0801090, 0x%x\n", val);
			break;
		case 7:
			ret = PX6130_write32(sd, 0xF080109C, val); 
			dev_info(&client->dev, "MIPI Time Clock HS Exit: 0xF080109C, 0x%x\n", val);
			break;
		case 30:
			dev_info(&client->dev, "None-continouse mode: 0x%x\n", 0x40);
			ret = PX6130_write32(sd, 0xF0801010, 0x40); // mipi none-continouse mode
			break;
		case 31:
			dev_info(&client->dev, "Continouse mode: 0x%x\n", 0x50);
			ret = PX6130_write32(sd, 0xF0801010, 0x50); // mipi continouse mode
			break;
		case 32:
			ret = PX6130_read32(sd, 0xF0801014, &device_val); // px6130 device ID
			dev_info(&client->dev, "px6130 Clock lane : 0x%x: 0x%x(0x04 : normal, 0x08 : converted)\n", 0xF0801014, device_val);  // expected 0x6130_0000
			break;
		case 40:
			ret = PX6130_read32(sd, 0xf0600004, &device_val); // px6130 device ID
			dev_info(&client->dev, "px6130 device ID(0x%x): 0x%x\n", 0xf0600004, device_val);  // expected 0x6130_0000
			break;
		case 41:
			ret = PX6130_read32(sd, 0xf0801000, &device_val);
			dev_info(&client->dev, "MIPI MIPI Controller Status(0x%x): 0x%x\n", 0xf0801000, device_val);
			break;
		case 42:
			ret = PX6130_write32(sd, 0xf0801000, 0xc1); //0100 0001
			dev_info(&client->dev, "MIPI MIPI Controller test mode enable(0x%x): 0x%x\n", 0xf0801000, device_val);
			break;
		case 43:
			ret = PX6130_write32(sd, 0xf0801000, 0xc0); //0100 0000
			dev_info(&client->dev, "MIPI MIPI Controller test mode disable(0x%x): 0x%x\n", 0xf0801000, device_val);
			break;
		default:
			break;
	}
	
	return ret;
}

static int PX6130_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct PX6130 *sensor = container_of(ctrl->handler,
					    struct PX6130, ctrls);
	struct v4l2_subdev *sd = &sensor->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	//dev_info(&client->dev, "dbg_info: %s, line: %d, id: %d\n", __func__, __LINE__, ctrl->id);
	
	switch (ctrl->id) {
	case V4L2_CID_PIXEL_RATE:
		/* Read-only, but we adjust it based on mode. */
		break;
	case V4L2_CID_HFLIP:
		PX6130_s_mirror(sd, PX6130_REG_MIRROR, ctrl->val, PX6130_MIRROR_H);
		break;
	case V4L2_CID_VFLIP:
		PX6130_s_mirror(sd, PX6130_REG_MIRROR, ctrl->val, PX6130_MIRROR_V);
		break;
	case V4L2_CID_HBLANK:
		break;
	case V4L2_CID_VBLANK:
		break;
	case V4L2_CID_LINK_FREQ:
		break;
	case V4L2_CID_PX6130_CUSTOM_SETTING:
		dev_info(&client->dev, "Custom control set to: %d\n", ctrl->val);
		if (ctrl->val != 0) {
			PX6130_s_custom(sd, ctrl->val);
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

static const struct v4l2_ctrl_ops PX6130_ctrl_ops = {
	.s_ctrl = PX6130_s_ctrl,
};

static const int PX6130_configure_regulators(struct device *dev, struct PX6130 *sensor)
{
	unsigned int i = 0;
	
	for (i = 0;i < PX6130_NUM_SUPPLIES; i++)
	{
		sensor->supplies[i].supply = PX6130_supply_names[i];
	}
	
	int nRet = devm_regulator_bulk_get(dev, PX6130_NUM_SUPPLIES, sensor->supplies);
	return nRet;
}

static const struct v4l2_ctrl_config PX6130_custom_ctrl_cfg = {
	.ops = &PX6130_ctrl_ops,
	.id = V4L2_CID_PX6130_CUSTOM_SETTING,
	.name = "PX6130_custom_set",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 128,
	.step = 1,
	.def = 0,
	.flags = 0,
};

static int PX6130_init_controls(struct PX6130 *sensor, struct device *dev)
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
	v4l2_ctrl_new_std(&sensor->ctrls, &PX6130_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, 0);
	
	v4l2_ctrl_new_std(&sensor->ctrls, &PX6130_ctrl_ops,
			  V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 0);
	
	v4l2_ctrl_new_std_menu(&sensor->ctrls, &PX6130_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL,
			       0, V4L2_EXPOSURE_MANUAL);
	
	exposure_max = sensor->mode->vts - 4;
	exposure_def = min(exposure_max, PX6130_EXPOSURE_DEFAULT);
	sensor->exposure = v4l2_ctrl_new_std(&sensor->ctrls, &PX6130_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     PX6130_EXPOSURE_MIN,
					     exposure_max, PX6130_EXPOSURE_STEP,
					     exposure_def);

	// min: 16 = 1.0x; max (10 bits); default: 32 = 2.0x.
	v4l2_ctrl_new_std(&sensor->ctrls, &PX6130_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 16, 1023, 1, 32);
	*/
	
	// By default, PIXEL_RATE is read only, but it does change per mode
	sensor->pixel_rate = v4l2_ctrl_new_std(&sensor->ctrls, &PX6130_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       sensor->mode->pixel_rate,
					       sensor->mode->pixel_rate, 1,
					       sensor->mode->pixel_rate);
	if (sensor->ctrls.error)
		dev_err(dev, "Pixel rate init failed (%d)\n", sensor->ctrls.error);

	sensor->link_freq =
		v4l2_ctrl_new_int_menu(&sensor->ctrls, &PX6130_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(PX6130_link_freq_menu) - 1, 0,
				       PX6130_link_freq_menu);
	if (sensor->ctrls.error)
		dev_err(dev, "Link freq init failed (%d)\n", sensor->ctrls.error);
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	
	hblank = sensor->mode->hts - sensor->mode->format.width;
	sensor->hblank = v4l2_ctrl_new_std(&sensor->ctrls, &PX6130_ctrl_ops,
					   V4L2_CID_HBLANK, hblank,
					   PX6130_HTS_MAX -
					   sensor->mode->format.width, 1,
					   hblank);
	if (sensor->ctrls.error)
		dev_err(dev, "H blank control init failed (%d)\n", sensor->ctrls.error);

	vblank = sensor->mode->vts - sensor->mode->format.height;
	sensor->vblank = v4l2_ctrl_new_std(&sensor->ctrls, &PX6130_ctrl_ops,
					   V4L2_CID_VBLANK, PX6130_VBLANK_MIN,
					   PX6130_VTS_MAX -
					   sensor->mode->format.height, 1,
					   vblank);
	if (sensor->ctrls.error)
		dev_err(dev, "V blank control init failed (%d)\n", sensor->ctrls.error);
					   
	/*
	v4l2_ctrl_new_std_menu_items(&sensor->ctrls, &PX6130_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(PX6130_test_pattern_menu) - 1,
				     0, 0, PX6130_test_pattern_menu);
	 */
	
	sensor->hflip = v4l2_ctrl_new_std(&sensor->ctrls, &PX6130_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (sensor->ctrls.error)
		dev_err(dev, "H flip control init failed (%d)\n", sensor->ctrls.error);
	if (sensor->hflip)
		sensor->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	sensor->vflip = v4l2_ctrl_new_std(&sensor->ctrls, &PX6130_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);	
	if (sensor->ctrls.error)
		dev_err(dev, "V flip control init failed (%d)\n", sensor->ctrls.error);
	if (sensor->vflip)
		sensor->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	sensor->custom_ctrl = v4l2_ctrl_new_custom(&sensor->ctrls,
		&PX6130_custom_ctrl_cfg,
		NULL);
	// sensor->custom_ctrl->flags |= V4L2_CTRL_FLAG_DISABLED;
	dev_info(dev, "sensor->custom_ctrl = 0x%x, error = %d\n",
		sensor->custom_ctrl, sensor->ctrls.error);
	
	v4l2_fwnode_device_parse(dev, &props);

	v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &PX6130_ctrl_ops,
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


static int PX6130_parse_dt(struct PX6130 *sensor, struct device_node *np, struct device *dev)
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
			(bus_cfg.link_frequencies[0] != PX6130_DEFAULT_LINK_FREQ)) {
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

static int PX6130_probe(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct device *dev = &client->dev;
	struct PX6130 *sensor;
	struct v4l2_subdev *sd;
	u32 xclk_freq;
	int ret;

	dev_info(dev, "PIXELPLUS PX6130 camera driver starts\n");

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_OF) && np) {
		ret = PX6130_parse_dt(sensor, np, dev);
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

	ret = PX6130_configure_regulators(dev, sensor);
	if (ret) {
		dev_err(dev, "Failed to get power regulators(%d)\n", ret);
		return ret;
	}

	mutex_init(&sensor->lock);

	sensor->mode = PX6130_DEFAULT_MODE;
	dev_info(dev, "PIXELPLUS PX6130 camera init controls\n");
	ret = PX6130_init_controls(sensor, dev);
	if (ret)
		goto mutex_destroy;

	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &PX6130_subdev_ops);
	sd->internal_ops = &PX6130_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR; //MEDIA_ENT_F_VID_IF_BRIDGE;
	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret < 0)
		goto ctrl_handler_free;

	// power from an external power supply, we don't need it
	/*
	ret = PX6130_power_on(dev);
	if (ret)
		goto entity_cleanup;
	

	ret = PX6130_detect(sd);
	if (ret < 0)
		goto power_off;
	*/

	ret = PX6130_power_on(dev);
	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret < 0)
	{
		dev_err(dev, "dbg_info: %s, line: %d, ret: %d\n", __func__, __LINE__, ret);
		goto power_off;
	}
	
	ret = PX6130_init_regs(sd);
	if (ret) {
		dev_err(dev, "Failed to initialize PX6130 registers(%d)\n", ret);
		//return ret;
	}

	// power from an external power supply, we don't need it
	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
	
	dev_info(dev, "PIXELPLUS PX6130 camera driver probed\n");

	TestRegs(dev, sd);

	return 0;

power_off:
	PX6130_power_off(dev);
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
	
	// ret = PX6130_write32(sd, 0xF0801010, 0x40); // mipi none-continouse mode

	ret = PX6130_read32(sd, nReg, &nVal); // check the sensor id
	dev_info(dev, "I2C Sensor ID success (0x%X, 0x%X)\n", nReg, nVal);

	nReg = 0xf0801000;
	ret = PX6130_read32(sd, nReg, &nVal); // check the sensor id
	dev_info(dev, "I2C MIPI enable success (0x%X, 0x%X)\n", nReg, nVal);
}

static void PX6130_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct PX6130 *sensor = to_sensor(sd);

	dev_info(&client->dev, "PIXELPLUS PX6130 driver is removed\n");

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);

	v4l2_ctrl_handler_free(&sensor->ctrls);
	v4l2_device_unregister_subdev(sd);

	pm_runtime_disable(&client->dev);

	mutex_destroy(&sensor->lock);
}

static const struct dev_pm_ops PX6130_pm_ops = {
	SET_RUNTIME_PM_OPS(PX6130_power_off, PX6130_power_on, NULL)
};

static const struct i2c_device_id PX6130_id[] = {
	{ "px6130", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, PX6130_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id PX6130_of_match[] = {
	{ .compatible = "pixelplus,px6130" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, PX6130_of_match);
#endif

static struct i2c_driver PX6130_driver = {
	.driver = {
		.of_match_table = of_match_ptr(PX6130_of_match),
		.name	= "px6130",
		.pm	= &PX6130_pm_ops,
	},
	.probe		= PX6130_probe,
	.remove		= PX6130_remove,
	.id_table	= PX6130_id,
};

module_i2c_driver(PX6130_driver);

MODULE_AUTHOR("Sanghyeon Kang <info@pixelplus.com>");
MODULE_DESCRIPTION("MIPI-CSI2 V4L2 driver for PIXELPLUS PX6130 sensors");
MODULE_LICENSE("GPL v2");

