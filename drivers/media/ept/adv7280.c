/*
 * Copyright 2005-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file adv7280.c
 *
 * @brief Analog Device ADV7280 video decoder functions
 *
 * @ingroup Camera
 */

#define DEBUG 1

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "compatiable.h"

#include <linux/debugfs.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include "adv7280_reg.c"


#define ADV7280_VOLTAGE_ANALOG               1800000
#define ADV7280_VOLTAGE_DIGITAL_CORE         1800000
#define ADV7280_VOLTAGE_DIGITAL_IO           3300000
#define ADV7280_VOLTAGE_PLL                  1800000


#define ADV7280_IRQ_USED	0

static struct regulator *dvddio_regulator;
static struct regulator *dvdd_regulator;
static struct regulator *avdd_regulator;
static struct regulator *pvdd_regulator;
static int pwn_gpio;	/* CSI0_PWN (SD1_DATA0) => GPIO1_IO16 */
static int rst_gpio;	/* CSI0_RST_B(SD1_DATA1)=> GPIO1_IO17 */

static int adv7280_probe(struct i2c_client *adapter,
			 const struct i2c_device_id *id);
static int adv7280_detach(struct i2c_client *client);

static const struct i2c_device_id adv7280_id[] = {
	{"adv7280", 0},
	{},
};
static void adv7280_hard_reset(bool cvbs);

MODULE_DEVICE_TABLE(i2c, adv7280_id);

static struct i2c_driver adv7280_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "adv7280",
		   },
	.probe = adv7280_probe,
	.remove = adv7280_detach,
	.id_table = adv7280_id,
};

/*!
 * Maintains the information on the current state of the sensor.
 */
struct adv7280_sensor_data {
	struct sensor_data sen;
	struct sensor_priv priv;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_captureparm streamcap;
	
	u32 mclk;
	v4l2_std_id std_id;
#if ADV7280_IRQ_USED
	struct work_struct work;
	struct workqueue_struct *workqueue;
#endif
} adv7280_data;


/*! List of input video formats supported. The video formats is corresponding
 * with v4l2 id in video_fmt_t
 */
typedef enum {
	ADV7280_NTSC = 0,	/*!< Locked on (M) NTSC video signal. */
	ADV7280_PAL,		/*!< (B, G, H, I, N)PAL video signal. */
	ADV7280_NOT_LOCKED,	/*!< Not locked on a signal. */
} video_fmt_idx;

/*! Number of video standards supported (including 'not locked' signal). */
#define ADV7280_STD_MAX		(ADV7280_PAL + 1)

/*! Video format structure. */
typedef struct {
	int v4l2_id;		/*!< Video for linux ID. */
	char name[16];		/*!< Name (e.g., "NTSC", "PAL", etc.) */
	u16 raw_width;		/*!< Raw width. */
	u16 raw_height;		/*!< Raw height. */
	u16 active_width;	/*!< Active width. */
	u16 active_height;	/*!< Active height. */
	int frame_rate;		/*!< Frame rate. */
} video_fmt_t;

/*! Description of video formats supported.
 *
 *  PAL: raw=720x625, active=720x576.
 *  NTSC: raw=720x525, active=720x480.
 */
static video_fmt_t video_fmts[] = {
	{			/*! NTSC */
	 .v4l2_id = V4L2_STD_NTSC,
	 .name = "NTSC",
	 .raw_width = 720,	/* SENS_FRM_WIDTH */
	 .raw_height = 525,	/* SENS_FRM_HEIGHT */
	 .active_width = 720,	/* ACT_FRM_WIDTH plus 1 */
	 .active_height = 480,	/* ACT_FRM_WIDTH plus 1 */
	 .frame_rate = 30,
	 },
	{			/*! (B, G, H, I, N) PAL */
	 .v4l2_id = V4L2_STD_PAL,
	 .name = "PAL",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .frame_rate = 25,
	 },
	{			/*! Unlocked standard */
	 .v4l2_id = V4L2_STD_ALL,
	 .name = "Autodetect",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .frame_rate = 0,
	 },
};

/*!* Standard index of ADV7280. */
static video_fmt_idx video_idx = ADV7280_PAL;

/*! @brief This mutex is used to provide mutual exclusion.
 *
 *  Create a mutex that can be used to provide mutually exclusive
 *  read/write access to the globally accessible data structures
 *  and variables that were defined above.
 */
static DEFINE_MUTEX(mutex);

#define IF_NAME                    "adv7280"
#define ADV7280_INPUT_CTL              0x00	/* Input Control */
#define ADV7280_STATUS_1               0x10	/* Status #1 */
#define ADV7280_BRIGHTNESS             0x0a	/* Brightness */
#define ADV7280_IDENT                  0x11	/* IDENT */
#define ADV7280_VSYNC_FIELD_CTL_1      0x31	/* VSYNC Field Control #1 */
#define ADV7280_MANUAL_WIN_CTL         0x3d	/* Manual Window Control */
#define ADV7280_SD_SATURATION_CB       0xe3	/* SD Saturation Cb */
#define ADV7280_SD_SATURATION_CR       0xe4	/* SD Saturation Cr */
#define ADV7280_PWR_MNG                0x0f     /* Power Management */

/* supported controls */
/* This hasn't been fully implemented yet.
 * This is how it should work, though. */
static struct v4l2_queryctrl adv7280_qctrl[] = {
	{
	.id = V4L2_CID_BRIGHTNESS,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Brightness",
	.minimum = 0,		/* check this value */
	.maximum = 255,		/* check this value */
	.step = 1,		/* check this value */
	.default_value = 127,	/* check this value */
	.flags = 0,
	}, {
	.id = V4L2_CID_SATURATION,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Saturation",
	.minimum = 0,		/* check this value */
	.maximum = 255,		/* check this value */
	.step = 0x1,		/* check this value */
	.default_value = 127,	/* check this value */
	.flags = 0,
	}
};

static inline void adv7280_power_down(int enable)
{
	if(enable){
		gpio_set_value_cansleep(pwn_gpio, 0);
		gpio_set_value_cansleep(rst_gpio, 0);
		gpio_set_value(pwn_gpio, 1);
		usleep_range(50000, 50000);
		gpio_set_value(rst_gpio, 1);
		usleep_range(100000,100000);
	}else{
		gpio_set_value_cansleep(pwn_gpio, 0);
		gpio_set_value_cansleep(rst_gpio, 0);		
	}
#if 0
	gpio_set_value_cansleep(pwn_gpio, !enable);
#endif
	pr_debug("adv7280_power_down(no delay need test)\n");
	// marked by yusw
	//msleep(2);
}

static int adv7280_regulator_enable(struct device *dev)
{
	int ret = 0;

	dvddio_regulator = devm_regulator_get(dev, "DOVDD");

	if (!IS_ERR(dvddio_regulator)) {
		regulator_set_voltage(dvddio_regulator,
				      ADV7280_VOLTAGE_DIGITAL_IO,
				      ADV7280_VOLTAGE_DIGITAL_IO);
		ret = regulator_enable(dvddio_regulator);
		if (ret) {
			dev_err(dev, "set io voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set io voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get io voltage\n");
	}

	dvdd_regulator = devm_regulator_get(dev, "DVDD");
	if (!IS_ERR(dvdd_regulator)) {
		regulator_set_voltage(dvdd_regulator,
				      ADV7280_VOLTAGE_DIGITAL_CORE,
				      ADV7280_VOLTAGE_DIGITAL_CORE);
		ret = regulator_enable(dvdd_regulator);
		if (ret) {
			dev_err(dev, "set core voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set core voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get core voltage\n");
	}

	avdd_regulator = devm_regulator_get(dev, "AVDD");
	if (!IS_ERR(avdd_regulator)) {
		regulator_set_voltage(avdd_regulator,
				      ADV7280_VOLTAGE_ANALOG,
				      ADV7280_VOLTAGE_ANALOG);
		ret = regulator_enable(avdd_regulator);
		if (ret) {
			dev_err(dev, "set analog voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set analog voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get analog voltage\n");
	}

	pvdd_regulator = devm_regulator_get(dev, "PVDD");
	if (!IS_ERR(pvdd_regulator)) {
		regulator_set_voltage(pvdd_regulator,
				      ADV7280_VOLTAGE_PLL,
				      ADV7280_VOLTAGE_PLL);
		ret = regulator_enable(pvdd_regulator);
		if (ret) {
			dev_err(dev, "set pll voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set pll voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get pll voltage\n");
	}

	return ret;
}


/***********************************************************************
 * I2C transfert.
 ***********************************************************************/

/*! Read one register from a ADV7280 i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static inline int adv7280_read(u8 reg)
{
	int val;

	val = i2c_smbus_read_byte_data(adv7280_data.i2c_client, reg);
	if (val < 0) {
		dev_dbg(&adv7280_data.i2c_client->dev,
			"%s:read reg error: reg=%2x\n", __func__, reg);
		return -1;
	}
	return val;
}

/*! Write one register of a ADV7280 i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static int adv7280_write_reg(u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(adv7280_data.i2c_client, reg, val);
	if (ret < 0) {
		dev_dbg(&adv7280_data.i2c_client->dev,
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		return -1;
	}
	return 0;
}

/***********************************************************************
 * mxc_v4l2_capture interface.
 ***********************************************************************/

/*!
 * Return attributes of current video standard.
 * Since this device autodetects the current standard, this function also
 * sets the values that need to be changed if the standard changes.
 * There is no set std equivalent function.
 *
 *  @return		None.
 */
static void adv7280_get_std(v4l2_std_id *std)
{
	int status_1, standard, idx;
	bool locked;

	dev_dbg(&adv7280_data.i2c_client->dev, "In adv7280_get_std\n");

	status_1 = adv7280_read(ADV7280_STATUS_1);
	locked = status_1 & 0x1;
	standard = status_1 & 0x70;

	mutex_lock(&mutex);
	*std = V4L2_STD_ALL;
	idx = ADV7280_NOT_LOCKED;
	if (locked) {
		if (standard == 0x40) {
			*std = V4L2_STD_PAL;
			idx = ADV7280_PAL;
		} else if (standard == 0) {
			*std = V4L2_STD_NTSC;
			idx = ADV7280_NTSC;
		}
	}
	mutex_unlock(&mutex);

	/* This assumes autodetect which this device uses. */
	if (*std != adv7280_data.std_id) {
		video_idx = idx;
		adv7280_data.std_id = *std;
		adv7280_data.pix.width = video_fmts[video_idx].raw_width;
		adv7280_data.pix.height = video_fmts[video_idx].raw_height;
	}
}

/***********************************************************************
 * IOCTL Functions from v4l2_int_ioctl_desc.
 ***********************************************************************/

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	dev_dbg(&adv7280_data.i2c_client->dev, "adv7280:ioctl_g_ifparm\n");

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	/* Initialize structure to 0s then set any non-0 values. */
	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility. */
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.nobt_hs_inv = 1;
	p->u.bt656.bt_sync_correct = 1;

	/* ADV7280 has a dedicated clock so no clock settings needed. */

	return 0;
}

/*!
 * Sets the camera power.
 *
 * s  pointer to the camera device
 * on if 1, power is to be turned on.  0 means power is to be turned off
 *
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 * This is called on open, close, suspend and resume.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct adv7280_sensor_data *sensor = s->priv;

	dev_dbg(&adv7280_data.i2c_client->dev, "adv7280:ioctl_s_power\n");

	if (on && !sensor->priv.on) {
		dev_info(&adv7280_data.i2c_client->dev,"ADV7280_PWR_MNG (%02x) = 0x04 \n", ADV7280_PWR_MNG);
		if (adv7280_write_reg(ADV7280_PWR_MNG, 0x00) != 0)	//0x04
			return -EIO;
		adv7280_write_reg(0x0C, 0x36);
		adv7280_write_reg(0x02, 0x04);
		adv7280_write_reg(0x14, 0x12);
		adv7280_hard_reset(true);
		/*
		 * FIXME:Additional 400ms to wait the chip to be stable?
		 * This is a workaround for preview scrolling issue.
		 */
		msleep(400);
	} else if (!on && sensor->priv.on) {
		dev_info(&adv7280_data.i2c_client->dev,"ADV7280_PWR_MNG (%02x) = 0x24 \n", ADV7280_PWR_MNG);
		if (adv7280_write_reg(ADV7280_PWR_MNG, 0x20) != 0)	//0x24
			return -EIO;
	}

	sensor->priv.on = on;

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct adv7280_sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	dev_dbg(&adv7280_data.i2c_client->dev, "In adv7280:ioctl_g_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("ioctl_g_parm:type is unknown %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 *
 * This driver cannot change these settings.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	dev_dbg(&adv7280_data.i2c_client->dev, "In adv7280:ioctl_s_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct adv7280_sensor_data *sensor = s->priv;

	dev_dbg(&adv7280_data.i2c_client->dev, "adv7280:ioctl_g_fmt_cap\n");

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   Returning size of %dx%d\n",
			 sensor->pix.width, sensor->pix.height);
		f->fmt.pix = sensor->pix;
		break;

	case V4L2_BUF_TYPE_PRIVATE: {
		v4l2_std_id std;
		adv7280_get_std(&std);
		f->fmt.pix.pixelformat = (u32)std;
		}
		break;

	default:
		f->fmt.pix = sensor->pix;
		break;
	}

	return 0;
}

/*!
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
			   struct v4l2_queryctrl *qc)
{
	int i;

	dev_dbg(&adv7280_data.i2c_client->dev, "adv7280:ioctl_queryctrl\n");

	for (i = 0; i < ARRAY_SIZE(adv7280_qctrl); i++)
		if (qc->id && qc->id == adv7280_qctrl[i].id) {
			memcpy(qc, &(adv7280_qctrl[i]),
				sizeof(*qc));
			return 0;
		}

	return -EINVAL;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;
	int sat = 0;

	dev_dbg(&adv7280_data.i2c_client->dev, "In adv7280:ioctl_g_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_BRIGHTNESS\n");
		adv7280_data.priv.brightness = adv7280_read(ADV7280_BRIGHTNESS);
		vc->value = adv7280_data.priv.brightness;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_CONTRAST\n");
		vc->value = adv7280_data.priv.contrast;
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_SATURATION\n");
		sat = adv7280_read(ADV7280_SD_SATURATION_CB);
		adv7280_data.priv.saturation = sat;
		vc->value = adv7280_data.priv.saturation;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_HUE\n");
		vc->value = adv7280_data.priv.hue;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_RED_BALANCE\n");
		vc->value = adv7280_data.priv.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_BLUE_BALANCE\n");
		vc->value = adv7280_data.priv.blue;
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_EXPOSURE\n");
		vc->value = adv7280_data.priv.ae_mode;
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&adv7280_data.i2c_client->dev,"   Default case\n");
		vc->value = 0;
		ret = -EPERM;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	u8 tmp;

	dev_dbg(&adv7280_data.i2c_client->dev, "In adv7280:ioctl_s_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		tmp = vc->value;
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_BRIGHTNESS(%02x) = %02x\n", ADV7280_BRIGHTNESS, tmp);
		adv7280_write_reg(ADV7280_BRIGHTNESS, tmp);
		adv7280_data.priv.brightness = vc->value;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_CONTRAST\n");
		break;
	case V4L2_CID_SATURATION:
		tmp = vc->value;
		dev_dbg(&adv7280_data.i2c_client->dev,"   ADV7280_SD_SATURATION_CB(%02x) = %02x\n", ADV7280_SD_SATURATION_CB, tmp);
		dev_dbg(&adv7280_data.i2c_client->dev,"   ADV7280_SD_SATURATION_CR(%02x) = %02x\n", ADV7280_SD_SATURATION_CR, tmp);
		adv7280_write_reg(ADV7280_SD_SATURATION_CB, tmp);
		adv7280_write_reg(ADV7280_SD_SATURATION_CR, tmp);
		adv7280_data.priv.saturation = vc->value;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_HUE\n");
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_RED_BALANCE\n");
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_BLUE_BALANCE\n");
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_EXPOSURE\n");
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&adv7280_data.i2c_client->dev,"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&adv7280_data.i2c_client->dev,"   Default case\n");
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index >= 1)
		return -EINVAL;

	fsize->discrete.width = video_fmts[video_idx].active_width;
	fsize->discrete.height  = video_fmts[video_idx].active_height;

	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	video_fmt_t fmt;
	int i;

	if (fival->index != 0)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(video_fmts) - 1; i++) {
		fmt = video_fmts[i];
		if (fival->width  == fmt.active_width &&
		    fival->height == fmt.active_height) {
			fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
			fival->discrete.numerator = 1;
			fival->discrete.denominator = fmt.frame_rate;
			return 0;
		}
	}

	return -EINVAL;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
						"adv7280_decoder");
	((struct v4l2_dbg_chip_ident *)id)->ident = V4L2_IDENT_ADV7280;

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	dev_dbg(&adv7280_data.i2c_client->dev, "In adv7280:ioctl_init\n");
	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	dev_dbg(&adv7280_data.i2c_client->dev, "adv7280:ioctl_dev_init\n");
	return 0;
}

/*!
 * This structure defines all the ioctls for this module.
 */
static struct v4l2_int_ioctl_desc adv7280_ioctl_desc[] = {

	{vidioc_int_dev_init_num, 			(v4l2_int_ioctl_func*)ioctl_dev_init},

	/*!
	 * Delinitialise the dev. at slave detach.
	 * The complement of ioctl_dev_init.
	 */
/*	{vidioc_int_dev_exit_num, 			(v4l2_int_ioctl_func *)ioctl_dev_exit}, */

	{vidioc_int_s_power_num, 			(v4l2_int_ioctl_func*)ioctl_s_power},
	{vidioc_int_g_ifparm_num, 			(v4l2_int_ioctl_func*)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,		(v4l2_int_ioctl_func*)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, 				(v4l2_int_ioctl_func*)ioctl_reset}, */
	{vidioc_int_init_num, 				(v4l2_int_ioctl_func*)ioctl_init},

	/*!
	 * VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
	 */
/*	{vidioc_int_enum_fmt_cap_num,		(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap}, */

	/*!
	 * VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.
	 * This ioctl is used to negotiate the image capture size and
	 * pixel format without actually making it take effect.
	 */
/*	{vidioc_int_try_fmt_cap_num,		(v4l2_int_ioctl_func*)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, 			(v4l2_int_ioctl_func*)ioctl_g_fmt_cap},

	/*!
	 * If the requested format is supported, configures the HW to use that
	 * format, returns error code if format not supported or HW can't be
	 * correctly configured.
	 */
/*	{vidioc_int_s_fmt_cap_num, 			(v4l2_int_ioctl_func*)ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, 			(v4l2_int_ioctl_func*)ioctl_g_parm},
	{vidioc_int_s_parm_num, 			(v4l2_int_ioctl_func*)ioctl_s_parm},
	{vidioc_int_queryctrl_num, 			(v4l2_int_ioctl_func*)ioctl_queryctrl},
	{vidioc_int_g_ctrl_num, 			(v4l2_int_ioctl_func*)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, 			(v4l2_int_ioctl_func*)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,	(v4l2_int_ioctl_func*)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,(v4l2_int_ioctl_func*)ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num,		(v4l2_int_ioctl_func*)ioctl_g_chip_ident},
};

static struct v4l2_int_slave adv7280_slave = {
	.ioctls = adv7280_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(adv7280_ioctl_desc),
};

static struct v4l2_int_device adv7280_int_device = {
	.module = THIS_MODULE,
	.name = "adv7280",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &adv7280_slave,
	},
};


/***********************************************************************
 * I2C client and driver.
 ***********************************************************************/

static void adv7280_reset(void)
{
	int i;
	for(i=0; i < ARRAY_SIZE(user_sub_map); i++)
	{
		adv7280_write_reg(user_sub_map[i].addr, user_sub_map[i].val);
	}
}

static void adv7280_select_param(reg_unit* arr, int size){
	int i;
	for(i=0; i < size; i++)
	{
		adv7280_write_reg(arr[i].addr, arr[i].val);
	}	
}

/*! ADV7280 Reset function.
 *
 *  @return		None.
 */
static int current_channel = CVBS_AIN1;//SDT_COLORBAR;//

static void adv7280_hard_reset(bool cvbs)
{
	int param_size = 0;
	reg_unit* param = 0;
	dev_dbg(&adv7280_data.i2c_client->dev,
		"In adv7280:adv7280_hard_reset(cvbs = %d) \n", cvbs?1:0);

	if (cvbs) {
		/* Set CVBS input on AIN1 */
		adv7280_write_reg(ADV7280_INPUT_CTL, 0x00);
		dev_dbg(&adv7280_data.i2c_client->dev, "Set CVBS input on AIN1: ADV7280_INPUT_CTL(%02x) = 0x00\n", ADV7280_INPUT_CTL);
	} else {
		/*
		 * Set YPbPr input on AIN1,4,5 and normal
		 * operations(autodection of all stds).
		 */
		adv7280_write_reg(ADV7280_INPUT_CTL, 0x09);
		dev_dbg(&adv7280_data.i2c_client->dev, "Set YPbPr input on AIN1,4,5: ADV7280_INPUT_CTL(%02x) = 0x09\n", ADV7280_INPUT_CTL);
	}

	param = get_reg_param(current_channel, &param_size);
	if(param==0){
		adv7280_reset();
	}else{
		adv7280_select_param(param, param_size);
	}

}                               

#if ADV7280_IRQ_USED
static void adv7280_debug_work(struct work_struct *work)
{
	struct adv7280_sensor_data *sensor =
			container_of(work, struct sensor, work);
	struct i2c_client *client = sensor->i2c_client;
	
	printk("adv7280_debug_work \n");
#if 0
	int lux;

	/* Clear interrupt flag */
	isl29023_set_int_flag(client, 0);

	data->mode_before_interrupt = isl29023_get_mode(client);
	lux = isl29023_get_adc_value(client);

	/* To clear the interrpt status */
	isl29023_set_power_state(client, ISL29023_PD_MODE);
	isl29023_set_mode(client, data->mode_before_interrupt);
#endif

	msleep(100);

}
static irqreturn_t adv7280_irq_handler(int irq, void *handle)
{
	struct adv7280_sensor_data *sensor = handle;
#if 0
	int cmd_1;
	cmd_1 = i2c_smbus_read_byte_data(sensor->i2c_client, ISL29023_COMMAND1);
	if (!(cmd_1 & ISL29023_INT_FLAG_MASK))
		return IRQ_NONE;
#endif
	
	queue_work(sensor->workqueue, &sensor->work);
	return IRQ_HANDLED;
}

#endif

struct dentry * adv7280_root;

static int get_reg(void *data, u64 * value)
{
	u8 reg = *(u8*)data;
	int val;
	val = i2c_smbus_read_byte_data(adv7280_data.i2c_client, reg);
	if (val < 0) {
		dev_dbg(&adv7280_data.i2c_client->dev,
			"%s:read reg error: reg=0x%2x\n", __func__, reg);
		return -EAGAIN;
	}
	dev_info(&adv7280_data.i2c_client->dev,
		"%s data = 0x%02x, val= 0x%02x\n",__func__,reg,val);
	*value = val;
	return 0;
}

//static int adv7280_write_reg(u8 reg, u8 val)
static int set_reg(void *data, u64 value)
{
	u8 reg = *(u8*)data;
	u8 val = (u8)value;
	s32 ret;
	ret = i2c_smbus_write_byte_data(adv7280_data.i2c_client, reg, val);
	if (ret < 0) {
		dev_info(&adv7280_data.i2c_client->dev,
			"%s:write reg error:reg=0x%2x,val=0x%2x\n", __func__, reg, val);
		return -EAGAIN;
	}
	dev_info(&adv7280_data.i2c_client->dev, 
			"%s data = 0x%02x, val= 0x%02x\n",__func__, (int)reg, (int)val);
	return 0;

}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "0x%02llx\n");
 
#if 0
static int get_rt_status(void *data, u64 * val)
{
	int i = (int)data;
	int ret;
	/* global irq number is passed in via data */
	ret = pm_chg_get_rt_status(the_chip, i);
	printk("%s i=%d ret=%d \n",__func__,i,ret);
	*val = ret;
	return 0;
}
 
DEFINE_SIMPLE_ATTRIBUTE(rt_fops, get_rt_status, NULL, "%llu\n");
#endif

static int all_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

static ssize_t all_read(struct file *filp, char __user *buffer,
		size_t count, loff_t *ppos)
{
	char buff[4096];
	int i,value;
	char *ptr = buff;
	dev_info(&adv7280_data.i2c_client->dev, "all_read(count[%d], ppos[%d])\n", (int)count, (int) *ppos);

	for(i=0; i<=255; i++){
		value = adv7280_read(i);
		sprintf(ptr, "reg[0x%02x]=0x%02x\n", i, value);
		ptr += strlen(ptr);	
		if(ptr > (buff + 4000)){
			dev_info(&adv7280_data.i2c_client->dev, "all_read(not enough space)\n");
		}
	}
	
	dev_info(&adv7280_data.i2c_client->dev, "all_read(size[%d])\n", (int)(ptr-buff));
	
	if (*ppos >= 4096)
		return 0;
	if (*ppos + count > 4096)
		count = 4096 - *ppos;
	
	if (copy_to_user(buffer, buff + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

static ssize_t all_write(struct file *filp, const char __user *buffer,
		size_t count, loff_t *ppos)
{
	return -EFAULT;
}

struct file_operations all_fops = {
	.owner = THIS_MODULE,
	.open = all_open,
	.read = all_read,
	.write = all_write,
};



static u8 adv7280_regmap[256];

static void debugfs_create_adv7280(void){
	int regno;
	char buf[16];
	struct dentry *f;
	adv7280_root = debugfs_create_dir("adv7280", NULL);
	
	for(regno=0; regno<=255; regno++){
		adv7280_regmap[regno] = (u8) regno;
		sprintf(buf, "%02x", regno);
		buf[15] = 0;
		f = debugfs_create_file(buf, 0644, adv7280_root, &adv7280_regmap[regno], &reg_fops);
		if (!f){
			goto Fail;
		}		
	}

	f = debugfs_create_file("adv7280_regs", 0644, NULL, NULL, &all_fops);
	if (!f){
		goto Fail;
	}		
		
	return;
Fail:
	debugfs_remove_recursive(adv7280_root);
	adv7280_root = NULL;
	
}


static ssize_t show_channel(struct device *dev, struct device_attribute *attr,
                         char *buf)
{
        struct i2c_client *client;
        client = to_i2c_client(dev);
		
        return scnprintf(buf, PAGE_SIZE, "%u\n", current_channel);
}

static ssize_t store_channel(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{

        struct i2c_client *client;
        int ret;
		unsigned long channel;
        client = to_i2c_client(dev);

		ret = kstrtoul(buf, 0, &channel);
		if (ret < 0){
			dev_err(&client->dev, "bad format for channel\n");
			return ret;
		}
		if (channel > SDT_COLORBAR){
			dev_err(&client->dev, "invalid channel\n");
			return -1;
		}
		
		current_channel = channel;

        return count;
}

static DEVICE_ATTR(channel, S_IRUGO | S_IWUSR, show_channel, store_channel);

static struct attribute *adv7280_attrs[] = {
        &dev_attr_channel.attr,
        NULL,
};

static struct attribute_group adv7280_group = {
        .name = "adv7280",
        .attrs = adv7280_attrs,
};

/*! ADV7280 I2C attach function.
 *
 *  @param *adapter	struct i2c_adapter *.
 *
 *  @return		Error code indicating success or failure.
 */

/*!
 * ADV7280 I2C probe function.
 * Function set in i2c_driver struct.
 * Called by insmod.
 *
 *  @param *adapter	I2C adapter descriptor.
 *
 *  @return		Error code indicating success or failure.
 */
static int adv7280_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int rev_id;
	int ret = 0;
	u32 cvbs = true;
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;

#if ADV7280_IRQ_USED
	struct irq_data *irq_data = irq_get_irq_data(client->irq);
	u32 irq_flag;
#endif

	pr_debug("adv7280_probe: sensor data is at %p\n", &adv7280_data);
	/* ov5640 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}


	/* request power down pin */
	pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(pwn_gpio)) {
		dev_err(dev, "no sensor pwdn pin available\n");
		return -ENODEV;
	}
	ret = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_LOW,	"adv7280_pwdn pin");
	if (ret < 0) {
		dev_err(dev, "no power pin available!\n");
		return ret;
	}

	/* request power down pin */
	rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(rst_gpio)) {
		dev_err(dev, "no sensor reset pin available\n");
		return -ENODEV;
	}
	ret = devm_gpio_request_one(dev, rst_gpio, GPIOF_OUT_INIT_LOW,	"adv7280_reset pin");
	if (ret < 0) {
		dev_err(dev, "no reset pin available!\n");
		return ret;
	}

	adv7280_regulator_enable(dev);

	adv7280_power_down(1);

	/* Set initial values for the sensor struct. */
	memset(&adv7280_data, 0, sizeof(adv7280_data));
	adv7280_data.i2c_client = client;
	adv7280_data.streamcap.timeperframe.denominator = 30;
	adv7280_data.streamcap.timeperframe.numerator = 1;
	adv7280_data.std_id = V4L2_STD_ALL;
	video_idx = ADV7280_NOT_LOCKED;
	adv7280_data.pix.width = video_fmts[video_idx].raw_width;
	adv7280_data.pix.height = video_fmts[video_idx].raw_height;
	adv7280_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;  /* YUV422 */
	adv7280_data.pix.priv = 1;  /* 1 is used to indicate TV in */
	adv7280_data.priv.on = true;

	adv7280_data.sen.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(adv7280_data.sen.sensor_clk)) {
		dev_err(dev, "get mclk failed\n");
		return PTR_ERR(adv7280_data.sen.sensor_clk);
	}

	ret = of_property_read_u32(dev->of_node, "mclk",
					&adv7280_data.mclk);
	if (ret) {
		dev_err(dev, "mclk frequency is invalid\n");
		return ret;
	}

	ret = of_property_read_u32(
		dev->of_node, "mclk_source",
		(u32 *) &(adv7280_data.sen.mclk_source));
	if (ret) {
		dev_err(dev, "mclk_source invalid\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "csi_id",
					&(adv7280_data.sen.csi));
	if (ret) {
		dev_err(dev, "csi_id invalid\n");
		return ret;
	}

	clk_prepare_enable(adv7280_data.sen.sensor_clk);

	dev_dbg(&adv7280_data.i2c_client->dev,
		"%s:adv7280 probe i2c address is 0x%02X\n",
		__func__, adv7280_data.i2c_client->addr);

	/*! Read the revision ID of the tvin chip */
	rev_id = adv7280_read(ADV7280_IDENT);
	dev_dbg(dev,
		"%s:Analog Device adv7%2X0 detected!\n", __func__,
		rev_id);

	ret = of_property_read_u32(dev->of_node, "cvbs", &(cvbs));
	if (ret) {
		dev_err(dev, "cvbs setting is not found\n");
		cvbs = true;
	}

	/*! ADV7280 initialization. */
	pr_debug("adv7280_probe (0)\n");
	adv7280_hard_reset(cvbs);

	pr_debug("adv7280_probe (1)\n");
	pr_debug("   type is %d (expect %d)\n",
		 adv7280_int_device.type, v4l2_int_type_slave);
	pr_debug("   num ioctls is %d\n",
		 adv7280_int_device.u.slave->num_ioctls);

#if ADV7280_IRQ_USED
	adv7280_data.workqueue = create_singlethread_workqueue("debug_adv7280");
	INIT_WORK(&adv7280_data.work, adv7280_debug_work);
	if (adv7280_data.workqueue == NULL) {
		dev_err(&client->dev, "couldn't create workqueue\n");
		ret = -ENOMEM;
		goto exit_error;
	}	
	
	irq_flag = irqd_get_trigger_type(irq_data);
	irq_flag |= IRQF_ONESHOT;
	ret = request_threaded_irq(client->irq, NULL, adv7280_irq_handler,
			  irq_flag, client->dev.driver->name, &adv7280_data);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register irq %d!\n",
			client->irq);
		goto exit_free_workqueue;
	}
	
#endif

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the adv7280_data structure here.*/
	pr_debug("adv7280_probe (2)\n");
	adv7280_int_device.priv = &adv7280_data;
	ret = v4l2_int_device_register(&adv7280_int_device);

	pr_debug("adv7280_probe (3)\n");
	clk_disable_unprepare(adv7280_data.sen.sensor_clk);
	
	debugfs_create_adv7280();
	sysfs_create_group(&client->dev.kobj, &adv7280_group);

	return ret;
#if ADV7280_IRQ_USED
exit_free_workqueue:
	destroy_workqueue(adv7280_data.workqueue);
exit_error:
	return ret;
#endif
}

/*!
 * ADV7280 I2C detach function.
 * Called on rmmod.
 *
 *  @param *client	struct i2c_client*.
 *
 *  @return		Error code indicating success or failure.
 */
static int adv7280_detach(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	
	dev_dbg(&adv7280_data.i2c_client->dev,
		"%s:Removing %s video decoder @ 0x%02X from adapter %s\n",
		__func__, IF_NAME, client->addr << 1, client->adapter->name);

	sysfs_remove_group(&client->dev.kobj, &adv7280_group);
	if(adv7280_root){
		debugfs_remove_recursive(adv7280_root);
		adv7280_root = NULL;	
	}

	/* Power down via i2c */
	adv7280_write_reg(ADV7280_PWR_MNG, 0x24);

	adv7280_power_down(0);
	
	devm_gpio_free(dev, pwn_gpio);
	devm_gpio_free(dev, rst_gpio);

#if ADV7280_IRQ_USED
	destroy_workqueue(adv7280_data.workqueue);
	free_irq(client->irq, &adv7280_data);
#endif
	if (dvddio_regulator)
		regulator_disable(dvddio_regulator);

	if (dvdd_regulator)
		regulator_disable(dvdd_regulator);

	if (avdd_regulator)
		regulator_disable(avdd_regulator);

	if (pvdd_regulator)
		regulator_disable(pvdd_regulator);

	v4l2_int_device_unregister(&adv7280_int_device);

	return 0;
}

/*!
 * ADV7280 init function.
 * Called on insmod.
 *
 * @return    Error code indicating success or failure.
 */
static __init int adv7280_init(void)
{
	u8 err = 0;

	pr_debug("In adv7280_init\n");

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&adv7280_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
			__func__, err);

	return err;
}

/*!
 * ADV7280 cleanup function.
 * Called on rmmod.
 *
 * @return   Error code indicating success or failure.
 */
static void __exit adv7280_clean(void)
{
	dev_dbg(&adv7280_data.i2c_client->dev, "In adv7280_clean\n");
	i2c_del_driver(&adv7280_i2c_driver);
}

module_init(adv7280_init);
module_exit(adv7280_clean);

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Anolog Device ADV7280 video decoder driver");
MODULE_LICENSE("GPL");
