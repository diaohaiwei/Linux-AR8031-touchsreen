#include <linux/mxc_v4l2.h>

struct sensor_priv {
#if 0	
	const struct ov5642_platform_data *platform_data;
	struct v4l2_int_device *v4l2_int_device;
#endif

	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	
};

struct sensor_data {
	
	u8 mclk_source;
	struct clk *sensor_clk;
	int csi;

	void (*io_init)(void);
};
