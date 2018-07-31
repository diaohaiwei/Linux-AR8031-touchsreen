/*
 * Copyright 2004-2014 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup MXC_V4L2_CAPTURE MXC V4L2 Video Capture Driver
 */
/*!
 * @file mxc_v4l2_capture.h
 *
 * @brief mxc V4L2 capture device API  Header file
 *
 * It include all the defines for frame operations, also three structure defines
 * use case ops structure, common v4l2 driver structure and frame structure.
 *
 * @ingroup MXC_V4L2_CAPTURE
 */
#ifndef __MXC_V4L2_CAPTURE_H__
#define __MXC_V4L2_CAPTURE_H__

#include "mxc_v4l2_priv.h"

struct sensor_data {
#if defined(MXC_EPT_XXXX_)
	//u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int csi;
	
	void (*io_init)(void);
#endif
};

void set_mclk_rate(uint32_t *p_mclk_freq, uint32_t csi);
#endif				/* __MXC_V4L2_CAPTURE_H__ */
