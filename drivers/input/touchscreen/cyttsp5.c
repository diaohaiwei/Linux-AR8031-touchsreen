/*
 * Parade TrueTouch(TM) Standard Product V5 Module.
 * For use with Parade touchscreen controllers.
 *
 * Copyright (C) 2015 Parade Technologies
 * Copyright (C) 2012-2015 Cypress Semiconductor
 * Copyright (C) 2017 Free Electrons
 *
 * Author: Mylène Josserand <mylene.josserand@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
//#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include "touchscreen.h"
#include <linux/of_gpio.h>


#define CYTTSP5_NAME				"cyttsp5"
#define CY_I2C_DATA_SIZE			(2 * 256)
#define HID_VERSION				0x0100
#define CY_MAX_INPUT				512
#define CYTTSP5_PREALLOCATED_CMD_BUFFER	32
#define CY_BITS_PER_BTN			1
#define CY_NUM_BTN_EVENT_ID			((1 << CY_BITS_PER_BTN) - 1)

#define MAX_AREA				255
#define HID_OUTPUT_BL_SOP			0x1
#define HID_OUTPUT_BL_EOP			0x17
#define HID_OUTPUT_BL_LAUNCH_APP		0x3B
#define HID_OUTPUT_BL_LAUNCH_APP_SIZE		11
#define HID_OUTPUT_GET_SYSINFO			0x2
#define HID_OUTPUT_GET_SYSINFO_SIZE		5

#define HID_DESC_REG				0x1
#define HID_OUTPUT_REG				0x4

#define REPORT_ID_TOUCH			0x1
#define REPORT_ID_BTN				0x3
#define REPORT_SIZE_5				5
#define REPORT_SIZE_8				8
#define REPORT_SIZE_16				16

/* Touch reports offsets */
/* Header offsets */
#define TOUCH_REPORT_DESC_HDR_CONTACTCOUNT	16
/* Record offsets */
#define TOUCH_REPORT_DESC_CONTACTID		8
#define TOUCH_REPORT_DESC_X			16
#define TOUCH_REPORT_DESC_Y			32
#define TOUCH_REPORT_DESC_P			48
#define TOUCH_REPORT_DESC_MAJ			56
#define TOUCH_REPORT_DESC_MIN			64

/* HID */
#define HID_TOUCH_REPORT_ID			0x1
#define HID_BTN_REPORT_ID			0x3
#define HID_APP_RESPONSE_REPORT_ID		0x1F
#define HID_APP_OUTPUT_REPORT_ID		0x2F
#define HID_BL_RESPONSE_REPORT_ID		0x30
#define HID_BL_OUTPUT_REPORT_ID		0x40

#define HID_OUTPUT_RESPONSE_REPORT_OFFSET	2
#define HID_OUTPUT_RESPONSE_CMD_OFFSET		4
#define HID_OUTPUT_RESPONSE_CMD_MASK		0x7F

#define HID_SYSINFO_SENSING_OFFSET		33
#define HID_SYSINFO_BTN_OFFSET			48
#define HID_SYSINFO_BTN_MASK			0xFF
#define HID_SYSINFO_MAX_BTN			8

/*  Timeout in ms */
#define CY_HID_OUTPUT_TIMEOUT			200
#define CY_HID_OUTPUT_GET_SYSINFO_TIMEOUT	3000
#define CY_HID_GET_HID_DESCRIPTOR_TIMEOUT	4000

/* maximum number of concurrent tracks */
#define TOUCH_REPORT_SIZE			10
#define TOUCH_INPUT_HEADER_SIZE		7
#define BTN_REPORT_SIZE			9
#define BTN_INPUT_HEADER_SIZE			5

/* All usage pages for Touch Report */
#define TOUCH_REPORT_USAGE_PG_X		0x00010030
#define TOUCH_REPORT_USAGE_PG_Y		0x00010031
#define TOUCH_REPORT_USAGE_PG_P		0x000D0030
#define TOUCH_REPORT_USAGE_PG_CONTACTID	0x000D0051
#define TOUCH_REPORT_USAGE_PG_CONTACTCOUNT	0x000D0054
#define TOUCH_REPORT_USAGE_PG_MAJ		0xFF010062
#define TOUCH_REPORT_USAGE_PG_MIN		0xFF010063
#define TOUCH_COL_USAGE_PG			0x000D0022

/* helpers */
#define IS_TMO(t)				((t) == 0)
#define HI_BYTE(x)				(u8)(((x) >> 8) & 0xFF)
#define LOW_BYTE(x)				(u8)((x) & 0xFF)

/* System Information interface definitions */
struct cyttsp5_sensing_conf_data_dev {
	u8 electrodes_x;
	u8 electrodes_y;
	__le16 len_x;
	__le16 len_y;
	__le16 res_x;
	__le16 res_y;
	__le16 max_z;
	u8 origin_x;
	u8 origin_y;
	u8 btn;
	u8 scan_mode;
	u8 max_num_of_tch_per_refresh_cycle;
} __packed;

struct cyttsp5_sensing_conf_data {
	u16 res_x;
	u16 res_y;
	u16 max_z;
	u16 len_x;
	u16 len_y;
	u8 origin_x;
	u8 origin_y;
	u8 max_tch;
};

enum cyttsp5_tch_abs {	/* for ordering within the extracted touch data array */
	CY_TCH_X,	/* X */
	CY_TCH_Y,	/* Y */
	CY_TCH_P,	/* P (Z) */
	CY_TCH_T,	/* TOUCH ID */
	CY_TCH_MAJ,	/* TOUCH_MAJOR */
	CY_TCH_MIN,	/* TOUCH_MINOR */
	CY_TCH_NUM_ABS,
};

struct cyttsp5_tch_abs_params {
	size_t ofs;	/* abs byte offset */
	size_t size;	/* size in bits */
	size_t min;	/* min value */
	size_t max;	/* max value */
	size_t bofs;	/* bit offset */
};

struct cyttsp5_touch {
	int hdr;
	int abs[CY_TCH_NUM_ABS];
};

struct cyttsp5_btn {
	bool enabled;
	int key_code;
};

struct cyttsp5_sysinfo {
	struct cyttsp5_sensing_conf_data sensing_conf_data;
	int num_btns;
	struct cyttsp5_btn *btn;
	struct cyttsp5_tch_abs_params tch_hdr;
	struct cyttsp5_tch_abs_params tch_abs[CY_TCH_NUM_ABS];
	u8 *xy_mode;
	u8 *xy_data;
};

struct cyttsp5_hid_desc {
	__le16 hid_desc_len;
	u8 packet_id;
	u8 reserved_byte;
	__le16 bcd_version;
	__le16 report_desc_len;
	__le16 report_desc_register;
	__le16 input_register;
	__le16 max_input_len;
	__le16 output_register;
	__le16 max_output_len;
	__le16 command_register;
	__le16 data_register;
	__le16 vendor_id;
	__le16 product_id;
	__le16 version_id;
	u8 reserved[4];
} __packed;

struct cyttsp5 {
	struct device *dev;
	struct mutex system_lock;
	struct mutex btn_lock;
	struct mutex mt_lock;
	wait_queue_head_t wait_q;
	int irq;
	struct cyttsp5_sysinfo sysinfo;
	int hid_cmd_state;
	struct cyttsp5_hid_desc hid_desc;
	u8 cmd_buf[CYTTSP5_PREALLOCATED_CMD_BUFFER];
	u8 input_buf[CY_MAX_INPUT];
	u8 response_buf[CY_MAX_INPUT];
#if 0	//yusw
	struct gpio_desc *reset_gpio;
#else
	int reset_gpio;
#endif
	struct input_dev *input;
	char phys[NAME_MAX];
	int num_prv_rec;
	struct regmap *regmap;
};

static int cyttsp5_read(struct cyttsp5 *ts, u8 *buf, u32 max)
{
	int rc;
	u32 size;

	if (!buf)
		return -EINVAL;

	/* Read the frame to retrieve the size */
	rc = regmap_bulk_read(ts->regmap, 0, buf, 2);
	if (rc < 0)
		return rc;

	size = get_unaligned_le16(&buf[0]);
	if (!size || size == 2)
		return 0;

	if (size > max)
		return -EINVAL;

	/* Get the real value */
	return regmap_bulk_read(ts->regmap, 0, buf, size);
}

static int cyttsp5_write(struct cyttsp5 *ts, unsigned int reg, u8 *data,
			 size_t size)
{
	u8 cmd[size + 1];

	/* High bytes of register address needed as first byte of cmd */
	cmd[0] = HI_BYTE(reg);
	/* Copy the rest of the data */
	memcpy(&cmd[1], data, size);

	return regmap_bulk_write(ts->regmap, LOW_BYTE(reg), cmd, size + 1);
}

static void cyttsp5_final_sync(struct input_dev *input, int max_slots,
			       unsigned long *ids)
{
	int t;

	for (t = 0; t < max_slots; t++) {
		if (test_bit(t, ids))
			continue;
		input_mt_slot(input, t);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
	}

	input_sync(input);
}

static void cyttsp5_report_slot_liftoff(struct cyttsp5 *ts, int max_slots)
{
	int t;

	if (ts->num_prv_rec == 0)
		return;

	for (t = 0; t < max_slots; t++) {
		input_mt_slot(ts->input, t);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	}
}

static void cyttsp5_mt_lift_all(struct cyttsp5 *ts)
{
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	int max = si->tch_abs[CY_TCH_T].max;

	if (ts->num_prv_rec != 0) {
		cyttsp5_report_slot_liftoff(ts, max);
		input_sync(ts->input);
		ts->num_prv_rec = 0;
	}
}

static void cyttsp5_get_touch_axis(int *axis, int size, int max, u8 *xy_data,
				   int bofs)
{
	int nbyte;
	int next;

	for (nbyte = 0, *axis = 0, next = 0; nbyte < size; nbyte++) {
		*axis = *axis + ((xy_data[next] >> bofs) << (nbyte * 8));
		next++;
	}

	*axis &= max - 1;
}

static void cyttsp5_get_touch_record(struct cyttsp5 *ts,
				     struct cyttsp5_touch *touch, u8 *xy_data)
{
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	enum cyttsp5_tch_abs abs;

	for (abs = CY_TCH_X; abs < CY_TCH_NUM_ABS; abs++) {
		cyttsp5_get_touch_axis(&touch->abs[abs],
				       si->tch_abs[abs].size,
				       si->tch_abs[abs].max,
				       xy_data + si->tch_abs[abs].ofs,
				       si->tch_abs[abs].bofs);
	}
}

static void cyttsp5_mt_process_touch(struct cyttsp5 *ts,
				     struct cyttsp5_touch *touch)
{
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	int tmp;

	tmp = touch->abs[CY_TCH_X];
	touch->abs[CY_TCH_X] = touch->abs[CY_TCH_Y];
	touch->abs[CY_TCH_Y] = tmp;

	/* Convert MAJOR/MINOR from mm to resolution */
	tmp = touch->abs[CY_TCH_MAJ] * 100 * si->sensing_conf_data.res_x;
	touch->abs[CY_TCH_MAJ] = tmp / si->sensing_conf_data.len_x;
	tmp = touch->abs[CY_TCH_MIN] * 100 * si->sensing_conf_data.res_x;
	touch->abs[CY_TCH_MIN] = tmp / si->sensing_conf_data.len_x;
}

static void cyttsp5_get_mt_touches(struct cyttsp5 *ts,
				   struct cyttsp5_touch *tch, int num_cur_tch)
{
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	int i, t = 0;
	DECLARE_BITMAP(ids, si->tch_abs[CY_TCH_T].max);
	u8 *tch_addr;

	bitmap_zero(ids, si->tch_abs[CY_TCH_T].max);
	memset(tch->abs, 0, sizeof(tch->abs));

	for (i = 0; i < num_cur_tch; i++) {
		tch_addr = si->xy_data + (i * TOUCH_REPORT_SIZE);
		cyttsp5_get_touch_record(ts, tch, tch_addr);

		/* Process touch */
		cyttsp5_mt_process_touch(ts, tch);

		t = tch->abs[CY_TCH_T];
		input_mt_slot(ts->input, t);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		__set_bit(t, ids);

		/* position and pressure fields */
		input_report_abs(ts->input, ABS_MT_POSITION_X, tch->abs[CY_TCH_X]);
		input_report_abs(ts->input, ABS_MT_POSITION_Y, tch->abs[CY_TCH_Y]);
		input_report_abs(ts->input, ABS_MT_PRESSURE, tch->abs[CY_TCH_P]);

		/* Get the extended touch fields */
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, tch->abs[CY_TCH_MAJ]);
		input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, tch->abs[CY_TCH_MIN]);
	}

	cyttsp5_final_sync(ts->input, si->tch_abs[CY_TCH_T].max, ids);

	ts->num_prv_rec = num_cur_tch;
}

/* read xy_data for all current touches */
static int cyttsp5_xy_worker(struct cyttsp5 *ts)
{
	struct device *dev = ts->dev;
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	int max_tch = si->sensing_conf_data.max_tch;
	struct cyttsp5_touch tch;
	u8 num_cur_tch;

	cyttsp5_get_touch_axis(&tch.hdr, si->tch_hdr.size,
			       si->tch_hdr.max,
			       si->xy_mode + 3 + si->tch_hdr.ofs,
			       si->tch_hdr.bofs);

	num_cur_tch = tch.hdr;
	if (num_cur_tch > max_tch) {
		dev_err(dev, "%s: Num touch err detected (n=%d)\n",
			__func__, num_cur_tch);
		num_cur_tch = max_tch;
	}

	if (num_cur_tch == 0 && ts->num_prv_rec == 0)
		return 0;

	/* extract xy_data for all currently reported touches */
	if (num_cur_tch)
		cyttsp5_get_mt_touches(ts, &tch, num_cur_tch);
	else
		cyttsp5_mt_lift_all(ts);

	return 0;
}

static int cyttsp5_mt_attention(struct device *dev)
{
	struct cyttsp5 *ts = dev_get_drvdata(dev);
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	int rc;

	if (si->xy_mode[2] != HID_TOUCH_REPORT_ID)
		return 0;

	/* core handles handshake */
	mutex_lock(&ts->mt_lock);
	rc = cyttsp5_xy_worker(ts);
	mutex_unlock(&ts->mt_lock);
	if (rc < 0)
		dev_err(dev, "%s: xy_worker error r=%d\n", __func__, rc);

	return rc;
}

static int cyttsp5_setup_input_device(struct device *dev)
{
	struct cyttsp5 *ts = dev_get_drvdata(dev);
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	int max_x, max_y, max_p;
	int max_x_tmp, max_y_tmp;
	int rc;

	__set_bit(EV_ABS, ts->input->evbit);
	__set_bit(EV_REL, ts->input->evbit);
	__set_bit(EV_KEY, ts->input->evbit);

	max_x_tmp = si->sensing_conf_data.res_x;
	max_y_tmp = si->sensing_conf_data.res_y;
	max_x = max_y_tmp - 1;
	max_y = max_x_tmp - 1;
	max_p = si->sensing_conf_data.max_z;

	input_mt_init_slots(ts->input, si->tch_abs[CY_TCH_T].max, 0);

	__set_bit(ABS_MT_POSITION_X, ts->input->absbit);
	__set_bit(ABS_MT_POSITION_Y, ts->input->absbit);
	__set_bit(ABS_MT_PRESSURE, ts->input->absbit);

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, max_p, 0, 0);

	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, MAX_AREA, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MINOR, 0, MAX_AREA, 0, 0);

	rc = input_register_device(ts->input);
	if (rc < 0)
		dev_err(dev, "%s: Error, failed register input device r=%d\n",
			__func__, rc);

	return rc;
}

static int cyttsp5_parse_dt_key_code(struct device *dev)
{
	struct cyttsp5 *ts = dev_get_drvdata(dev);
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	struct device_node *np, *pp;
	int rc, i;

	np = dev->of_node;
	if (!np)
		return -EINVAL;

	if (si->num_btns) {
		si->btn = devm_kzalloc(dev,
				       si->num_btns * sizeof(struct cyttsp5_btn),
				       GFP_KERNEL);
		if (!si->btn)
			return -ENOMEM;

		/* Initialized the button to RESERVED */
		for (i = 0; i < si->num_btns; i++) {
			struct cyttsp5_btn *btns = &si->btn[i];
			btns->key_code = KEY_RESERVED;
			btns->enabled = true;
		}
	}

	i = 0;
	for_each_child_of_node(np, pp) {
		struct cyttsp5_btn *btns = &si->btn[i];

		rc = of_property_read_u32(pp, "linux,code", &btns->key_code);
		if (rc) {
			dev_err(dev, "%s: Inval linux,code prop\n", pp->name);
			return -EINVAL;
		}
		btns->enabled = true;

		i++;
	}

	return i;
}

static int cyttsp5_btn_attention(struct device *dev)
{
	struct cyttsp5 *ts = dev_get_drvdata(dev);
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	int cur_btn;
	int cur_btn_state;

	if (si->xy_mode[2] != HID_BTN_REPORT_ID)
		return 0;

	/* core handles handshake */
	mutex_lock(&ts->btn_lock);

	/* extract button press/release touch information */
	if (si->num_btns > 0) {
		for (cur_btn = 0; cur_btn < si->num_btns; cur_btn++) {
			/* Get current button state */
			cur_btn_state = (si->xy_data[0] >> (cur_btn * CY_BITS_PER_BTN))
				& CY_NUM_BTN_EVENT_ID;

			if (!si->btn[cur_btn].enabled)
				continue;

			input_report_key(ts->input, si->btn[cur_btn].key_code,
					 cur_btn_state);
			input_sync(ts->input);
		}
	}

	mutex_unlock(&ts->btn_lock);

	return 0;
}

static const u16 crc_table[16] = {
	0x0000, 0x1021, 0x2042, 0x3063,
	0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b,
	0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
};

static u16 cyttsp5_compute_crc(u8 *buf, u32 size)
{
	u16 remainder = 0xFFFF;
	u16 xor_mask = 0x0000;
	u32 index;
	u32 byte_value;
	u32 table_index;
	u32 crc_bit_width = sizeof(u16) * 8;

	/* Divide the message by polynomial, via the table. */
	for (index = 0; index < size; index++) {
		byte_value = buf[index];
		table_index = ((byte_value >> 4) & 0x0F)
			^ (remainder >> (crc_bit_width - 4));
		remainder = crc_table[table_index] ^ (remainder << 4);
		table_index = (byte_value & 0x0F)
			^ (remainder >> (crc_bit_width - 4));
		remainder = crc_table[table_index] ^ (remainder << 4);
	}

	/* Perform the final remainder CRC. */
	return remainder ^ xor_mask;
}

static int cyttsp5_validate_cmd_response(struct cyttsp5 *ts, u8 code)
{
	u16 size, crc;
	u8 status, offset;
	int command_code;

	size = get_unaligned_le16(&ts->response_buf[0]);

	if (!size)
		return 0;

	offset = ts->response_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET];

	if (offset == HID_BL_RESPONSE_REPORT_ID) {
		if (ts->response_buf[4] != HID_OUTPUT_BL_SOP) {
			dev_err(ts->dev, "%s: HID output response, wrong SOP\n",
				__func__);
			return -EPROTO;
		}

		if (ts->response_buf[size - 1] != HID_OUTPUT_BL_EOP) {
			dev_err(ts->dev, "%s: HID output response, wrong EOP\n",
				__func__);
			return -EPROTO;
		}

		crc = cyttsp5_compute_crc(&ts->response_buf[4], size - 7);
		if (ts->response_buf[size - 3] != LOW_BYTE(crc)
		    || ts->response_buf[size - 2] != HI_BYTE(crc)) {
			dev_err(ts->dev, "%s: HID output response, wrong CRC 0x%X\n",
				__func__, crc);
			return -EPROTO;
		}

		status = ts->response_buf[5];
		if (status) {
			dev_err(ts->dev, "%s: HID output response, ERROR:%d\n",
				__func__, status);
			return -EPROTO;
		}
	}

	if (offset == HID_APP_RESPONSE_REPORT_ID) {
		command_code = ts->response_buf[HID_OUTPUT_RESPONSE_CMD_OFFSET]
			& HID_OUTPUT_RESPONSE_CMD_MASK;
		if (command_code != code) {
			dev_err(ts->dev,
				"%s: HID output response, wrong command_code:%X\n",
				__func__, command_code);
			return -EPROTO;
		}
	}

	return 0;
}

static void cyttsp5_si_get_btn_data(struct cyttsp5 *ts)
{
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	int i;
	int num_btns = 0;
	unsigned int btns = ts->response_buf[HID_SYSINFO_BTN_OFFSET]
		& HID_SYSINFO_BTN_MASK;

	for (i = 0; i < HID_SYSINFO_MAX_BTN; i++) {
		if (btns & (1 << i))
			num_btns++;
	}
	si->num_btns = num_btns;
}

static int cyttsp5_get_sysinfo_regs(struct cyttsp5 *ts)
{
	struct cyttsp5_sensing_conf_data *scd = &ts->sysinfo.sensing_conf_data;
	struct cyttsp5_sensing_conf_data_dev *scd_dev =
		(struct cyttsp5_sensing_conf_data_dev *)
		&ts->response_buf[HID_SYSINFO_SENSING_OFFSET];
	struct cyttsp5_sysinfo *si = &ts->sysinfo;

	cyttsp5_si_get_btn_data(ts);

	scd->max_tch = scd_dev->max_num_of_tch_per_refresh_cycle;
	scd->res_x = get_unaligned_le16(&scd_dev->res_x);
	scd->res_y = get_unaligned_le16(&scd_dev->res_y);
	scd->max_z = get_unaligned_le16(&scd_dev->max_z);
	scd->len_x = get_unaligned_le16(&scd_dev->len_x);
	scd->len_y = get_unaligned_le16(&scd_dev->len_y);

	si->xy_data = devm_kzalloc(ts->dev, scd->max_tch * TOUCH_REPORT_SIZE,
				   GFP_KERNEL);
	if (!si->xy_data)
		return -ENOMEM;

	si->xy_mode = devm_kzalloc(ts->dev, TOUCH_INPUT_HEADER_SIZE, GFP_KERNEL);
	if (!si->xy_mode) {
		devm_kfree(ts->dev, si->xy_data);
		return -ENOMEM;
	}

	return 0;
}

static int cyttsp5_hid_output_get_sysinfo(struct cyttsp5 *ts)
{
	int rc, t;
	u8 cmd[HID_OUTPUT_GET_SYSINFO_SIZE];

	mutex_lock(&ts->system_lock);
	ts->hid_cmd_state = HID_OUTPUT_GET_SYSINFO + 1;
	mutex_unlock(&ts->system_lock);

	/* HI bytes of Output register address */
	cmd[0] = LOW_BYTE(HID_OUTPUT_GET_SYSINFO_SIZE);
	cmd[1] = HI_BYTE(HID_OUTPUT_GET_SYSINFO_SIZE);
	cmd[2] = HID_APP_OUTPUT_REPORT_ID;
	cmd[3] = 0x0; /* Reserved */
	cmd[4] = HID_OUTPUT_GET_SYSINFO;

	rc = cyttsp5_write(ts, HID_OUTPUT_REG, cmd,
			   HID_OUTPUT_GET_SYSINFO_SIZE);
	if (rc) {
		dev_err(ts->dev, "%s: Failed to write command %d",
			__func__, rc);
		goto error;
	}

	t = wait_event_timeout(ts->wait_q, (ts->hid_cmd_state == 0),
			       msecs_to_jiffies(CY_HID_OUTPUT_GET_SYSINFO_TIMEOUT));
	if (IS_TMO(t)) {
		dev_err(ts->dev, "%s: HID output cmd execution timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	rc = cyttsp5_validate_cmd_response(ts, HID_OUTPUT_GET_SYSINFO);
	goto exit;

error:
	mutex_lock(&ts->system_lock);
	ts->hid_cmd_state = 0;
	mutex_unlock(&ts->system_lock);
	return rc;
exit:
	rc = cyttsp5_get_sysinfo_regs(ts);

	return rc;
}

static int cyttsp5_hid_output_bl_launch_app(struct cyttsp5 *ts)
{
	int rc, t;
	u8 cmd[HID_OUTPUT_BL_LAUNCH_APP];
	u16 crc;

	mutex_lock(&ts->system_lock);
	ts->hid_cmd_state = HID_OUTPUT_BL_LAUNCH_APP + 1;
	mutex_unlock(&ts->system_lock);

	cmd[0] = LOW_BYTE(HID_OUTPUT_BL_LAUNCH_APP_SIZE);
	cmd[1] = HI_BYTE(HID_OUTPUT_BL_LAUNCH_APP_SIZE);
	cmd[2] = HID_BL_OUTPUT_REPORT_ID;
	cmd[3] = 0x0; /* Reserved */
	cmd[4] = HID_OUTPUT_BL_SOP;
	cmd[5] = HID_OUTPUT_BL_LAUNCH_APP;
	cmd[6] = 0x0; /* Low bytes of data */
	cmd[7] = 0x0; /* Hi bytes of data */
	crc = cyttsp5_compute_crc(&cmd[4], 4);
	cmd[8] = LOW_BYTE(crc);
	cmd[9] = HI_BYTE(crc);
	cmd[10] = HID_OUTPUT_BL_EOP;

	rc = cyttsp5_write(ts, HID_OUTPUT_REG, cmd,
			   HID_OUTPUT_BL_LAUNCH_APP_SIZE);
	if (rc) {
		dev_err(ts->dev, "%s: Failed to write command %d",
			__func__, rc);
		goto error;
	}

	t = wait_event_timeout(ts->wait_q, (ts->hid_cmd_state == 0),
			       msecs_to_jiffies(CY_HID_OUTPUT_TIMEOUT));
	if (IS_TMO(t)) {
		dev_err(ts->dev, "%s: HID output cmd execution timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	rc = cyttsp5_validate_cmd_response(ts, HID_OUTPUT_BL_LAUNCH_APP);
	goto exit;

error:
	mutex_lock(&ts->system_lock);
	ts->hid_cmd_state = 0;
	mutex_unlock(&ts->system_lock);
exit:
	return rc;
}

static int cyttsp5_get_hid_descriptor(struct cyttsp5 *ts,
				      struct cyttsp5_hid_desc *desc)
{
	struct device *dev = ts->dev;
	__le16 hid_desc_register = HID_DESC_REG;
	int rc;
	int t;
	u8 cmd[2];

	/* Read HID descriptor length and version */
	mutex_lock(&ts->system_lock);
	ts->hid_cmd_state = 1;
	mutex_unlock(&ts->system_lock);

	/* Set HID descriptor register */
	memcpy(cmd, &hid_desc_register, sizeof(hid_desc_register));

	regmap_write(ts->regmap, HID_DESC_REG, cmd[0]);
	rc = regmap_write(ts->regmap, HID_DESC_REG, cmd[1]);
	if (rc < 0) {
		dev_err(dev, "%s: failed to get HID descriptor, rc=%d\n",
			__func__, rc);
		goto error;
	}

	t = wait_event_timeout(ts->wait_q, (ts->hid_cmd_state == 0),
			       msecs_to_jiffies(CY_HID_GET_HID_DESCRIPTOR_TIMEOUT));
	if (IS_TMO(t)) {
		dev_err(ts->dev, "%s: HID get descriptor timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	memcpy((u8 *)desc, ts->response_buf, sizeof(struct cyttsp5_hid_desc));

	/* Check HID descriptor length and version */
	if (le16_to_cpu(desc->hid_desc_len) != sizeof(*desc) ||
	    le16_to_cpu(desc->bcd_version) != HID_VERSION) {
		dev_err(dev, "%s: Unsupported HID version\n", __func__);
		return -ENODEV;
	}

	goto exit;

error:
	mutex_lock(&ts->system_lock);
	ts->hid_cmd_state = 0;
	mutex_unlock(&ts->system_lock);
exit:
	return rc;
}

static int fill_tch_abs(struct cyttsp5_tch_abs_params *tch_abs, int report_size,
			int offset)
{
	tch_abs->ofs = offset / 8;
	tch_abs->size = report_size / 8;
	if (report_size % 8)
		tch_abs->size += 1;
	tch_abs->min = 0;
	tch_abs->max = 1 << report_size;
	tch_abs->bofs = offset - (tch_abs->ofs << 3);

	return 0;
}

static int move_button_data(struct cyttsp5 *ts, struct cyttsp5_sysinfo *si)
{
	memcpy(si->xy_mode, ts->input_buf, BTN_INPUT_HEADER_SIZE);
	memcpy(si->xy_data, &ts->input_buf[BTN_INPUT_HEADER_SIZE],
	       BTN_REPORT_SIZE);

	return 0;
}

static int move_touch_data(struct cyttsp5 *ts, struct cyttsp5_sysinfo *si)
{
	int max_tch = si->sensing_conf_data.max_tch;
	int num_cur_tch;
	int length;
	struct cyttsp5_tch_abs_params *tch = &si->tch_hdr;

	memcpy(si->xy_mode, ts->input_buf, TOUCH_INPUT_HEADER_SIZE);

	cyttsp5_get_touch_axis(&num_cur_tch, tch->size,
			       tch->max, si->xy_mode + 3 + tch->ofs, tch->bofs);
	if (unlikely(num_cur_tch > max_tch))
		num_cur_tch = max_tch;

	length = num_cur_tch * TOUCH_REPORT_SIZE;

	memcpy(si->xy_data, &ts->input_buf[TOUCH_INPUT_HEADER_SIZE], length);
	return 0;
}

static irqreturn_t cyttsp5_handle_irq(int irq, void *handle)
{
	struct cyttsp5 *ts = handle;
	int report_id;
	int size;
	int rc;

	rc = cyttsp5_read(ts, ts->input_buf, CY_MAX_INPUT);
	if (!rc) {
		size = get_unaligned_le16(&ts->input_buf[0]);

		/* check reset */
		if (size == 0) {
			memcpy(ts->response_buf, ts->input_buf, 2);

			ts->hid_cmd_state = 0;
			wake_up(&ts->wait_q);
			mutex_unlock(&ts->system_lock);
			return IRQ_HANDLED;
		}

		report_id = ts->input_buf[2];

		if (report_id == HID_TOUCH_REPORT_ID) {
			move_touch_data(ts, &ts->sysinfo);
			cyttsp5_mt_attention(ts->dev);
		} else if (report_id == HID_BTN_REPORT_ID) {
			move_button_data(ts, &ts->sysinfo);
			cyttsp5_btn_attention(ts->dev);
		} else {
			/* It is not an input but a command response */
			memcpy(ts->response_buf, ts->input_buf, size);

			mutex_lock(&ts->system_lock);
			ts->hid_cmd_state = 0;
			mutex_unlock(&ts->system_lock);
			wake_up(&ts->wait_q);
		}
	}

	return IRQ_HANDLED;
}

static int cyttsp5_deassert_int(struct cyttsp5 *ts)
{
	u16 size;
	u8 buf[2];
	int rc;

	rc = regmap_bulk_read(ts->regmap, 0, buf, 2);
	if (rc < 0)
		return rc;

	size = get_unaligned_le16(&buf[0]);
	if (size == 2 || size == 0)
		return 0;

	return -EINVAL;
}

static int cyttsp5_fill_all_touch(struct cyttsp5 *ts)
{
	struct cyttsp5_sysinfo *si = &ts->sysinfo;

	fill_tch_abs(&si->tch_abs[CY_TCH_X], REPORT_SIZE_16,
		     TOUCH_REPORT_DESC_X);
	fill_tch_abs(&si->tch_abs[CY_TCH_Y], REPORT_SIZE_16,
		     TOUCH_REPORT_DESC_Y);
	fill_tch_abs(&si->tch_abs[CY_TCH_P], REPORT_SIZE_8,
		     TOUCH_REPORT_DESC_P);
	fill_tch_abs(&si->tch_abs[CY_TCH_T], REPORT_SIZE_5,
		     TOUCH_REPORT_DESC_CONTACTID);
	fill_tch_abs(&si->tch_hdr, REPORT_SIZE_5,
		     TOUCH_REPORT_DESC_HDR_CONTACTCOUNT);
	fill_tch_abs(&si->tch_abs[CY_TCH_MAJ], REPORT_SIZE_8,
		     TOUCH_REPORT_DESC_MAJ);
	fill_tch_abs(&si->tch_abs[CY_TCH_MIN], REPORT_SIZE_8,
		     TOUCH_REPORT_DESC_MIN);

	return 0;
}

static int cyttsp5_startup(struct cyttsp5 *ts)
{
	int rc;

	rc = cyttsp5_deassert_int(ts);
	if (rc) {
		dev_err(ts->dev, "%s: Error on deassert int r=%d\n",
			__func__, rc);
		return -ENODEV;
	}

	/*
	 * Launch the application as the device starts in bootloader mode
	 * because of a power-on-reset
	 */
	rc = cyttsp5_hid_output_bl_launch_app(ts);
	if (rc < 0) {
		dev_err(ts->dev, "%s: Error on launch app r=%d\n",
			__func__, rc);
		goto exit;
	}

	rc = cyttsp5_get_hid_descriptor(ts, &ts->hid_desc);
	if (rc < 0) {
		dev_err(ts->dev, "%s: Error on getting HID descriptor r=%d\n",
			__func__, rc);
		goto exit;
	}

	rc = cyttsp5_fill_all_touch(ts);
	if (rc < 0) {
		dev_err(ts->dev, "%s: Error on getting report descriptor r=%d\n",
			__func__, rc);
		goto exit;
	}

	rc = cyttsp5_hid_output_get_sysinfo(ts);
	if (rc) {
		dev_err(ts->dev, "%s: Error on getting sysinfo r=%d\n",
			__func__, rc);
		goto exit;
	}

exit:

	return rc;
}

static const struct of_device_id cyttsp5_of_match[] = {
	{ .compatible = "cypress,cyttsp5", },
	{ }
};
MODULE_DEVICE_TABLE(of, cyttsp5_of_match);

static const struct i2c_device_id cyttsp5_i2c_id[] = {
	{ CYTTSP5_NAME, 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cyttsp5_i2c_id);

static int cyttsp5_probe(struct device *dev, struct regmap *regmap, int irq,
			 const char *name)
{
	struct cyttsp5 *ts;
	struct cyttsp5_sysinfo *si;
	int rc = 0, i;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		rc = -ENOMEM;
		goto error_alloc_data;
	}

	/* Initialize device info */
	ts->regmap = regmap;
	ts->dev = dev;
	si = &ts->sysinfo;
	dev_set_drvdata(dev, ts);

	/* Initialize mutexes and spinlocks */
	mutex_init(&ts->system_lock);
	mutex_init(&ts->mt_lock);
	mutex_init(&ts->btn_lock);

	/* Initialize wait queue */
	init_waitqueue_head(&ts->wait_q);

	/* Call platform init function */
#if 0 //yusw
	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio)) {
		rc = PTR_ERR(ts->reset_gpio);
		dev_err(dev, "Failed to request reset gpio, error %d\n", rc);
		return rc;
	}
#else
	ts->reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpios", 0);
	/* configure the gpio pins */
	//rc  = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_HIGH, "cyttsp5_reset");
	rc  = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_LOW, "cyttsp5_reset");
	usleep_range(20000,21000);
	gpio_direction_output(ts->reset_gpio, 1);
	usleep_range(50000,51000);
	
	if (rc < 0) {
		dev_err(dev, "Failed to request reset gpio, error %d\n", rc);
		return rc;
	}


#endif

	/* Need a delay to have device up */
	msleep(20);

	ts->irq = irq;
	rc = devm_request_threaded_irq(dev, ts->irq, NULL, cyttsp5_handle_irq,
				       IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				       name, ts);
	if (rc) {
		dev_err(dev, "unable to request IRQ\n");
		goto error_setup_irq;
	}

	rc = cyttsp5_startup(ts);
	if (rc) {
		dev_err(ts->dev, "%s: Fail initial startup r=%d\n",
			__func__, rc);
		goto error_startup;
	}

	rc = cyttsp5_parse_dt_key_code(dev);
	if (rc < 0) {
		dev_err(ts->dev, "%s: Error while parsing dts\n", __func__);
		goto error_startup;
	}

	ts->input = input_allocate_device();
	if (!ts->input) {
		dev_err(dev, "%s: Error, failed to allocate input device\n",
			__func__);
		rc = -ENODEV;
		goto error_startup;
	}

	ts->input->name = "cyttsp5";
	scnprintf(ts->phys, sizeof(ts->phys), "%s/input%d", dev_name(dev), 0);
	ts->input->phys = ts->phys;
	ts->input->dev.parent = ts->dev;
	input_set_drvdata(ts->input, ts);

	touchscreen_parse_properties(ts->input, true, NULL);

	if (si) {
		__set_bit(EV_KEY, ts->input->evbit);
		for (i = 0; i < si->num_btns; i++)
			__set_bit(si->btn[i].key_code, ts->input->keybit);

		rc = cyttsp5_setup_input_device(dev);
		if (rc)
			goto error_init_input;
	}

	return 0;

error_init_input:
	input_free_device(ts->input);
error_startup:
error_setup_irq:
	dev_set_drvdata(dev, NULL);
error_alloc_data:
	dev_err(dev, "%s failed.\n", __func__);
	if (dev->of_node)
		of_node_put(dev->of_node);

	return rc;
}

static int cyttsp5_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct regmap *regmap;
	static const struct regmap_config config = {
		.reg_bits = 8,
		.val_bits = 8,
	};

	regmap = devm_regmap_init_i2c(client, &config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "%s: regmap allocation failed: %ld\n",
			__func__, PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return cyttsp5_probe(&client->dev, regmap, client->irq, client->name);
}

static int cyttsp5_remove(struct device *dev)
{
	const struct of_device_id *match;
	struct cyttsp5 *ts = dev_get_drvdata(dev);

#if 1 //yusw
	gpio_free(ts->reset_gpio);
#endif

	input_unregister_device(ts->input);

	dev_set_drvdata(dev, NULL);

	match = of_match_device(of_match_ptr(cyttsp5_of_match), dev);
	if (match && dev->of_node)
		of_node_put(dev->of_node);

	return 0;
}

static int cyttsp5_i2c_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;

	return cyttsp5_remove(dev);
}

static struct i2c_driver cyttsp5_i2c_driver = {
	.driver = {
		.name = CYTTSP5_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cyttsp5_of_match,
	},
	.probe = cyttsp5_i2c_probe,
	.remove = cyttsp5_i2c_remove,
	.id_table = cyttsp5_i2c_id,
};

module_i2c_driver(cyttsp5_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Touchscreen driver for Cypress TrueTouch Gen 5 Product");
MODULE_AUTHOR("Mylène Josserand <mylene.josserand@free-electrons.com>");


  
