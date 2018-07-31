/*
add by e-planet
supported:
	adv7280
	adv7281
	adv7282
	adv7283
*/
/*
1. Main Map
	a. User Sub Map
	b. Interrupt / VDP Sub Map
	c. User Sub Map 2
2. VPP Map (video post process)
3. CSI Map (mipi-csi model, only in ADV7280-M)
*/

/*Register Map*/
#define MAIN_MAP_ADDR	0x20	//0x21
	#define USER_SUB_MAP	0x00	// register 0x0E[6:5]
	#define VDP_SUB_MAP		0x01	// register 0x0E[6:5]
	#define USER_SUB_MAP2	0x02	// register 0x0E[6:5]
	
#define VPP_MAP_ADDR	0x42	
#define CSI_MAP_ADDR	0x44

/*
#	adv7280_write_reg(0x0F, 0x00);//Exit Power Down Mode [ADV7282 writes begin]
#	adv7280_write_reg(0x00, 0x00);//CVBS in on AIN1
#	adv7280_write_reg(0x0E, 0x80);//ADI Required Write 
#	adv7280_write_reg(0x9C, 0x00);//Reset Clamp Circuitry [step1] 
#	adv7280_write_reg(0x9C, 0xFF);//Reset Clamp Circuitry [step2] 

#	adv7280_write_reg(0x0E, 0x00);//Enter User Sub Map
#	adv7280_write_reg(0x0E, 0x80);//ADI Required Write 
#	adv7280_write_reg(0x9C, 0x00);//ADI Required Write 
#	adv7280_write_reg(0x9C, 0xFF);//ADI Required Write 

#	adv7280_write_reg(0x0E, 0x00);//Enter User Sub Map
#	adv7280_write_reg(0x03, 0x0C);//Enable Pixel & Sync output drivers
#	adv7280_write_reg(0x04, 0x07);//Power-up INTRQ, HS & VS pads
#	adv7280_write_reg(0x13, 0x00);//Enable INTRQ output driver

#	adv7280_write_reg(0x17, 0x41);//Enable SH1
#	adv7280_write_reg(0x1D, 0x40);//Enable LLC output driver
#	adv7280_write_reg(0x52, 0xCD);//ADI Required Write 

#	adv7280_write_reg(0x80, 0x51);//ADI Required Write
#	adv7280_write_reg(0x81, 0x51);//ADI Required Write
#	adv7280_write_reg(0x82, 0x68);//ADI Required Write [ADV7282 writes finished]
*/

/*
:Autodetect CVBS Single Ended In Ain 3, YPrPb Out:
42 0F 80 ; Reset ADV7283

42 0F 00 ; Exit Power Down Mode [ADV7283 writes begin]
42 52 CD ; AFE IBIAS
42 00 02 ; CVBS in on AIN3
42 0E 80 ; ADI Required Write 

42 9C 00 ; Reset Current Clamp Circuitry [step1] 
42 9C FF ; Reset Current Clamp Circuitry [step2] 

42 0E 00 ; Enter User Sub Map
42 80 51 ; ADI Required Write
42 81 51 ; ADI Required Write
42 82 68 ; ADI Required Write

42 17 41 ; Enable SH1
42 03 0C ; Enable Pixel & Sync output drivers
42 04 07 ; Power-up INTRQ, HS & VS pads
42 13 00 ; Enable ADV7283 for 28_63636MHz crystal

42 1D 40 ; Enable LLC output driver [ADV7283 writes finished]


*/

/*
User Sub Map 
*/
#define ADV72XX_INPUT		0x00
#define ADV72XX_VIDEO_SEL_1	0x01
#define ADV72XX_VIDEO_SEL_2	0x02
#define ADV72XX_OUTPUT		0x03
#define ADV72XX_EX_OUTPUT	0x04
//#define ADV72XX_RSV1	0x05
//#define ADV72XX_RSV2	0x06
#define ADV72XX_AUTO_DETECT	0x07
#define ADV72XX_CONTRAST	0x08
//#define ADV72XX_RSV3	0x09
#define ADV72XX_BRIGHTNESS	0x0A
#define ADV72XX_HUE			0x0B
#define ADV72XX_DEFAULT_Y	0x0C
#define ADV72XX_DEFAULT_C	0x0D
#define ADV72XX_ADI_1	0x0E
#define ADV72XX_PWR_MGM		0x0F

#define ADV72XX_STATUS_1	0x10	/* global status registers */
#define ADV72XX_IDENT		0x11
#define ADV72XX_STATUS_2	0x12	/* global status registers */
#define ADV72XX_STATUS_3	0x13	/* global status registers */
#define ADV72XX_AC_CTRL		0x14
#define ADV72XX_DC_CTRL		0x15
//#define ADV72XX_RSV4		0x16
#define ADV72XX_SF_CTRL_1	0x17
#define ADV72XX_SF_CTRL_2	0x18
#define ADV72XX_CF_CTRL		0x19
#define ADV72XX_ADI_2		0x1D

#define ADV72XX_PIXEL_DELAY	0x27
#define ADV72XX_MISC_GAIN	0x2B
#define ADV72XX_AGC_MODE	0x2C
#define ADV72XX_CHROMA_GAIN_1	0x2D
#define ADV72XX_CHROMA_GAIN_2	0x2E
#define ADV72XX_LUMA_GAIN_1	0x2F
#define ADV72XX_LUMA_GAIN_2	0x30

#define ADV72XX_VS_FIELD_1	0x31
#define ADV72XX_VS_FIELD_2	0x32
#define ADV72XX_VS_FIELD_3	0x33
#define ADV72XX_HS_POS_1	0x34
#define ADV72XX_HS_POS_2	0x35
#define ADV72XX_HS_POS_3	0x36
#define ADV72XX_POLARITY	0x37
#define ADV72XX_NTSC_COMB	0x38
#define ADV72XX_PAL_COMB	0x39
#define ADV72XX_MANUAL_WIN	0x3D

#define ADV72XX_RESAMPLE	0x41
#define ADV72XX_CTI_DNR_1	0x4D
#define ADV72XX_CTI_DNR_2	0x4E

#define ADV72XX_DNR_NOISE_TH_1	0x50
#define ADV72XX_LOCK_COUNT		0x51
#define ADV72XX_DIAG_1		0x5D
#define ADV72XX_DIAG_2		0x5E
#define ADV72XX_GPO			0x59

#define ADV72XX_ADC_SIWTCH_3	0x60
#define ADV72XX_OUTPUT_SYNC_SEL_1	0x6A
#define ADV72XX_OUTPUT_SYNC_SEL_2	0x6B

#define ADV72XX_FREERUN_LINE_LEN	0x8F

#define ADV72XX_CCAP_1	0x99
#define ADV72XX_CCAP_2	0x9A
#define ADV72XX_LETTER_BOX_1	0x9B
#define ADV72XX_LETTER_BOX_2	0x9C
#define ADV72XX_LETTER_BOX_3	0x9D

#define ADV72XX_CRC_ENABLE	0xB2

#define ADV72XX_ADC_SWITCH_1	0xC3
#define ADV72XX_ADC_SWITCH_2	0xC4

#define ADV72XX_LETTER_BOX_CTRL_1	0xDC
#define ADV72XX_LETTER_BOX_CTRL_2	0xDD
#define ADV72XX_ST_NOISE_READBACK_1	0xDE
#define ADV72XX_ST_NOISE_READBACK_2	0xDF

#define ADV72XX_SD_OFFSET_CB	0xE1
#define ADV72XX_SD_OFFSET_CR	0xE2
#define ADV72XX_SD_SAUTRATION_CB	0xE3
#define ADV72XX_SD_SAUTRATION_CR	0xE4
#define ADV72XX_NTSC_VBIT_BEGIN	0xE5
#define ADV72XX_NTSC_VBIT_END	0xE6
#define ADV72XX_NTSC_FBIT_TOGGLE	0xE7
#define ADV72XX_PAL_VBIT_BEGIN	0xE8
#define ADV72XX_PAL_VBIT_END	0xE9
#define ADV72XX_PAL_FBIT_TOGGLE	0xEA
#define ADV72XX_VBLANK_1	0xEB
#define ADV72XX_VBLANK_2	0xEE

#define ADV72XX_AFE_1	0xF3
#define ADV72XX_DIRVE_STRENGTH	0xF4
#define ADV72XX_IF_COMP	0xF8
#define ADV72XX_VS_MODE	0xF9
#define ADV72XX_PEAKING_GAIN	0xFB
#define ADV72XX_DNR_NOISE_TH_2	0xFC
#define ADV72XX_VPP_ADDR	0xFD
#define ADV72XX_CSI_ADDR	0xFE

/*
User Sub Map 2
*/

#define ADV72XX_ACE_1	0x80
#define ADV72XX_ACE_4	0x83
#define ADV72XX_ACE_5	0x84
#define ADV72XX_ACE_6	0x85

#define ADV72XX_DITHER	0x92

#define ADV72XX_MIN_MAX_0	0xD9
#define ADV72XX_MIN_MAX_1	0xDA
#define ADV72XX_MIN_MAX_2	0xDB
#define ADV72XX_MIN_MAX_3	0xDC
#define ADV72XX_MIN_MAX_4	0xDD
#define ADV72XX_MIN_MAX_5	0xDE

#define ADV72XX_FL	0xE0
#define ADV72XX_Y_AVERAGE_0	0xE1
#define ADV72XX_Y_AVERAGE_1	0xE2
#define ADV72XX_Y_AVERAGE_2	0xE3
#define ADV72XX_Y_AVERAGE_3	0xE4
#define ADV72XX_Y_AVERAGE_4	0xE5
#define ADV72XX_Y_AVERAGE_5	0xE6
#define ADV72XX_Y_AVERAGE_MSB	0xE7
#define ADV72XX_Y_AVERAGE_LSB	0xE8

/*
Interrupt / VDP Sub Map
*/
#define ADV72XX_INT_CONFIG_1		0x40
#define ADV72XX_INT_STATUS_1		0x42
#define ADV72XX_INT_CLEAR_1		0x43
#define ADV72XX_INT_MASK_1		0x44
#define ADV72XX_RAW_STATUS_2		0x45
#define ADV72XX_INT_STATUS_2		0x46
#define ADV72XX_INT_CLEAR_2		0x47
#define ADV72XX_INT_MASK_2		0x48
#define ADV72XX_RAW_STATUS_3		0x49
#define ADV72XX_INT_STATUS_3		0x4A
#define ADV72XX_INT_CLEAR_3		0x4B
#define ADV72XX_INT_MASK_3		0x4C
#define ADV72XX_INT_STATUS_4		0x4E
#define ADV72XX_INT_CLEAR_4		0x4F

#define ADV72XX_INT_MASK_4		0x50
#define ADV72XX_INT_LATCH_0		0x51

#define ADV72XX_VDP_CONFIG_1		0x60
#define ADV72XX_VDP_ADF_CONFIG_1		0x62
#define ADV72XX_VDP_ADF_CONFIG_2		0x63
#define ADV72XX_VDP_LINE_00E		0x64
#define ADV72XX_VDP_LINE_00F		0x65
#define ADV72XX_VDP_LINE_010		0x66
#define ADV72XX_VDP_LINE_011		0x67
#define ADV72XX_VDP_LINE_012		0x68
#define ADV72XX_VDP_LINE_013		0x69
#define ADV72XX_VDP_LINE_014		0x6A
#define ADV72XX_VDP_LINE_015		0x6B
#define ADV72XX_VDP_LINE_016		0x6C
#define ADV72XX_VDP_LINE_017		0x6D
#define ADV72XX_VDP_LINE_018		0x6E
#define ADV72XX_VDP_LINE_019		0x6F
#define ADV72XX_VDP_LINE_01A		0x70
#define ADV72XX_VDP_LINE_01B		0x71
#define ADV72XX_VDP_LINE_01C		0x72
#define ADV72XX_VDP_LINE_01D		0x73
#define ADV72XX_VDP_LINE_01E		0x74
#define ADV72XX_VDP_LINE_01F		0x75
#define ADV72XX_VDP_LINE_020		0x76
#define ADV72XX_VDP_LINE_021		0x77
#define ADV72XX_VDP_STATUS		0x78
#define ADV72XX_VDP_CCAP_DATA_0		0x79
#define ADV72XX_VDP_CCAP_DATA_1		0x7A
#define ADV72XX_VDP_CGMS_WSS_DATA_0		0x7D
#define ADV72XX_VDP_CGMS_WSS_DATA_1		0x7E
#define ADV72XX_VDP_CGMS_WSS_DATA_2		0x7F

#define ADV72XX_VDP_OUTPUT_SEL		0x9C

/*
VPP Map
*/
#define ADV72XX_DEINT_RESET			0x41
#define ADV72XX_I2C_DEINT_ENABLE	0x55
#define ADV72XX_ADV_TIMING_MODE_EN	0x5B

/*
CSI Map
*/

struct reg_unit{
	unsigned char addr;
	unsigned char val;
};
typedef struct reg_unit reg_unit;
reg_unit* get_reg_param(int mode, int* p_size);

#define CVBS_NULL	0
#define CVBS_AIN1	1
#define CVBS_AIN2	2
#define CVBS_AIN3	3
#define CVBS_AIN4	4
#define CVBS_AIN5	5
#define CVBS_AIN6	6
#define SDT_LUMARAMP 	7
#define SDT_BOUNDARY	8
#define SDT_SINGLE_COLOR	9
#define SDT_COLORBAR	10


//#define ARRAY_SIZE(a) sizeof(a)/sizeof(reg_unit)
