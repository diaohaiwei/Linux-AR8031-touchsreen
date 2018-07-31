#include "adv7280.h"

static reg_unit user_sub_map[] =  {
	/* Datasheet recommends */
	{0x01, 0xc8},	{0x02, 0x04},	{0x03, 0x00},	{0x04, 0x45},	{0x05, 0x00},	{0x06, 0x02},	{0x07, 0x7F},	{0x08, 0x80},
	{0x0A, 0x00},	{0x0B, 0x00},	{0x0C, 0x36},	{0x0D, 0x7C},	{0x0E, 0x00},	{0x0F, 0x00},	{0x13, 0x00},	{0x14, 0x12},
	{0x15, 0x00},	{0x16, 0x00},	{0x17, 0x01},	{0x18, 0x93},	{0xF1, 0x19},	{0x1A, 0x00},	{0x1B, 0x00},	{0x1C, 0x00},
	{0x1D, 0x40},	{0x1E, 0x00},	{0x1F, 0x00},	{0x20, 0x00},	{0x21, 0x00},	{0x22, 0x00},	{0x23, 0xC0},	{0x24, 0x00},
	{0x25, 0x00},	{0x26, 0x00},	{0x27, 0x58},	{0x28, 0x00},	{0x29, 0x00},	{0x2A, 0x00},	{0x2B, 0xE1},	{0x2C, 0xAE},
	{0x2D, 0xF4},	{0x2E, 0x00},	{0x2F, 0xF0},	{0x30, 0x00},	{0x31, 0x12},	{0x32, 0x41},	{0x33, 0x84},	{0x34, 0x00},
	{0x35, 0x02},	{0x36, 0x00},	{0x37, 0x01},	{0x38, 0x80},	{0x39, 0xC0},	{0x3A, 0x10},	{0x3B, 0x05},	{0x3C, 0x58},
	{0x3D, 0xB2},	{0x3E, 0x64},	{0x3F, 0xE4},	{0x40, 0x90},	{0x41, 0x01},	{0x42, 0x7E},	{0x43, 0xA4},	{0x44, 0xFF},
	{0x45, 0xB6},	{0x46, 0x12},	{0x48, 0x00},	{0x49, 0x00},	{0x4A, 0x00},	{0x4B, 0x00},	{0x4C, 0x00},	{0x4D, 0xEF},
	{0x4E, 0x08},	{0x4F, 0x08},	{0x50, 0x08},	{0x51, 0x24},	{0x52, 0x0B},	{0x53, 0x4E},	{0x54, 0x80},	{0x55, 0x00},
	{0x56, 0x10},	{0x57, 0x00},	{0x58, 0x00},	{0x59, 0x00},	{0x5A, 0x00},	{0x5B, 0x00},	{0x5C, 0x00},	{0x5D, 0x00},
	{0x5E, 0x00},	{0x5F, 0x00},	{0x60, 0x00},	{0x61, 0x00},	{0x62, 0x20},	{0x63, 0x00},	{0x64, 0x00},	{0x65, 0x00},
	{0x66, 0x00},	{0x67, 0x03},	{0x68, 0x01},	{0x69, 0x00},	{0x6A, 0x00},	{0x6B, 0xC0},	{0x6C, 0x00},	{0x6D, 0x00},
	{0x6E, 0x00},	{0x6F, 0x00},	{0x70, 0x00},	{0x71, 0x00},	{0x72, 0x00},	{0x73, 0x10},	{0x74, 0x04},	{0x75, 0x01},
	{0x76, 0x00},	{0x77, 0x3F},	{0x78, 0xFF},	{0x79, 0xFF},	{0x7A, 0xFF},	{0x7B, 0x1E},	{0x7C, 0xC0},	{0x7D, 0x00},
	{0x7E, 0x00},	{0x7F, 0x00},	{0x80, 0x00},	{0x81, 0xC0},	{0x82, 0x04},	{0x83, 0x00},	{0x84, 0x0C},	{0x85, 0x02},
	{0x86, 0x03},	{0x87, 0x63},	{0x88, 0x5A},	{0x89, 0x08},	{0x8A, 0x10},	{0x8B, 0x00},	{0x8C, 0x40},	{0x8D, 0x00},
	{0x8E, 0x40},	{0x8F, 0x00},	{0x90, 0x00},	{0x91, 0x50},	{0x92, 0x00},	{0x93, 0x00},	{0x94, 0x00},	{0x95, 0x00},
	{0x96, 0x00},	{0x97, 0xF0},	{0x98, 0x00},	{0x99, 0x00},	{0x9A, 0x00},	{0x9B, 0x00},	{0x9C, 0x00},	{0x9D, 0x00},
	{0x9E, 0x00},	{0x9F, 0x00},	{0xA0, 0x00},	{0xA1, 0x00},	{0xA2, 0x00},	{0xA3, 0x00},	{0xA4, 0x00},	{0xA5, 0x00},
	{0xA6, 0x00},	{0xA7, 0x00},	{0xA8, 0x00},	{0xA9, 0x00},	{0xAA, 0x00},	{0xAB, 0x00},	{0xAC, 0x00},	{0xAD, 0x00},
	{0xAE, 0x60},	{0xAF, 0x00},	{0xB0, 0x00},	{0xB1, 0x60},	{0xB2, 0x1C},	{0xB3, 0x54},	{0xB4, 0x00},	{0xB5, 0x00},
	{0xB6, 0x00},	{0xB7, 0x13},	{0xB8, 0x03},	{0xB9, 0x33},	{0xBF, 0x02},	{0xC0, 0x00},	{0xC1, 0x00},	{0xC2, 0x00},
	{0xC3, 0x00},	{0xC4, 0x00},	{0xC5, 0x81},	{0xC6, 0x00},	{0xC7, 0x00},	{0xC8, 0x00},	{0xC9, 0x04},	{0xCC, 0x69},
	{0xCD, 0x00},	{0xCE, 0x01},	{0xCF, 0xB4},	{0xD0, 0x00},	{0xD1, 0x10},	{0xD2, 0xFF},	{0xD3, 0xFF},	{0xD4, 0x7F},
	{0xD5, 0x7F},	{0xD6, 0x3E},	{0xD7, 0x08},	{0xD8, 0x3C},	{0xD9, 0x08},	{0xDA, 0x3C},	{0xDB, 0x9B},	{0xDC, 0xAC},
	{0xDD, 0x4C},	{0xDE, 0x00},	{0xDF, 0x00},	{0xE0, 0x14},	{0xE1, 0x80},	{0xE2, 0x80},	{0xE3, 0x80},	{0xE4, 0x80},
	{0xE5, 0x25},	{0xE6, 0x44},	{0xE7, 0x63},	{0xE8, 0x65},	{0xE9, 0x14},	{0xEA, 0x63},	{0xEB, 0x55},	{0xEC, 0x55},
	{0xEE, 0x00},	{0xEF, 0x4A},	{0xF0, 0x44},	{0xF1, 0x0C},	{0xF2, 0x32},	{0xF3, 0x00},	{0xF4, 0x3F},	{0xF5, 0xE0},
	{0xF6, 0x69},	{0xF7, 0x10},	{0xF8, 0x00},	{0xF9, 0x03},	{0xFA, 0xFA},	{0xFB, 0x40},
};

//Free-run, 480i 60Hz YPrPb Out:
static reg_unit adv7280_YPrPb_480i_60Hz[] = 
{
	{0x0F, 0x00},// Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x03},// ADI Required Write
	{0x0C, 0x37},// Force Free run mode
	{0x02, 0x54},// Force standard to NTSC-M
	{0x14, 0x11},// Set Free-run pattern to 100% color bars
	{0x03, 0x0C},// Enable Pixel & Sync output drivers
	{0x04, 0x07},// Power-up INTRQ, HS & VS pads
	{0x13, 0x00},// Enable INTRQ output driver
	{0x17, 0x41},// Enable SH1
	{0x1D, 0x40},// Enable LLC output driver
	{0x52, 0xCD},// ADI Required Write 
	{0x80, 0x51},// ADI Required Write
	{0x81, 0x51},// ADI Required Write
	{0x82, 0x68},// ADI Required Write [ADV7282 writes finished]
};

//Free-run, Color Bars 480p YPrPb Out:
static reg_unit adv7280_ColorBar_YPrPb_480p[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x03},//ADI Required Write
	{0x0C, 0x37},//Force Free run mode
	{0x02, 0x54},//Force standard to NTSC-M
	{0x14, 0x11},//Set Free-run pattern to 100% color bars
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write 
	{0x81, 0x51},//ADI Required Write 
	{0x82, 0x68},//ADI Required Write

	//{0xFD, 0x84},//Set VPP Map                            
	//vpp_map_write(0x84, 0xA3, 0x00},//ADI Required Write [ADV7282 VPP writes begin]
	//vpp_map_write(0x84, 0x5B, 0x00},//Enable Advanced Timing Mode
	//vpp_map_write(0x84, 0x55, 0x80},//Enable the Deinterlacer for I2P [All ADV7282 writes finished]
	
};                      
                             
//Free-run, 576i 50Hz YPrPb Out:
static reg_unit adv7280_YPrPb_576i_50Hz[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x0C, 0x37},//Force Free run mode
	{0x02, 0x84},//Force standard to PAL
	{0x00, 0x03},//ADI Required Write
	{0x14, 0x11},//Set Free-run pattern to 100% color bars
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write
	{0x81, 0x51},//ADI Required Write
	{0x82, 0x68},//ADI Required Write [ADV7282 writes finished]
	
};

//Free-run, Color Bars 576p YPrPb Out:
static reg_unit adv7280_ColorBar_YPrPb_576p[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x03},//ADI Required Write
	{0x0C, 0x37},//Force Free run mode
	{0x02, 0x84},//Force standard to PAL
	{0x14, 0x11},//Set Free-run pattern to 100% color bars
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write 
	{0x81, 0x51},//ADI Required Write 
	{0x82, 0x68},//ADI Required Write
	
	//{0xFD, 0x84},//Set VPP Map
	//vpp_map_write(0x84, 0xA3, 0x00},//ADI Required Write [ADV7282 VPP writes begin]
	//vpp_map_write(0x84, 0x5B, 0x00},//Enable Advanced Timing Mode
	//vpp_map_write(0x84, 0x55, 0x80},//Enable the Deinterlacer for I2P [All ADV7282 writes finished]
};

//Autodetect CVBS Single Ended In Ain 1, YPrPb Out:
static reg_unit adv7280_CVBS_YPrPb[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x00},//CVBS in on AIN1
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//Reset Clamp Circuitry [step1] 
	{0x9C, 0xFF},//Reset Clamp Circuitry [step2] 
	{0x0E, 0x00},//Enter User Sub Map
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//ADI Required Write 
	{0x9C, 0xFF},//ADI Required Write 
	{0x0E, 0x00},//Enter User Sub Map
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write
	{0x81, 0x51},//ADI Required Write
	{0x82, 0x68},//ADI Required Write [ADV7282 writes finished]
};

//FAST Switch CVBS Single Ended In Ain1, YPrPb Out:
static reg_unit adv7280_CVBS_YPrPb_fast_switch[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x00},//CVBS in on AIN1
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//Reset Clamp Circuitry [step1] 
	{0x9C, 0xFF},//Reset Clamp Circuitry [step2] 
	{0x0E, 0x00},//Enter User Sub Map
	{0x0E, 0x80},//ADI Required Write
	{0x9C, 0x00},//ADI Required Write
	{0x9C, 0xFF},//ADI Required Write
	{0x0E, 0x00},//Enter User Sub Map
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write
	{0x80, 0x51},//ADI Required Write
	{0x81, 0x51},//ADI Required Write
	{0x82, 0x68},//ADI Required Write
	{0x0E, 0x80},//ADI Required Write
	{0xD9, 0x44},//ADI Required Write
	{0x0E, 0x00},//ADI Required Write
	{0x0E, 0x40},//Select User Sub Map 2
	{0xE0, 0x01},//Select fast Switching Mode.
	{0x0E, 0x00},//Select User Map [ADV7282 writes finished]
	
};

//I2P - NTSC In Ain1,YPrPb Out (480p EAV/SAV):
static reg_unit adv7280_NTSC_YPrPb_480p[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x00},//CVBS in on AIN1
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//Reset Clamp Circuitry [step1] 
	{0x9C, 0xFF},//Reset Clamp Circuitry [step2] 
	{0x0E, 0x00},//Enter User Sub Map
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//ADI Required Write 
	{0x9C, 0xFF},//ADI Required Write 
	{0x0E, 0x00},//Enter User Sub Map
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write 
	{0x81, 0x51},//ADI Required Write 
	{0x82, 0x68},//ADI Required Write

	//{0xFD, 0x84},//Set VPP Map
	//vpp_map_write(0x84, 0xA3, 0x00},//ADI Required Write [ADV7282 VPP writes begin]
	//vpp_map_write(0x84, 0x5B, 0x00},//Enable Advanced Timing Mode
	//vpp_map_write(0x84, 0x55, 0x80},//Enable the Deinterlacer for I2P [All ADV7282 writes finished]
};
                              
//I2P - PAL In Ain1, YPrPb Out (576p EAV/SAV):
static reg_unit adv7280_PAL_YPrPb_576[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x00},//CVBS in on AIN1
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//Reset Clamp Circuitry [step1] 
	{0x9C, 0xFF},//Reset Clamp Circuitry [step2] 
	{0x0E, 0x00},//Enter User Sub Map
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//ADI Required Write 
	{0x9C, 0xFF},//ADI Required Write 
	{0x0E, 0x00},//Enter User Sub Map
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ pad, Enable SFL & VS pin
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write
	{0x81, 0x51},//ADI Required Write
	{0x82, 0x68},//ADI Required Write
	
	//{0xFD, 0x84},//Set VPP Map	
	//vpp_map_write(0x84, 0xA3, 0x00},//ADI Required Write [ADV7282 VPP writes begin]
	//vpp_map_write(0x84, 0x5B, 0x00},//Enable Advanced Timing Mode
	//vpp_map_write(0x84, 0x55, 0x80},//Enable the Deinterlacer for I2P [All ADV7282 writes finished]
	
};
                             
//I2P FAST SWITCH NTSC Single Ended In Ain1, YPrPb Out (480p EAV/SAV)
static reg_unit adv7280_NTSC_YPrPb_480p_fast_switch[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x00},//CVBS in on AIN1
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//Reset Clamp Circuitry [step1] 
	{0x9C, 0xFF},//Reset Clamp Circuitry [step2] 
	{0x0E, 0x00},//Enter User Sub Map
	{0x0E, 0x80},//ADI Required Write
	{0x9C, 0x00},//ADI Required Write
	{0x9C, 0xFF},//ADI Required Write
	{0x0E, 0x00},//Enter User Sub Map
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write
	{0x80, 0x51},//ADI Required Write
	{0x81, 0x51},//ADI Required Write
	{0x82, 0x68},//ADI Required Write
	{0x0E, 0x80},//ADI Required Write
	{0xD9, 0x44},//ADI Required Write
	{0x0E, 0x00},//ADI Required Write
	{0x0E, 0x40},//Select User Sub Map 2
	{0xE0, 0x01},//Select fast Switching Mode.
	{0x0E, 0x00},//Select User Map
	
	//{0xFD, 0x84},//Set VPP Map
	//vpp_map_write(0x84, 0xA3, 0x00},//ADI Required Write [ADV7282 VPP writes begin]
	//vpp_map_write(0x84, 0x5B, 0x00},//Enable Advanced Timing Mode
	//vpp_map_write(0x84, 0x55, 0x80},//Enable the Deinterlacer for I2P [All ADV7282 writes finished]
	
};
                               
//I2P FAST SWITCH PAL Single Ended In Ain1, YPrPb Out (576p EAV/SAV)
static reg_unit adv7280_PAL_YPrPb_576_fast_switch[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x00},//CVBS in on AIN1
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//Reset Clamp Circuitry [step1] 
	{0x9C, 0xFF},//Reset Clamp Circuitry [step2] 
	{0x0E, 0x00},//Enter User Sub Map
	{0x0E, 0x80},//ADI Required Write
	{0x9C, 0x00},//ADI Required Write
	{0x9C, 0xFF},//ADI Required Write
	{0x0E, 0x00},//Enter User Sub Map
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ pad, Enable SFL & VS pin
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write
	{0x80, 0x51},//ADI Required Write
	{0x81, 0x51},//ADI Required Write
	{0x82, 0x68},//ADI Required Write
	{0x0E, 0x80},//ADI Required Write
	{0xD9, 0x44},//ADI Required Write
	{0x0E, 0x00},//ADI Required Write
	{0x0E, 0x40},//Select User Sub Map 2
	{0xE0, 0x01},//Select fast Switching Mode.
	{0x0E, 0x00},//Select User Map
	
	//{0xFD, 0x84},//Set VPP Map
	//vpp_map_write(0x84, 0xA3, 0x00},//ADI Required Write [ADV7282 VPP writes begin]
	//vpp_map_write(0x84, 0x5B, 0x00},//Enable Advanced Timing Mode
	//vpp_map_write(0x84, 0x55, 0x80},//Enable the Deinterlacer for I2P [All ADV7282 writes finished]
};

//4;//5;
reg_unit* get_reg_param_old(int mode, int* p_size){
	reg_unit* ret = 0;
	
	switch(mode){
		case 0:
			*p_size=0;break;
		case 1:
			ret = adv7280_YPrPb_480i_60Hz;
			*p_size = ARRAY_SIZE(adv7280_YPrPb_480i_60Hz);
			break;
		case 2:
			ret = adv7280_ColorBar_YPrPb_480p;
			*p_size = ARRAY_SIZE(adv7280_ColorBar_YPrPb_480p);
			break;
		case 3:
			ret = adv7280_YPrPb_576i_50Hz;
			*p_size = ARRAY_SIZE(adv7280_YPrPb_576i_50Hz);
			break;
		case 4:
			ret = adv7280_ColorBar_YPrPb_576p;
			*p_size = ARRAY_SIZE(adv7280_ColorBar_YPrPb_576p);
			break;
		case 5:
			ret = adv7280_CVBS_YPrPb;
			*p_size = ARRAY_SIZE(adv7280_CVBS_YPrPb);
			break;
		case 6:
			ret = adv7280_CVBS_YPrPb_fast_switch;
			*p_size = ARRAY_SIZE(adv7280_CVBS_YPrPb_fast_switch);
			break;
		case 7:
			ret = adv7280_NTSC_YPrPb_480p;
			*p_size = ARRAY_SIZE(adv7280_NTSC_YPrPb_480p);
			break;
		case 8:
			ret = adv7280_PAL_YPrPb_576;
			*p_size = ARRAY_SIZE(adv7280_PAL_YPrPb_576);
			break;
		case 9:
			ret = adv7280_NTSC_YPrPb_480p_fast_switch;
			*p_size = ARRAY_SIZE(adv7280_NTSC_YPrPb_480p_fast_switch);
			break;
		case 10:
			ret = adv7280_PAL_YPrPb_576_fast_switch;
			*p_size = ARRAY_SIZE(adv7280_PAL_YPrPb_576_fast_switch);
			break;
	
	}
	return ret;
}

static reg_unit adv7280_CVBS_YPrPb_AIN1[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x00},//CVBS in on AIN1
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//Reset Clamp Circuitry [step1] 
	{0x9C, 0xFF},//Reset Clamp Circuitry [step2] 
	{0x0E, 0x00},//Enter User Sub Map
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//ADI Required Write 
	{0x9C, 0xFF},//ADI Required Write 
	{0x0E, 0x00},//Enter User Sub Map
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write
	{0x81, 0x51},//ADI Required Write
	{0x82, 0x68},//ADI Required Write [ADV7282 writes finished]
};

static reg_unit adv7280_CVBS_YPrPb_AIN2[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x01},//CVBS in on AIN1
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//Reset Clamp Circuitry [step1] 
	{0x9C, 0xFF},//Reset Clamp Circuitry [step2] 
	{0x0E, 0x00},//Enter User Sub Map
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//ADI Required Write 
	{0x9C, 0xFF},//ADI Required Write 
	{0x0E, 0x00},//Enter User Sub Map
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write
	{0x81, 0x51},//ADI Required Write
	{0x82, 0x68},//ADI Required Write [ADV7282 writes finished]
};

static reg_unit adv7280_CVBS_YPrPb_AIN3[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x02},//CVBS in on AIN1
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//Reset Clamp Circuitry [step1] 
	{0x9C, 0xFF},//Reset Clamp Circuitry [step2] 
	{0x0E, 0x00},//Enter User Sub Map
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//ADI Required Write 
	{0x9C, 0xFF},//ADI Required Write 
	{0x0E, 0x00},//Enter User Sub Map
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write
	{0x81, 0x51},//ADI Required Write
	{0x82, 0x68},//ADI Required Write [ADV7282 writes finished]
};
static reg_unit adv7280_CVBS_YPrPb_AIN4[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x03},//CVBS in on AIN1
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//Reset Clamp Circuitry [step1] 
	{0x9C, 0xFF},//Reset Clamp Circuitry [step2] 
	{0x0E, 0x00},//Enter User Sub Map
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//ADI Required Write 
	{0x9C, 0xFF},//ADI Required Write 
	{0x0E, 0x00},//Enter User Sub Map
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write
	{0x81, 0x51},//ADI Required Write
	{0x82, 0x68},//ADI Required Write [ADV7282 writes finished]
};
static reg_unit adv7280_CVBS_YPrPb_AIN5[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x06},//CVBS in on AIN1
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//Reset Clamp Circuitry [step1] 
	{0x9C, 0xFF},//Reset Clamp Circuitry [step2] 
	{0x0E, 0x00},//Enter User Sub Map
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//ADI Required Write 
	{0x9C, 0xFF},//ADI Required Write 
	{0x0E, 0x00},//Enter User Sub Map
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write
	{0x81, 0x51},//ADI Required Write
	{0x82, 0x68},//ADI Required Write [ADV7282 writes finished]
};
static reg_unit adv7280_CVBS_YPrPb_AIN6[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x07},//CVBS in on AIN1
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//Reset Clamp Circuitry [step1] 
	{0x9C, 0xFF},//Reset Clamp Circuitry [step2] 
	{0x0E, 0x00},//Enter User Sub Map
	{0x0E, 0x80},//ADI Required Write 
	{0x9C, 0x00},//ADI Required Write 
	{0x9C, 0xFF},//ADI Required Write 
	{0x0E, 0x00},//Enter User Sub Map
	
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write
	{0x81, 0x51},//ADI Required Write
	{0x82, 0x68},//ADI Required Write [ADV7282 writes finished]
};

// [0C]
// bit 0: Forces free-run mode
// bit 1: Enables automatic free-run mode
// [02] default is 04, 84 means PAL B/G/H/I/D

//Free-run, Color Bars 576p YPrPb Out:
static reg_unit adv7280_SDT_YPrPb_ColorBar[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x03},//ADI Required Write
	{0x0C, 0x37},//Force Free run mode
	{0x02, 0x84},//Force standard to PAL
	{0x14, 0x11},//Set Free-run pattern to 100% color bars
	
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write 
	{0x81, 0x51},//ADI Required Write 
	{0x82, 0x68},//ADI Required Write
	
	//{0xFD, 0x84},//Set VPP Map
	//vpp_map_write(0x84, 0xA3, 0x00},//ADI Required Write [ADV7282 VPP writes begin]
	//vpp_map_write(0x84, 0x5B, 0x00},//Enable Advanced Timing Mode
	//vpp_map_write(0x84, 0x55, 0x80},//Enable the Deinterlacer for I2P [All ADV7282 writes finished]
};

static reg_unit adv7280_SDT_YPrPb_LumaRamp[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x03},//ADI Required Write
	{0x0C, 0x37},//Force Free run mode
	{0x02, 0x84},//Force standard to PAL
	{0x14, 0x12},//Set Free-run pattern to luma ramp
	
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write 
	{0x81, 0x51},//ADI Required Write 
	{0x82, 0x68},//ADI Required Write
	
	//{0xFD, 0x84},//Set VPP Map
	//vpp_map_write(0x84, 0xA3, 0x00},//ADI Required Write [ADV7282 VPP writes begin]
	//vpp_map_write(0x84, 0x5B, 0x00},//Enable Advanced Timing Mode
	//vpp_map_write(0x84, 0x55, 0x80},//Enable the Deinterlacer for I2P [All ADV7282 writes finished]
};

static reg_unit adv7280_SDT_YPrPb_Boundary[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x03},//ADI Required Write
	{0x0C, 0x37},//Force Free run mode
	{0x02, 0x84},//Force standard to PAL
	{0x14, 0x15},//Set Free-run pattern to boundary
	
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write 
	{0x81, 0x51},//ADI Required Write 
	{0x82, 0x68},//ADI Required Write
	
	//{0xFD, 0x84},//Set VPP Map
	//vpp_map_write(0x84, 0xA3, 0x00},//ADI Required Write [ADV7282 VPP writes begin]
	//vpp_map_write(0x84, 0x5B, 0x00},//Enable Advanced Timing Mode
	//vpp_map_write(0x84, 0x55, 0x80},//Enable the Deinterlacer for I2P [All ADV7282 writes finished]
};

static reg_unit adv7280_SDT_YPrPb_SingleColor[] =
{
	{0x0F, 0x00},//Exit Power Down Mode [ADV7282 writes begin]
	{0x00, 0x03},//ADI Required Write
	{0x0C, 0x37},//Force Free run mode
	{0x02, 0x84},//Force standard to PAL
	{0x14, 0x10},//Set Free-run pattern to single color
	
	{0x03, 0x0C},//Enable Pixel & Sync output drivers
	{0x04, 0x07},//Power-up INTRQ, HS & VS pads
	{0x13, 0x00},//Enable INTRQ output driver
	{0x17, 0x41},//Enable SH1
	{0x1D, 0x40},//Enable LLC output driver
	
	{0x52, 0xCD},//ADI Required Write 
	{0x80, 0x51},//ADI Required Write 
	{0x81, 0x51},//ADI Required Write 
	{0x82, 0x68},//ADI Required Write
	
	//{0xFD, 0x84},//Set VPP Map
	//vpp_map_write(0x84, 0xA3, 0x00},//ADI Required Write [ADV7282 VPP writes begin]
	//vpp_map_write(0x84, 0x5B, 0x00},//Enable Advanced Timing Mode
	//vpp_map_write(0x84, 0x55, 0x80},//Enable the Deinterlacer for I2P [All ADV7282 writes finished]
};

reg_unit* get_reg_param(int mode, int* p_size){
	reg_unit* ret = 0;
	
	switch(mode){
		case CVBS_NULL:
			pr_info("CVBS_NULL/n");
			*p_size=0;break;
		case CVBS_AIN1:
			pr_info("CVBS_AIN1/n");
			ret = adv7280_CVBS_YPrPb_AIN1;
			*p_size = ARRAY_SIZE(adv7280_CVBS_YPrPb_AIN1);
			break;
		case CVBS_AIN2:
			pr_info("CVBS_AIN2/n");
			ret = adv7280_CVBS_YPrPb_AIN2;
			*p_size = ARRAY_SIZE(adv7280_CVBS_YPrPb_AIN2);
			break;
		case CVBS_AIN3:
			pr_info("CVBS_AIN3/n");
			ret = adv7280_CVBS_YPrPb_AIN3;
			*p_size = ARRAY_SIZE(adv7280_CVBS_YPrPb_AIN3);
			break;
		case CVBS_AIN4:
			pr_info("CVBS_AIN4/n");
			ret = adv7280_CVBS_YPrPb_AIN4;
			*p_size = ARRAY_SIZE(adv7280_CVBS_YPrPb_AIN4);
			break;
		case CVBS_AIN5:
			pr_info("CVBS_AIN5/n");
			ret = adv7280_CVBS_YPrPb_AIN5;
			*p_size = ARRAY_SIZE(adv7280_CVBS_YPrPb_AIN5);
			break;
		case CVBS_AIN6:
			pr_info("CVBS_AIN6/n");
			ret = adv7280_CVBS_YPrPb_AIN6;
			*p_size = ARRAY_SIZE(adv7280_CVBS_YPrPb_AIN6);
			break;
			
		case SDT_COLORBAR:
			pr_info("SDT_COLORBAR/n");
			ret = adv7280_SDT_YPrPb_ColorBar;
			*p_size = ARRAY_SIZE(adv7280_SDT_YPrPb_ColorBar);
			break;
	
		case SDT_LUMARAMP:
			pr_info("SDT_LUMARAMP/n");
			ret = adv7280_SDT_YPrPb_LumaRamp;
			*p_size = ARRAY_SIZE(adv7280_SDT_YPrPb_LumaRamp);
			break;
		case SDT_BOUNDARY:
			pr_info("SDT_COLORBAR/n");
			ret = adv7280_SDT_YPrPb_Boundary;
			*p_size = ARRAY_SIZE(adv7280_SDT_YPrPb_Boundary);
			break;
			
		case SDT_SINGLE_COLOR:
			pr_info("SDT_SINGLE_COLOR/n");
			ret = adv7280_SDT_YPrPb_SingleColor;
			*p_size = ARRAY_SIZE(adv7280_SDT_YPrPb_SingleColor);
			break;
	}
	return ret;
}