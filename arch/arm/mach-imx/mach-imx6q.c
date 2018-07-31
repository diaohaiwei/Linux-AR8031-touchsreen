/*
 * Copyright 2011-2014 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/can/platform/flexcan.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/clocksource.h>
#include <linux/cpu.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/opp.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/micrel_phy.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/of_net.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>
#include <linux/delay.h>
#ifdef CONFIG_MX6_CLK_FOR_BOOTUI_TRANS
#include <linux/memblock.h>
#endif

#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mii.h>
#include "common.h"
#include "cpuidle.h"
#include "hardware.h"
#include "tja110x.h"

static int managed_mode;
/* Permission: do not show up in sysfs */
module_param(managed_mode, int, 0000);
MODULE_PARM_DESC(managed_mode, "Use PHY in managed or autonomous mode");
/* Detemines the level of verbosity for debug messages */
static int verbosity;
/* Permission: do not show up in sysfs */
module_param(verbosity, int, 0000);
MODULE_PARM_DESC(verbosity, "Set verbosity level");

static struct flexcan_platform_data flexcan_pdata[2];
static int flexcan_en_gpio;
static int flexcan_stby_gpio;
static int flexcan0_en;
static int flexcan1_en;
static int verbosity;
static void mx6q_flexcan_switch(void)
{
	if (flexcan0_en || flexcan1_en) {
		/*
		 * The transceiver TJA1041A on sabreauto RevE baseboard will
		 * fail to transit to Normal state if EN/STBY is high by default
		 * after board power up. So we set the EN/STBY initial state to low
		 * first then to high to guarantee the state transition successfully.
		 */
		#if 1
		//gpio_set_value_cansleep(flexcan_en_gpio, 0);
		gpio_set_value_cansleep(flexcan_stby_gpio, 0);
		udelay(1000);
		//gpio_set_value_cansleep(flexcan_en_gpio, 1);
		gpio_set_value_cansleep(flexcan_stby_gpio, 1);
		udelay(1000);
		gpio_set_value_cansleep(flexcan_stby_gpio, 0);

		#endif
		printk("EPT-------can stby setup en-------\n");
	} else {
		/*
		 * avoid to disable CAN xcvr if any of the CAN interfaces
		 * are down. XCRV will be disabled only if both CAN2
		 * interfaces are DOWN.
		*/
		#if 1
		//gpio_set_value_cansleep(flexcan_en_gpio, 0);
		gpio_set_value_cansleep(flexcan_stby_gpio, 0);
		#endif

		printk("EPT-------can stby setup off-------\n");

	}
}

static void imx6q_flexcan0_switch_auto(int enable)
{
	flexcan0_en = enable;
	mx6q_flexcan_switch();
}

static void imx6q_flexcan1_switch_auto(int enable)
{
	flexcan1_en = enable;
	mx6q_flexcan_switch();
}

static int __init imx6q_flexcan_fixup_auto(void)
{
	struct device_node *np;
	//bool gpio_valid, request_valid;
	printk("Harry:-------imx6q_flexcan_fixup_auto-------\n");
	np = of_find_node_by_path("/soc/aips-bus@02000000/can@02090000");
	if (!np)
	{
		printk("Harry:-------of_find_node_by_path failed!-------\n");
		return -ENODEV;
	}
	printk("Harry:-------flexcan_stby_gpio-------\n");
	//flexcan_en_gpio = of_get_named_gpio(np, "trx-en-gpio", 0);
	flexcan_stby_gpio = of_get_named_gpio(np, "trx-stby-gpio", 0);
	/*if (gpio_is_valid(flexcan_en_gpio) && gpio_is_valid(flexcan_stby_gpio) &&
		!gpio_request_one(flexcan_en_gpio, GPIOF_DIR_OUT, "flexcan-trx-en") &&
		!gpio_request_one(flexcan_stby_gpio, GPIOF_DIR_OUT, "flexcan-trx-stby"))*/
	//gpio_valid = gpio_is_valid(flexcan_stby_gpio);
	//request_valid = gpio_request_one(flexcan_stby_gpio, GPIOF_DIR_OUT, "flexcan-trx-stby");
	//printk("Harry:*****gpio_valid = %d  request_valid = %d \n", gpio_valid, request_valid);
	if(gpio_is_valid(flexcan_stby_gpio) &&
		!gpio_request_one(flexcan_stby_gpio, GPIOF_DIR_OUT, "flexcan-trx-stby")){
		/* flexcan 0 & 1 are using the same GPIOs for transceiver */
		printk("Harry:-------gpio_is_valid-------\n");
		flexcan_pdata[0].transceiver_switch = imx6q_flexcan0_switch_auto;
		flexcan_pdata[1].transceiver_switch = imx6q_flexcan1_switch_auto;
	}

	return 0;
}

/* For imx6q sabrelite board: set KSZ9021RN RGMII pad skew */
static int ksz9021rn_phy_fixup(struct phy_device *phydev)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		/* min rx data delay */
		phy_write(phydev, 0x0b, 0x8105);
		phy_write(phydev, 0x0c, 0x0000);

		/* max rx/tx clock delay, min rx/tx control delay */
		phy_write(phydev, 0x0b, 0x8104);
		phy_write(phydev, 0x0c, 0xf0f0);
		phy_write(phydev, 0x0b, 0x104);
	}

	return 0;
}

static void mmd_write_reg(struct phy_device *dev, int device, int reg, int val)
{
	phy_write(dev, 0x0d, device);
	phy_write(dev, 0x0e, reg);
	phy_write(dev, 0x0d, (1 << 14) | device);
	phy_write(dev, 0x0e, val);
}

static int ksz9031rn_phy_fixup(struct phy_device *dev)
{
	/*
	 * min rx data delay, max rx/tx clock delay,
	 * min rx/tx control delay
	 */
	mmd_write_reg(dev, 2, 4, 0);
	mmd_write_reg(dev, 2, 5, 0);
	mmd_write_reg(dev, 2, 8, 0x003ff);

	return 0;
}

static int ar8031_phy_fixup(struct phy_device *dev)
{
	u16 val;
	printk("Harry:************ar8031_phy_fixup!");
	/* disable phy AR8031 SmartEEE function. */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);
	val = phy_read(dev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(dev, 0xe, val);

	/* To enable AR8031 output a 125MHz clk from CLK_25M */
	phy_write(dev, 0xd, 0x7);
	phy_write(dev, 0xe, 0x8016);
	phy_write(dev, 0xd, 0x4007);

	val = phy_read(dev, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(dev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(dev, 0x1d, 0x5);
	val = phy_read(dev, 0x1e);
	val |= 0x0100;
	phy_write(dev, 0x1e, val);

	return 0;
}

/* helper function, waits until a given condition is met
 *
 * The function delays until the part of the register at reg_addr,
 * defined by reg_mask equals cond, or a timeout (timeout*DELAY_LENGTH) occurs.
 * @return	0 if condition is met, <0 if timeout or read error occurred
 */
static int wait_on_condition(struct phy_device *phydev, int reg_addr,
			     int reg_mask, int cond, int timeout)
{
	int reg_val;

	do {
		udelay(DELAY_LENGTH);
		reg_val = phy_read(phydev, reg_addr);
		if (reg_val < 0)
			return reg_val;
	} while ((reg_val & reg_mask) != cond && --timeout);
	if (timeout)
		return 0;
	return -1;
}
static inline int phy_configure_bit(struct phy_device *phydev, int reg_name,
				    int bit_mask, int bit_value)
{
	int reg_val, err;

	reg_val = phy_read(phydev, reg_name);
	if (reg_val < 0)
		goto phy_read_error;

	if (bit_value)
		reg_val |= bit_mask;
	else
		reg_val &= ~bit_mask;

	err = phy_write(phydev, reg_name, reg_val);
	if (err < 0)
		goto phy_write_error;

	return 0;

/* error handling */
phy_read_error:
	dev_err(&phydev->dev, "read error: phy config failed\n");
	return reg_val;

phy_write_error:
	dev_err(&phydev->dev, "write error: phy config failed\n");
	return err;
}
static inline int phy_configure_bits(struct phy_device *phydev, int reg_name,
				     int bit_mask, int bit_value)
{
	int reg_val, err;

	if (verbosity > 2)
		dev_alert(&phydev->dev, "set mask [%08x] of reg [%d] of phy %x to value [%08x]\n",
		bit_mask, reg_name, phydev->addr, bit_value);

	reg_val = phy_read(phydev, reg_name);
	if (reg_val < 0)
		goto phy_read_error;

	reg_val &= ~bit_mask;
	reg_val |= bit_value;

	err = phy_write(phydev, reg_name, reg_val);
	if (err < 0)
		goto phy_write_error;

	return 0;

/* error handling */
phy_read_error:
	dev_err(&phydev->dev, "read error: phy config failed\n");
	return reg_val;

phy_write_error:
	dev_err(&phydev->dev, "write error: phy config failed\n");
	return err;
}
/* helper function, enables or disables link control */
static void set_link_control(struct phy_device *phydev, int enable_link_control)
{
	int err;

	err = phy_configure_bit(phydev, MII_ECTRL, ECTRL_LINK_CONTROL,
				enable_link_control);
	if (err < 0)
		goto phy_configure_error;
	if (verbosity > 1)
		dev_alert(&phydev->dev,
		"set link ctrl to [%d] for phy %x completed\n",
		enable_link_control, phydev->addr);

	return;

phy_configure_error:
	dev_err(&phydev->dev, "phy r/w error: setting link control failed\n");
	return;
}

/* helper function, enters the test mode specified by tmode
 * @return          0 if test mode was entered, <0 on read or write error
 */
static int enter_test_mode(struct phy_device *phydev, enum test_mode tmode)
{
	int reg_val = -1;
	int err;

	if (verbosity > 1)
		dev_alert(&phydev->dev, "phy %x entering test mode: %d\n",
		phydev->addr, tmode);
	switch (tmode) {
	case NO_TMODE:
		reg_val = ECTRL_NO_TMODE;
		break;
	case TMODE1:
		reg_val = ECTRL_TMODE1;
		break;
	case TMODE2:
		reg_val = ECTRL_TMODE2;
		break;
	case TMODE3:
		reg_val = ECTRL_TMODE3;
		break;
	case TMODE4:
		reg_val = ECTRL_TMODE4;
		break;
	case TMODE5:
		reg_val = ECTRL_TMODE5;
		break;
	case TMODE6:
		reg_val = ECTRL_TMODE6;
		break;
	default:
		break;
	}

	if (reg_val >= 0) {
		/* set test mode bits accordingly */
		err = phy_configure_bits(phydev, MII_ECTRL, ECTRL_TEST_MODE,
					 reg_val);
		if (err < 0)
			goto phy_configure_error;
	}

	return 0;

/* error handling */
phy_configure_error:
	dev_err(&phydev->dev, "phy r/w error: setting test mode failed\n");
	return err;
}

static int tja1102_phy_fixup(struct phy_device *phydev)
{
	u16 val;
	int err;
	int reg_name, reg_value = -1, reg_mask;
	dev_alert(&phydev->dev, "initializing phy %x\n", phydev->addr);
	printk("Harry:************tja1102_phy_fixup\n");
	/* enable configuration register access once during initialization */
	err = phy_configure_bit(phydev, MII_ECTRL, ECTRL_CONFIG_EN, 1);
	reg_value = phy_read(phydev, MII_ECTRL);
	printk("Harry:****MII_ECTRL(17) reg_value is %8x  \n", reg_value);
	if (err < 0)
		printk("Harry:************access failed!");
	/* set power mode bits to standby mode */
	err = phy_configure_bits(phydev, MII_ECTRL, ECTRL_POWER_MODE,
				 POWER_MODE_STANDBY);
	if (err < 0)
		printk("Harry:************ to standby mode failed!\n");
	/* wait until power mode transition is completed */
	err = wait_on_condition(phydev, MII_ECTRL, ECTRL_POWER_MODE,
				POWER_MODE_STANDBY, POWER_MODE_TIMEOUT);
	if (err < 0)
		printk("Harry:************wait until to standby mode failed!\n");
	/* set power mode bits to normal mode */
	err = phy_configure_bits(phydev, MII_ECTRL, ECTRL_POWER_MODE,
				 POWER_MODE_NORMAL);
	if (err < 0)
		printk("Harry:************ to normal mode failed!\n");

	/* wait until the PLL is locked, indicating a completed transition */
	err = wait_on_condition(phydev, MII_GENSTAT, GENSTAT_PLL_LOCKED,
				GENSTAT_PLL_LOCKED, POWER_MODE_TIMEOUT);
	if (err < 0)
		printk("Harry:************wait until to normal mode failed!\n");
#if 1
	/* if phy is configured as slave, also send a wakeup request to master */
	/* link control must be reset for wake request */
	set_link_control(phydev, 0);

	/* start sending bus wakeup signal */
	err = phy_configure_bit(phydev, MII_ECTRL, ECTRL_WAKE_REQUEST, 1);
	if (err < 0)
		printk("Harry:************ bus wakeup signal failed!\n");
	/* stop sending bus wakeup signal */
	err = phy_configure_bit(phydev, MII_ECTRL,
				ECTRL_WAKE_REQUEST, 0);
	if (err < 0)
		printk("Harry:************ sending bus wakeup signal!\n");
	/* reenable link control */
	set_link_control(phydev, 1);
#endif
	reg_value = phy_read(phydev, MII_ECTRL);
	printk("Harry:****MII_ECTRL(17) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, MII_CFG2);
	printk("Harry:******MII_CFG2(18) reg_value is %8x  \n", reg_value);
	return 0;
}

int tja1100_phy_fixup(struct phy_device *phydev)
{
	u16 val;
	int err,tmode;
	int reg_name, reg_value = -1, reg_mask;
	dev_alert(&phydev->dev, "initializing phy %x\n", phydev->addr);
	printk("Harry:************tja1100_phy_fixup\n");
	/* disable autoneg and manually configure speed, duplex, pause frames */
	phydev->autoneg = 0;

	phydev->speed = SPEED_100;
	phydev->duplex = DUPLEX_FULL;

	phydev->pause = 0;
	phydev->asym_pause = 0;
	/* set features of the PHY */
	reg_value = phy_read(phydev, MII_BMSR);
	if (reg_value < 0)
		printk("Harry:************set features of the PHY failed!\n");
	if (reg_value & BMSR_ESTATEN) {
		reg_value = phy_read(phydev, MII_ESTATUS);

		if (reg_value < 0)
			printk("Harry:************set BMSR_ESTATEN failed!\n");

		if (reg_value & ESTATUS_100T1_FULL) {
			/* update phydev to include the supported features */
			phydev->supported |= SUPPORTED_100BASET1_FULL;
			phydev->advertising |= ADVERTISED_100BASET1_FULL;
		}
	}
	/* enable configuration register access once during initialization */
	err = phy_configure_bit(phydev, MII_ECTRL, ECTRL_CONFIG_EN, 1);
	reg_value = phy_read(phydev, MII_ECTRL);
	printk("Harry:****MII_ECTRL(17) reg_value is %8x  \n", reg_value);
	if (err < 0)
		printk("Harry:************access failed!");
	reg_name = MII_CFG1;
		reg_value = TJA1100_CFG1_LED_EN | CFG1_LED_LINKUP;
		if (!managed_mode)
			reg_value |= TJA1100_CFG1_AUTO_OP;
		reg_mask = TJA1100_CFG1_AUTO_OP |
		    TJA1100_CFG1_LED_EN | TJA1100_CFG1_LED_MODE;
	/* only configure the phys that have an auto_op bit or leds */
	if (reg_value != -1) {
		err = phy_configure_bits(phydev, reg_name, reg_mask, reg_value);
		if (err < 0)
			printk("Harry:************ configure the phys that have an auto_op bit failed!");
	}
#if 0
    /*Configure MII MODE*/
	err = phy_configure_bits(phydev, MII_CFG1, CFG1_MII_MODE, CFG1_MII_50M_MODE);
	reg_value = phy_read(phydev, MII_CFG1);
	printk("Harry:****MII_CFG1(18) reg_value is %8x  \n", reg_value);
	if (err < 0)
		printk("Harry:************Configure RMII 50MHz failed!");
#endif
	/* enable sleep confirm */
	err = phy_configure_bit(phydev, MII_CFG1, CFG1_SLEEP_CONFIRM, 1);
	if (err < 0)
		printk("Harry:************enable sleep confirm failed!");
	/* set sleep request timeout to 16ms */
	err = phy_configure_bits(phydev, MII_CFG2, CFG2_SLEEP_REQUEST_TO,
				 SLEEP_REQUEST_TO_16MS);
	if (err < 0)
		printk("Harry:************ set sleep request timeout to 16ms failed!");
	/* enable all interrupts
		 * PHY_INTERRUPT_ENABLED macro does not interfere with any
		 * of the possible interrupt source macros
		 */
	err = phy_write(phydev, MII_INTMASK, INTERRUPT_ALL);
	if (err < 0)
		printk("Harry:************ INTERRUPT_ALL failed!");
	/* start sending bus wakeup signal */
	err = phy_configure_bit(phydev, MII_ECTRL,
					ECTRL_WAKE_REQUEST, 1);
	if (err < 0)
		printk("Harry:************ start sending bus wakeup signal  failed!");
	/* wait until link partner is guranteed to be awake */
	usleep_range(TJA100_WAKE_REQUEST_TIMEOUT_US, TJA100_WAKE_REQUEST_TIMEOUT_US + 1U);
	/* stop sending bus wakeup signal */
	err = phy_configure_bit(phydev, MII_ECTRL,
					ECTRL_WAKE_REQUEST, 0);
	if (err < 0)
		printk("Harry:************ stop sending bus wakeup signal  failed!");

	/* reenable link control */
	set_link_control(phydev, 1);
    #if 0
	tmode = 6;
	switch (tmode) {
	case 0:
		err = enter_test_mode(phydev, NO_TMODE);
		/* enable link control after exiting test */
		set_link_control(phydev, 1);
		break;
	case 1:
		/* disbale link control before entering test */
		set_link_control(phydev, 0);
		err = enter_test_mode(phydev, TMODE1);
		printk("Harry:************ Enter phy test mode1!");
		break;
	case 2:
		/* disbale link control before entering test */
		set_link_control(phydev, 0);
		err = enter_test_mode(phydev, TMODE2);
		printk("Harry:************ Enter phy test mode2!");
		break;
	case 3:
		/* disbale link control before entering test */
		set_link_control(phydev, 0);
		err = enter_test_mode(phydev, TMODE3);
		printk("Harry:************ Enter phy test mode3!");
		break;
	case 4:
		/* disbale link control before entering test */
		set_link_control(phydev, 0);
		err = enter_test_mode(phydev, TMODE4);
		printk("Harry:************ Enter phy test mode4!");
		break;
	case 5:
		/* disbale link control before entering test */
		set_link_control(phydev, 0);
		err = enter_test_mode(phydev, TMODE5);
		printk("Harry:************ Enter phy test mode5!");
		break;
	case 6:
		/* disbale link control before entering test */
		set_link_control(phydev, 0);
		err = enter_test_mode(phydev, TMODE6);
		printk("Harry:************ Enter phy test mode6!");
		break;
	default:
		break;
	}
	if (err)
		printk("Harry:************ tmode error: sysfs_get_test_mode failed!");
    #endif
#if 0
	reg_value = phy_read(phydev, MII_BMCR);
	printk("Harry:****MII_ECTRL(0) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, 0x01);
	printk("Harry:****MII_ECTRL(0x01) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, 0x02);
	printk("Harry:****MII_ECTRL(0x02) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, 0x03);
	printk("Harry:****MII_ECTRL(0x03) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, 0x0F);
	printk("Harry:****MII_ECTRL(15) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, MII_ECTRL);
	printk("Harry:****MII_ECTRL(17) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, MII_CFG1);
	printk("Harry:******MII_CFG2(18) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, MII_CFG2);
	printk("Harry:******MII_CFG2(19) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, MII_SYMERRCNT);
	printk("Harry:******MII_CFG2(20) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, MII_INTSRC);
	printk("Harry:******MII_CFG2(21) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, MII_INTMASK);
	printk("Harry:******MII_CFG2(22) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, MII_COMMSTAT);
	printk("Harry:******MII_CFG2(23) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, MII_GENSTAT);
	printk("Harry:******MII_CFG2(24) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, MII_EXTERNAL_STATUS);
	printk("Harry:******MII_CFG2(25) reg_value is %8x  \n", reg_value);
	reg_value = phy_read(phydev, MII_LINK_FAIL_COUNTER);
	printk("Harry:******MII_CFG2(26) reg_value is %8x  \n", reg_value);
#endif
	return 0;
}
EXPORT_SYMBOL(tja1100_phy_fixup);

#define PHY_ID_AR8031	0x004dd074

static void __init imx6q_enet_phy_init(void)
{
	//printk("Harry:************imx6q_enet_phy_init!");
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_KSZ9021, MICREL_PHY_ID_MASK,
				ksz9021rn_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_KSZ9031, MICREL_PHY_ID_MASK,
				ksz9031rn_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_AR8031, 0xffffffff,
				ar8031_phy_fixup);
		/*phy_register_fixup_for_uid(NXP_PHY_ID_TJA1102P0, 0xfffffff0,
				tja1102_phy_fixup);*/
		phy_register_fixup_for_uid(NXP_PHY_ID_TJA1100, 0xfffffff0,
				tja1100_phy_fixup);
	}
}

static void __init imx6q_1588_init(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_ENET_CLK_SEL_MASK,
				IMX6Q_GPR1_ENET_CLK_SEL_ANATOP);
	else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");

}

static void __init imx6q_csi_mux_init(void)
{
	/*
	 * MX6Q SabreSD board:
	 * IPU1 CSI0 connects to parallel interface.
	 * Set GPR1 bit 19 to 0x1.
	 *
	 * MX6DL SabreSD board:
	 * IPU1 CSI0 connects to parallel interface.
	 * Set GPR13 bit 0-2 to 0x4.
	 * IPU1 CSI1 connects to MIPI CSI2 virtual channel 1.
	 * Set GPR13 bit 3-5 to 0x1.
	 */
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		if (of_machine_is_compatible("fsl,imx6q-sabresd") ||
			of_machine_is_compatible("fsl,imx6q-sabreauto"))
			regmap_update_bits(gpr, IOMUXC_GPR1, 1 << 19, 1 << 19);
		else if (of_machine_is_compatible("fsl,imx6dl-sabresd") ||
			 of_machine_is_compatible("fsl,imx6dl-sabreauto"))
			regmap_update_bits(gpr, IOMUXC_GPR13, 0x3F, 0x0C);
	} else {
		pr_err("%s(): failed to find fsl,imx6q-iomux-gpr regmap\n",
		       __func__);
	}
}

#define OCOTP_MACn(n)	(0x00000620 + (n) * 0x10)
void __init imx6_enet_mac_init(const char *compatible)
{
	struct device_node *ocotp_np, *enet_np, *from = NULL;
	void __iomem *base;
	struct property *newmac;
	u32 macaddr_low;
	u32 macaddr_high = 0;
	u32 macaddr1_high = 0;
	u8 *macaddr;
	int i;
	//printk("Harry:-------imx6_enet_mac_init\n");
	for (i = 0; i < 2; i++) {
		enet_np = of_find_compatible_node(from, NULL, compatible);
		if (!enet_np)
			return;

		from = enet_np;

		if (of_get_mac_address(enet_np))
			goto put_enet_node;

		ocotp_np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
		if (!ocotp_np) {
			pr_warn("failed to find ocotp node\n");
			goto put_enet_node;
		}

		base = of_iomap(ocotp_np, 0);
		if (!base) {
			pr_warn("failed to map ocotp\n");
			goto put_ocotp_node;
		}

		macaddr_low = readl_relaxed(base + OCOTP_MACn(1));
		if (i)
			macaddr1_high = readl_relaxed(base + OCOTP_MACn(2));
		else
			macaddr_high = readl_relaxed(base + OCOTP_MACn(0));

		newmac = kzalloc(sizeof(*newmac) + 6, GFP_KERNEL);
		if (!newmac)
			goto put_ocotp_node;

		newmac->value = newmac + 1;
		newmac->length = 6;
		newmac->name = kstrdup("local-mac-address", GFP_KERNEL);
		if (!newmac->name) {
			kfree(newmac);
			goto put_ocotp_node;
		}

		macaddr = newmac->value;
		if (i) {
			macaddr[5] = (macaddr_low >> 16) & 0xff;
			macaddr[4] = (macaddr_low >> 24) & 0xff;
			macaddr[3] = macaddr1_high & 0xff;
			macaddr[2] = (macaddr1_high >> 8) & 0xff;
			macaddr[1] = (macaddr1_high >> 16) & 0xff;
			macaddr[0] = (macaddr1_high >> 24) & 0xff;
		} else {
			macaddr[5] = macaddr_high & 0xff;
			macaddr[4] = (macaddr_high >> 8) & 0xff;
			macaddr[3] = (macaddr_high >> 16) & 0xff;
			macaddr[2] = (macaddr_high >> 24) & 0xff;
			macaddr[1] = macaddr_low & 0xff;
			macaddr[0] = (macaddr_low >> 8) & 0xff;
		}

		of_update_property(enet_np, newmac);

put_ocotp_node:
	of_node_put(ocotp_np);
put_enet_node:
	of_node_put(enet_np);
	}
}

static inline void imx6q_enet_init(void)
{
	//printk("Harry:-------imx6q_enet_init1\n");
	imx6_enet_mac_init("fsl,imx6q-fec");
	printk("Harry:-------imx6q_enet_init2\n");
	imx6q_enet_phy_init();
	printk("Harry:-------imx6q_enet_init3\n");
	imx6q_1588_init();
}

/* Add auxdata to pass platform data */
static const struct of_dev_auxdata imx6q_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02090000, NULL, &flexcan_pdata[0]),
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02094000, NULL, &flexcan_pdata[1]),
	{ /* sentinel */ }
};

static void __init imx6q_init_machine(void)
{
	struct device *parent;
	printk("Harry:************imx6q_init_machine!");
	mxc_arch_reset_init_dt();
	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_populate(NULL, of_default_bus_match_table,
					imx6q_auxdata_lookup, parent);

	imx6q_enet_init();
	imx_anatop_init();
	imx6_pm_init();
	imx6q_csi_mux_init();
}

#define OCOTP_CFG3			0x440
#define OCOTP_CFG3_SPEED_SHIFT		16
#define OCOTP_CFG3_SPEED_1P2GHZ		0x3
#define OCOTP_CFG3_SPEED_1GHZ		0x2
#define OCOTP_CFG3_SPEED_850MHZ		0x1
#define OCOTP_CFG3_SPEED_800MHZ		0x0

static void __init imx6q_opp_check_speed_grading(struct device *cpu_dev)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	/*
	 * SPEED_GRADING[1:0] defines the max speed of ARM:
	 * 2b'11: 1200000000Hz; -- i.MX6Q only.
	 * 2b'10: 1000000000Hz;
	 * 2b'01: 850000000Hz; -- i.MX6Q Only, exclusive with 1GHz.
	 * 2b'00: 800000000Hz;
	 * We need to set the max speed of ARM according to fuse map.
	 */
	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_CFG3_SPEED_SHIFT;
	if (cpu_is_imx6q()) {
		if ((val & 0x3) < OCOTP_CFG3_SPEED_1P2GHZ)
			if (opp_disable(cpu_dev, 1200000000))
				pr_warn("failed to disable 1.2 GHz OPP\n");
	}
	if ((val & 0x3) < OCOTP_CFG3_SPEED_1GHZ)
		if (opp_disable(cpu_dev, 996000000))
			pr_warn("failed to disable 1 GHz OPP\n");
	if (cpu_is_imx6q()) {
		if ((val & 0x3) < OCOTP_CFG3_SPEED_850MHZ ||
			(val & 0x3) == OCOTP_CFG3_SPEED_1GHZ)
			if (opp_disable(cpu_dev, 852000000))
				pr_warn("failed to disable 850 MHz OPP\n");
	}

	if (IS_ENABLED(CONFIG_MX6_VPU_352M)) {
		if (opp_disable(cpu_dev, 396000000))
			pr_warn("failed to disable 396MHz OPP\n");
		pr_info("remove 396MHz OPP for VPU running at 352MHz!\n");
	}

put_node:
	of_node_put(np);
}

static void __init imx6q_opp_init(struct device *cpu_dev)
{
	struct device_node *np;

	np = of_find_node_by_path("/cpus/cpu@0");
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	cpu_dev->of_node = np;
	if (of_init_opp_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

	imx6q_opp_check_speed_grading(cpu_dev);

put_node:
	of_node_put(np);
}

#define ESAI_AUDIO_MCLK 24576000

static void __init imx6q_audio_lvds2_init(void)
{
	struct clk *pll4_sel, *lvds2_in, *pll4_audio_div, *esai_extal;

	pll4_audio_div = clk_get_sys(NULL, "pll4_audio_div");
	pll4_sel = clk_get_sys(NULL, "pll4_sel");
	lvds2_in = clk_get_sys(NULL, "lvds2_in");
	esai_extal = clk_get_sys(NULL, "esai_extal");
	if (IS_ERR(pll4_audio_div) || IS_ERR(pll4_sel) ||
	    IS_ERR(lvds2_in) || IS_ERR(esai_extal))
		return;

	if (clk_get_rate(lvds2_in) != ESAI_AUDIO_MCLK)
		return;

	clk_set_parent(pll4_sel, lvds2_in);
	clk_set_rate(pll4_audio_div, 786432000);
	clk_set_rate(esai_extal, ESAI_AUDIO_MCLK);
}

static struct platform_device imx6q_cpufreq_pdev = {
	.name = "imx6-cpufreq",
};

static void __init imx6q_init_late(void)
{
	struct regmap *gpr;

	/*
	 * Need to force IOMUXC irq pending to meet CCM low power mode
	 * restriction, this is recommended by hardware team.
	 */
	printk("Harry:-------imx6q_init_late-------\n");
	/*phy_register_fixup_for_uid(NXP_PHY_ID_TJA1102P0, 0xfffffff0,
				tja1102_phy_fixup);*/

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1,
			IMX6Q_GPR1_GINT_MASK,
			IMX6Q_GPR1_GINT_ASSERT);

	/*
	 * WAIT mode is broken on TO 1.0 and 1.1, so there is no point
	 * to run cpuidle on them.
	 */
	if ((cpu_is_imx6q() && imx_get_soc_revision() > IMX_CHIP_REVISION_1_1)
		|| (cpu_is_imx6dl() && imx_get_soc_revision() >
		IMX_CHIP_REVISION_1_0))
		imx6q_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6_CPUFREQ)) {
		imx6q_opp_init(&imx6q_cpufreq_pdev.dev);
		platform_device_register(&imx6q_cpufreq_pdev);
	}
	printk("Harry:-------of_machine_is_compatible(fsl,imx6q-sabreauto)-------\n");
	if (of_machine_is_compatible("fsl,imx6q-sabreauto")
		|| of_machine_is_compatible("fsl,imx6dl-sabreauto") || of_machine_is_compatible("fsl,imx6q-sabresd")) {
		imx6q_flexcan_fixup_auto();
		imx6q_audio_lvds2_init();
		printk("Harry:-------imx6q_flexcan_fixup_auto() call condition-------\n");
	}
}

static void __init imx6q_map_io(void)
{
	debug_ll_io_init();
	imx_scu_map_io();
	imx6_pm_map_io();
	imx6_busfreq_map_io();
}

static void __init imx6q_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	imx_gpc_init();
	irqchip_init();
}

static void __init imx6q_timer_init(void)
{
	of_clk_init(NULL);
	clocksource_of_init();
	imx_print_silicon_rev(cpu_is_imx6dl() ? "i.MX6DL" : "i.MX6Q",
			      imx_get_soc_revision());
}

static const char *imx6q_dt_compat[] __initdata = {
	"fsl,imx6dl",
	"fsl,imx6q",
	NULL,
};

#ifdef CONFIG_MX6_CLK_FOR_BOOTUI_TRANS
static void __init imx6q_init_reserve(void)
{
	phys_addr_t base, size;

	/*
	 * Frame buffer base address.
	 * It is same as CONFIG_FB_BASE in Uboot.
	 */
	base = 0x18800000;

	/*
	 * Reserved display memory size.
	 * It should be bigger than 3 x framer buffer size.
	 * For 1080P 32 bpp, 1920*1080*4*3 = 0x017BB000.
	 */
	size = 0x01800000;

	memblock_reserve(base, size);
	memblock_remove(base, size);
}
#endif

DT_MACHINE_START(IMX6Q, "Freescale i.MX6 Quad/DualLite (Device Tree)")
	/*
	 * i.MX6Q/DL maps system memory at 0x10000000 (offset 256MiB), and
	 * GPU has a limit on physical address that it accesses, which must
	 * be below 2GiB.
	 */
	.dma_zone_size	= (SZ_2G - SZ_256M),
	.smp		= smp_ops(imx_smp_ops),
	.map_io		= imx6q_map_io,
	.init_irq	= imx6q_init_irq,
	.init_time	= imx6q_timer_init,
	.init_machine	= imx6q_init_machine,
	.init_late      = imx6q_init_late,
	.dt_compat	= imx6q_dt_compat,
	.restart	= mxc_restart,
#ifdef CONFIG_MX6_CLK_FOR_BOOTUI_TRANS
	.reserve = imx6q_init_reserve,
#endif
MACHINE_END
