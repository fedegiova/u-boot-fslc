// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 */

#include <image.h>
#include <init.h>
#include <net.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/mach-imx/spi.h>
#include <env.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc_imx.h>
#include <miiphy.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <input.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../common/pfuze.h"
#include <usb.h>
#include <usb/ehci-ci.h>
#include <splash.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)
#define SPI_PAD_CTRL_CS (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST | PAD_CTL_PUS_47K_UP)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define I2C_PMIC	1

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)


int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};


static iomux_v3_cfg_t const usdhc3_pads[] = {
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_RST__SD3_RESET  | MUX_PAD_CTRL(USDHC_PAD_CTRL)), /* CD */
};




enum BoardRevision{
    DELTA80_REV1,
    //DELTA80_REV2, //new DDR
    ////DELTA80_REV3, //removed the audio multiplexer
};

static enum BoardRevision get_board_revision(void)
{
    //TODO read the HW version pins
    return DELTA80_REV1;
}


static struct i2c_pads_info mx6q_i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6Q_PAD_KEY_COL3__I2C2_SCL | I2C_PAD,
		.gpio_mode = MX6Q_PAD_KEY_COL3__GPIO4_IO12 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_KEY_ROW3__I2C2_SDA | I2C_PAD,
		.gpio_mode = MX6Q_PAD_KEY_ROW3__GPIO4_IO13 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 13)
	}
};


static iomux_v3_cfg_t const ecspi1_pads[] = {
	IOMUX_PADS(PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D19__GPIO3_IO19  | MUX_PAD_CTRL(SPI_PAD_CTRL_CS)),
};
static void setup_spi(void) //DONE
{
	SETUP_IOMUX_PADS(ecspi1_pads);
}

iomux_v3_cfg_t const di0_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(SPI_PAD_CTRL)),	/* DISP0_CLK */
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02 | MUX_PAD_CTRL(SPI_PAD_CTRL)),		/* DISP0_HSYNC */
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03 | MUX_PAD_CTRL(SPI_PAD_CTRL)),		/* DISP0_VSYNC */
	//DEBUG ROUTING
	IOMUX_PADS(PAD_NANDF_CS2__CCM_CLKO2| MUX_PAD_CTRL(SPI_PAD_CTRL)),/*Debug clk out 2*/
	//IOMUX_PADS(PAD_GPIO_0__CCM_CLKO1   | MUX_PAD_CTRL(SPI_PAD_CTRL)),/*Debug clk out 1 */
};

static void setup_iomux_uart(void) //DONE
{
	SETUP_IOMUX_PADS(uart1_pads);
}

#ifdef CONFIG_FSL_ESDHC_IMX
int board_mmc_get_env_dev(int devno) //TODO
{
	return devno - 1;
}
struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};
int board_mmc_getcd(struct mmc *mmc)
{
    return 1;
}
int board_mmc_init(struct bd_info *bis)
{
	imx_iomux_v3_setup_multiple_pads(
		usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	return fsl_esdhc_initialize(bis, &usdhc_cfg[1]);
}
#endif

int board_phy_config(struct phy_device *phydev)
{
	phydev->autoneg = AUTONEG_DISABLE;
	phydev->speed = SPEED_100;
	phydev->duplex = DUPLEX_FULL;
	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)

struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= NULL,
	.mode	= {
		.name           = "Ampire",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 600,
		.pixclock       = KHZ2PICOS(40000),
		.left_margin    = 40,
		.right_margin   = 112,
		.upper_margin   = 39,
		.lower_margin   = 18,
		.hsync_len      = 48,
		.vsync_len      = 3,
		.sync 			= FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED,
		.flag 			= FB_MODE_IS_DETAILED
	}
} };
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	//SETUP_IOMUX_PADS(di0_pads);

	/* The row below set the PFD0 to 280MHz
	 * which divided by 7 yeld exaclty 40MHz for
	 * the LVDS display.
	 * We have these configuration with the previous release
	 * of u-boot but now I've found that the display works well even
	 * without.
	 * */
	/*DISABLE PFD0*/
	writel(0x1 << 7,ANATOP_BASE_ADDR + 0x104);
	writel(0x3F, ANATOP_BASE_ADDR + 0x108); /*Clear PFD0_FRAC*/
	writel(0x22, ANATOP_BASE_ADDR + 0x104); /*Set to 280MHz */

	/*switch ldb_di0_serial_clock_root to PPL2_PFD0*/
	reg  =  readl(&mxc_ccm->cs2cdr);
	reg &= ~MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK;
	reg |= 1 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET;
	writel(reg,&mxc_ccm->cs2cdr);

	///*set the 1 bit divider at /7 */
	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	///*set the mux for IPU1_DI0_CLOCK_ROOT*/
	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	writel(0x1 << 7,ANATOP_BASE_ADDR + 0x108); /*Enable PFD0*/

	/*enable ipu clock*/
	reg = readl(&mxc_ccm->CCGR3);
	reg |= MXC_CCM_CCGR3_IPU1_IPU_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* Turn on LDB0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_LVDS0_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#ifdef CONFIG_SPLASH_SCREEN
static struct splash_location default_splash_locations[] = {
	{
		.name		= "mmc_raw",
		.storage	= SPLASH_STORAGE_MMC,
		.flags		= SPLASH_STORAGE_RAW,
		.devpart	= "0:0",
		.offset   	= 9*1024*1024,
	},
};

int splash_screen_prepare(void)
{
	return splash_source_load(default_splash_locations,
				  ARRAY_SIZE(default_splash_locations));
}
#endif //CONFIG_SPLASH_SCREEN
#endif //CONFIG_VIDEO_IPUV3


#ifdef CONFIG_USB_EHCI_MX6
static void setup_usb(void)
{
	/*
	 * set daisy chain for otg_pin_id on 6q.
	 * for 6dl, this bit is reserved
	 */
	imx_iomux_set_gpr_register(1, 13, 1, 0);
}
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();

	/*Enable the pull up on the tiwi serial adaptor*/
	SETUP_IOMUX_PAD(PAD_SD2_DAT3__GPIO1_IO12 | MUX_PAD_CTRL(PAD_CTL_PUS_47K_UP| PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED));
	gpio_direction_output(IMX_GPIO_NR(1,12) , 1);

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif
	return 0;
}

int board_init(void) //done
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;


    setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6q_i2c_pad_info1);

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

	return 0;
}

int power_init_board(void)
{
	struct pmic *p;
	unsigned int reg;
	int ret;

	p = pfuze_common_init(I2C_PMIC);
	if (!p)
		return -ENODEV;

	ret = pfuze_mode_init(p, APS_PFM);
	if (ret < 0)
		return ret;

	/* Increase VGEN3 from 2.5 to 2.8V */
	pmic_reg_read(p, PFUZE100_VGEN3VOL, &reg);
	reg &= ~LDO_VOL_MASK;
	reg |= LDOB_2_80V;
	pmic_reg_write(p, PFUZE100_VGEN3VOL, reg);

	/* Increase VGEN5 from 2.8 to 3V */
	pmic_reg_read(p, PFUZE100_VGEN5VOL, &reg);
	reg &= ~LDO_VOL_MASK;
	reg |= LDOB_3_00V;
	pmic_reg_write(p, PFUZE100_VGEN5VOL, reg);

	return 0;
}

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? (IMX_GPIO_NR(3, 19)) : -1;
}
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,	 0},
};
#endif


#ifndef CONFIG_SPL_BUILD
static int do_mark_mtest_complete(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
    unsigned char buf[2];
    int ret;
    if( argc < 2 )
    {
        puts("Missing test result");
        return -1;
    }
    buf[0] = 0xb3;
    buf[1] = argv[1][0] == '1' ? 11 : 10;
    i2c_set_bus_num(CONFIG_SYS_RTC_BUS_NUM);
    ret = i2c_write(CONFIG_SYS_I2C_RTC_ADDR, CONFIG_SYS_BOOTCOUNT_ADDR + 6,
                    CONFIG_BOOTCOUNT_ALEN, buf, 2);
    return ret == 2;
}

U_BOOT_CMD(
       mark_mtest_complete,2,  0,      do_mark_mtest_complete,
       "Save the memory test result in the NVRAM and clear the flag",
       ""
);
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

//#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
//	env_set("board_name", "SABRESD");
//
//	if (is_mx6dqp())
//		env_set("board_rev", "MX6QP");
//	else if (is_mx6dq())
//		env_set("board_rev", "MX6Q");
//	else if (is_mx6sdl())
//		env_set("board_rev", "MX6DL");
//#endif

    //read bootdelay
    unsigned char buf[2];
    i2c_set_bus_num(CONFIG_SYS_RTC_BUS_NUM);
	int ret = i2c_read(CONFIG_SYS_I2C_RTC_ADDR, CONFIG_SYS_BOOTCOUNT_ADDR + 2,
		       CONFIG_BOOTCOUNT_ALEN, buf, 2);
    if( ret == 0 && buf[0] == 0xbd && buf[1])
    {
        env_set("bootdelay","4");
        puts("unlocking bootloader\n");
    }
    //read test command
	ret = i2c_read(CONFIG_SYS_I2C_RTC_ADDR, CONFIG_SYS_BOOTCOUNT_ADDR + 6,
		       CONFIG_BOOTCOUNT_ALEN, buf, 2);
    if( ret == 0 && buf[0] == 0xb3 && buf[1] == 1)
    {
        env_set("bootcmd","run bootcmd_memtest");
        puts("requested memtes\n");
    }

	return 0;
}

int checkboard(void)
{
    const char * rev_str = "<ND>";
    switch(get_board_revision())
    {
    case DELTA80_REV1: rev_str = "rev 1"; break;
    }
	printf("Board: Delta80 %s\n",rev_str);
	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <asm/arch/mx6-ddr.h>
#include <spl.h>
#include <linux/libfdt.h>

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
    return 1; //always start u-boot for now
	//gpio_request(KEY_VOL_UP, "KEY Volume UP");
	//gpio_direction_input(KEY_VOL_UP);

	///* Only enter in Falcon mode if KEY_VOL_UP is pressed */
	//return gpio_get_value(KEY_VOL_UP);
}
#endif

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	//writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x0F0000F3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}


static int delta80r1_dcd_table[] = {
    0x020e0798, 0x000C0000,
    0x020e0758, 0x00000000,
    0x020e0588, 0x00000030,
    0x020e0594, 0x00000030,
    0x020e056c, 0x00000030,
    0x020e0578, 0x00000030,
    0x020e074c, 0x00000030,
    0x020e057c, 0x00000030,
    0x020e058c, 0x00000000,
    0x020e059c, 0x00000030,
    0x020e05a0, 0x00000030,
    0x020e078c, 0x00000030,
    0x020e0750, 0x00020000,
    0x020e05a8, 0x00000030,
    0x020e05b0, 0x00000030,
    0x020e0524, 0x00000030,
    0x020e051c, 0x00000030,
    0x020e0518, 0x00000030,
    0x020e050c, 0x00000030,
    0x020e05b8, 0x00000030,
    0x020e05c0, 0x00000030,
    0x020e0774, 0x00020000,
    0x020e0784, 0x00000030,
    0x020e0788, 0x00000030,
    0x020e0794, 0x00000030,
    0x020e079c, 0x00000030,
    0x020e07a0, 0x00000030,
    0x020e07a4, 0x00000030,
    0x020e07a8, 0x00000030,
    0x020e0748, 0x00000030,
    0x020e05ac, 0x00000030,
    0x020e05b4, 0x00000030,
    0x020e0528, 0x00000030,
    0x020e0520, 0x00000030,
    0x020e0514, 0x00000030,
    0x020e0510, 0x00000030,
    0x020e05bc, 0x00000030,
    0x020e05c4, 0x00000030,

    0x021b0800, 0xa1390003,

    /*Write leveling*/
    0x021b080c, 0x001F001F,
    0x021b0810, 0x001F001F,
    0x021b480c, 0x001F001F,
    0x021b4810, 0x001F001F,
    /*Calibration*/
    0x021b083c, 0x43400354,
    0x021b0840, 0x03300330,
    0x021b483c, 0x434C035C,
    0x021b4840, 0x03400300,
    0x021b0848, 0x342C3032,
    0x021b4848, 0x2C2E2C3C,
    0x021b0850, 0x32363E32,
    0x021b4850, 0x4634463E,

    0x021b081c, 0x33333333,
    0x021b0820, 0x33333333,
    0x021b0824, 0x33333333,
    0x021b0828, 0x33333333,
    0x021b481c, 0x33333333,
    0x021b4820, 0x33333333,
    0x021b4824, 0x33333333,
    0x021b4828, 0x33333333,

    0x021b08b8, 0x00000800,
    0x021b48b8, 0x00000800,

    0x021b0004, 0x00020036,
    0x021b0008, 0x09444040,
    0x021b000c, 0x555A7975,
    0x021b0010, 0xFF538F64,
    0x021b0014, 0x01FF00DB,
    0x021b0018, 0x00001740,

    0x021b001c, 0x00008000,
    0x021b002c, 0x000026d2,
    0x021b0030, 0x005A1023,
    0x021b0040, 0x00000027,

    /*MDCTL*/
    0x021b0000, 0x821A0000,

    0x021b001c, 0x04088032,
    0x021b001c, 0x00008033,
    0x021b001c, 0x00048031,
    0x021b001c, 0x09408030,
    0x021b001c, 0x04008040,

    0x021b0020, 0x00005800,
    0x021b0818, 0x00011117,
    0x021b4818, 0x00011117,
    0x021b0004, 0x00025576,
    0x021b0404, 0x00011006,
    0x021b001c, 0x00000000,


   /*
    * Setup CCM_CCOSR register as follows (debug clock divisor):
    *
    * cko1_en  = 1	   --> CKO1 enabled
    * cko1_div = 111  --> divide by 8
    * cko1_sel = 1011 --> ahb_clk_root
    *
    * This sets CKO1 at ahb_clk_root/8 = 132/8 = 16.5 MHz
    */
    /*0x020c4060, 0x000000fb,*/
    /**
    *	Clock debug: (NAND_CS2 and PCIE_ENABLE on the board)
    *	IPU_1_DI0 		-> OUT2
    *	LDB_DI0_SERIAL	-> OUT1
    */
    //0x020c4060, 0x1F800F7
};

static void ddr_init(int *table, int size)
{
	int i;

	for (i = 0; i < size / 2 ; i++)
		writel(table[2 * i + 1], table[2 * i]);
}

static void spl_dram_init(void)
{
    switch( get_board_revision() )
    {
    case DELTA80_REV1:
		ddr_init(delta80r1_dcd_table, ARRAY_SIZE(delta80r1_dcd_table));
        break;
    }
}

void board_init_f(ulong dummy)
{
    /* Keep PERST pin DOWN! to avoid briking GSM modules*/
	SETUP_IOMUX_PAD(PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL( PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED));
	gpio_direction_output(IMX_GPIO_NR(1,5) , 0);

	/* DDR initialization */
	spl_dram_init();

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif

//#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
    if(!strcmp(name, "delta80")) return 0;

	return -1;
}
//#endif
