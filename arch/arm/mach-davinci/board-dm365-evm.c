/*
 * TI DaVinci DM365 EVM board support
 *
 * Copyright (C) 2009 Texas Instruments Incorporated
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/i2c/at24.h>
#include <linux/leds.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/slab.h>
#include <linux/mtd/nand.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/mux.h>
#include <mach/dm365.h>
#include <mach/common.h>
#include <mach/i2c.h>
#include <mach/serial.h>
#include <mach/mmc.h>
#include <mach/nand.h>
#include <mach/keyscan.h>

#include <media/tvp7002.h>

static inline int have_imager(void)
{
	/* REVISIT when it's supported, trigger via Kconfig */
	return 0;
}

static inline int have_tvp7002(void)
{
	/* REVISIT when it's supported, trigger via Kconfig */
	return 0;
}

#define DM365_EVM_PHY_ID		"0:01"

static struct at24_platform_data eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
	.setup          = davinci_get_mac_addr,
	.context	= (void *)0x7f00,
};

static struct snd_platform_data dm365_evm_snd_data = {
	.asp_chan_q = EVENTQ_3,
};

static struct i2c_board_info i2c_info[] = {
	{
		I2C_BOARD_INFO("dm365evm_keys", 0x25),
	},
	{
		I2C_BOARD_INFO("24c256", 0x50),
		.platform_data	= &eeprom_info,
	},
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq	= 400	/* kHz */,
	.bus_delay	= 0	/* usec */,
};

static int dm365evm_keyscan_enable(struct device *dev)
{
	return davinci_cfg_reg(DM365_KEYSCAN);
}

static unsigned short dm365evm_keymap[] = {
	KEY_KP2,
	KEY_LEFT,
	KEY_EXIT,
	KEY_DOWN,
	KEY_ENTER,
	KEY_UP,
	KEY_KP1,
	KEY_RIGHT,
	KEY_MENU,
	KEY_RECORD,
	KEY_REWIND,
	KEY_KPMINUS,
	KEY_STOP,
	KEY_FASTFORWARD,
	KEY_KPPLUS,
	KEY_PLAYPAUSE,
	0
};

static struct davinci_ks_platform_data dm365evm_ks_data = {
	.device_enable	= dm365evm_keyscan_enable,
	.keymap		= dm365evm_keymap,
	.keymapsize	= ARRAY_SIZE(dm365evm_keymap),
	.rep		= 1,
	/* Scan period = strobe + interval */
	.strobe		= 0x5,
	.interval	= 0x2,
	.matrix_type	= DAVINCI_KEYSCAN_MATRIX_4X4,
};

static struct davinci_mmc_config dm365evm_mmc_config = {
	//.get_cd		= cpld_mmc_get_cd,
	//.get_ro		= cpld_mmc_get_ro,
	.get_cd		= NULL,  // pretty sure this is a BAD IDEA
	.get_ro		= NULL,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_2,
};

static void dm365evm_emac_configure(void)
{
	/*
	 * EMAC pins are multiplexed with GPIO and UART
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 125 - 127
	 */
	davinci_cfg_reg(DM365_EMAC_TX_EN);
	davinci_cfg_reg(DM365_EMAC_TX_CLK);
	davinci_cfg_reg(DM365_EMAC_COL);
	davinci_cfg_reg(DM365_EMAC_TXD3);
	davinci_cfg_reg(DM365_EMAC_TXD2);
	davinci_cfg_reg(DM365_EMAC_TXD1);
	davinci_cfg_reg(DM365_EMAC_TXD0);
	davinci_cfg_reg(DM365_EMAC_RXD3);
	davinci_cfg_reg(DM365_EMAC_RXD2);
	davinci_cfg_reg(DM365_EMAC_RXD1);
	davinci_cfg_reg(DM365_EMAC_RXD0);
	davinci_cfg_reg(DM365_EMAC_RX_CLK);
	davinci_cfg_reg(DM365_EMAC_RX_DV);
	davinci_cfg_reg(DM365_EMAC_RX_ER);
	davinci_cfg_reg(DM365_EMAC_CRS);
	davinci_cfg_reg(DM365_EMAC_MDIO);
	davinci_cfg_reg(DM365_EMAC_MDCLK);

	/*
	 * EMAC interrupts are multiplexed with GPIO interrupts
	 * Details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 133 - 134
	 */
	davinci_cfg_reg(DM365_INT_EMAC_RXTHRESH);
	davinci_cfg_reg(DM365_INT_EMAC_RXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_TXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_MISCPULSE);
}

static void dm365evm_mmc_configure(void)
{
	/*
	 * MMC/SD pins are multiplexed with GPIO and EMIF
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 118, 128 - 131
	 */
	davinci_cfg_reg(DM365_SD1_CLK);
	davinci_cfg_reg(DM365_SD1_CMD);
	davinci_cfg_reg(DM365_SD1_DATA3);
	davinci_cfg_reg(DM365_SD1_DATA2);
	davinci_cfg_reg(DM365_SD1_DATA1);
	davinci_cfg_reg(DM365_SD1_DATA0);
}

static struct tvp7002_config tvp7002_cfg = {
	.clk_polarity = 0,
	.hs_polarity = 0,
	.vs_polarity = 0,
	.sog_polarity = 0,
	.fid_polarity = 0
};

#define TVP514X_STD_ALL        (V4L2_STD_NTSC | V4L2_STD_PAL)
/* Inputs available at the TVP5146 */
static struct v4l2_input tvp7002_inputs[] = {
	{
		.index = 0,
		.name = "Composite",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = TVP514X_STD_ALL,
	},
	{
		.index = 1,
		.name = "S-Video",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = TVP514X_STD_ALL,
	},
};

/*
 * this is the route info for connecting each input to decoder
 * ouput that goes to vpfe. There is a one to one correspondence
 * with tvp7002_inputs
 */
static struct vpfe_route tvp7002_routes[] = {
	{
		.input = 0, // INPUT_CVBS_VI2B,
		.output = 0, // OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
{
		.input = 0, // INPUT_SVIDEO_VI2C_VI1C,
		.output = 0, // OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
};

static struct vpfe_subdev_info vpfe_sub_devs[] = {
	{
		.name = "tvp7002",
		.grp_id = 0,
		.num_inputs = ARRAY_SIZE(tvp7002_inputs),
		.inputs = tvp7002_inputs,
		.routes = tvp7002_routes,
		.can_route = 1,
		.ccdc_if_params = {
			.if_type = VPFE_BT656,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
		},
		.board_info = {
			I2C_BOARD_INFO("tvp7002", 0x5d),
			.platform_data = &tvp7002_cfg,
		},
	},
};

static struct vpfe_config vpfe_cfg = {
	.num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
	.sub_devs = vpfe_sub_devs,
	.i2c_adapter_id = 1,
	.card_name = "DM365 EVM",
	.ccdc = "ISIF",
};

static void __init evm_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);
	i2c_register_board_info(1, i2c_info, ARRAY_SIZE(i2c_info));
}

static inline int have_leds(void)
{
#ifdef CONFIG_LEDS_CLASS
	return 1;
#else
	return 0;
#endif
}


static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0),
};

static void __init dm365_evm_map_io(void)
{
	/* setup input configuration for VPFE input devices */
	dm365_set_vpfe_config(&vpfe_cfg);
	dm365_init();
}

static struct spi_eeprom at25640 = {
	.byte_len	= SZ_64K / 8,
	.name		= "at25640",
	.page_size	= 32,
	.flags		= EE_ADDR2,
};

static struct spi_board_info dm365_evm_spi_info[] __initconst = {
	{
		.modalias	= "at25",
		.platform_data	= &at25640,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 0,
		.chip_select	= 0,
		.mode		= SPI_MODE_0,
	},
};

static __init void dm365_evm_init(void)
{
	evm_init_i2c();
	davinci_serial_init(&uart_config);

	dm365evm_emac_configure();
	dm365evm_mmc_configure();

	davinci_setup_mmc(0, &dm365evm_mmc_config);

#ifdef CONFIG_SND_DM365_AIC3X_CODEC
	dm365_init_asp(&dm365_evm_snd_data);
#elif defined(CONFIG_SND_DM365_VOICE_CODEC)
	dm365_init_vc(&dm365_evm_snd_data);
#endif
	dm365_init_rtc();
	dm365_init_ks(&dm365evm_ks_data);

	dm365_init_spi0(BIT(0), dm365_evm_spi_info,
			ARRAY_SIZE(dm365_evm_spi_info));
}

MACHINE_START(DAVINCI_DM365_EVM, "DaVinci DM365 EVM")
	.atag_offset	= 0x100,
	.map_io		= dm365_evm_map_io,
	.init_irq	= davinci_irq_init,
	.timer		= &davinci_timer,
	.init_machine	= dm365_evm_init,
	.dma_zone_size	= SZ_128M,
	.restart	= davinci_restart,
MACHINE_END

