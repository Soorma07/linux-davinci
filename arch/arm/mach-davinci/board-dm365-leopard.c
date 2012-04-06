/*
 * Leopard Imaging DM365 Leopard board
 *
 * Derived from: arch/arm/mach-davinci/board-evm.c
 * Copyright (C) 2006 Texas Instruments.
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <linux/config.h>
+#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/major.h>
#include <linux/root_dev.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>

#include <asm/setup.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/arch/irqs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/edma.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/nand.h>
#include <linux/kgdb.h>
#include "clock.h"
#include <asm/arch/cpu.h>
#include <asm/arch/common.h>
#include <asm/arch/clock.h>
#include <asm/arch/mmc.h>

/**************************************************************************
 * Definitions
 **************************************************************************/
static struct plat_serial8250_port serial_platform_data[] = {
	{
		.membase = (char *) IO_ADDRESS(DAVINCI_UART0_BASE),
		.mapbase = (unsigned long) DAVINCI_UART0_BASE,
		.irq = IRQ_UARTINT0,
		.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype = UPIO_MEM,
		.regshift = 2,
		.uartclk = 24000000,
	},
	{
		.membase = (char *) IO_ADDRESS(DM365_UART1_BASE),
		.mapbase = (unsigned long) DM365_UART1_BASE,
		.irq = IRQ_UARTINT1,
		.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype = UPIO_MEM,
		.regshift = 2,
		.uartclk = 24000000,
	},
	{
		.flags = 0
	},
};

static struct platform_device serial_device = {
	.name = "serial8250",
	.id = 0,
	.dev = {
		.platform_data = serial_platform_data,
	},
};

/**************************************************************************
 * Public Functions
 **************************************************************************/
int cpu_type(void)
{
	return MACH_TYPE_DAVINCI_DM365_LEOPARD;
}
EXPORT_SYMBOL(cpu_type);

void davinci_serial_init(struct platform_device *pdev);

static struct resource mmc0_resources[] = {
	[0] = {			/* registers */
		.start = DM365_MMC_SD0_BASE,
		.end = DM365_MMC_SD0_BASE + SZ_1K - 1,
		.flags = IORESOURCE_MEM,
		},
	[1] = {			/* interrupt */
		.start = IRQ_DM3XX_MMCINT0,
		.end = IRQ_DM3XX_MMCINT0,
		.flags = IORESOURCE_IRQ,
		},
	[2] = {			/* dma rx */
		.start = DM365_DMA_MMC0RXEVT,
		.end = DM365_DMA_MMC0RXEVT,
		.flags = IORESOURCE_DMA,
		},
	[3] = {			/* dma tx */
		.start = DM365_DMA_MMC0TXEVT,
		.end = DM365_DMA_MMC0TXEVT,
		.flags = IORESOURCE_DMA,
		},
	[4] = {			/* event queue */
		.start = EVENTQ_3,
		.end = EVENTQ_3,
		.flags = IORESOURCE_DMA,
		},
};

static struct davinci_mmc_platform_data mmc0_platform_data = {
	.mmc_clk = "MMCSDCLK0",
	.rw_threshold = 64,
	.use_4bit_mode = 1,
	.use_8bit_mode		= 0,
	.max_frq		= 50000000,
	.pio_set_dmatrig	= 1,
};

static struct platform_device mmc0_device = {
	.name = "davinci-mmc",
	.id = 0,
	.dev = {
		.platform_data = &mmc0_platform_data,
		},
	.num_resources = ARRAY_SIZE(mmc0_resources),
	.resource = mmc0_resources,
};

static struct resource mmc1_resources[] = {
	[0] = {			/* registers */
		.start = DM365_MMC_SD1_BASE,
		.end = DM365_MMC_SD1_BASE + SZ_1K - 1,
		.flags = IORESOURCE_MEM,
		},
	[1] = {			/* interrupt */
		.start = IRQ_DM3XX_MMCINT1,
		.end = IRQ_DM3XX_MMCINT1,
		.flags = IORESOURCE_IRQ,
		},
	[2] = {			/* dma rx */
		.start = DM365_DMA_MMC1RXEVT,
		.end = DM365_DMA_MMC1RXEVT,
		.flags = IORESOURCE_DMA,
		},
	[3] = {			/* dma tx */
		.start = DM365_DMA_MMC1TXEVT,
		.end = DM365_DMA_MMC1TXEVT,
		.flags = IORESOURCE_DMA,
		},
	[4] = {			/* event queue */
		.start = EVENTQ_3,
		.end = EVENTQ_3,
		.flags = IORESOURCE_DMA,
		},
};

static struct davinci_mmc_platform_data mmc1_platform_data = {
	.mmc_clk = "MMCSDCLK1",
	.rw_threshold = 64,
	.use_4bit_mode = 1,
	.use_8bit_mode		= 0,
	.max_frq		= 50000000,
	.pio_set_dmatrig	= 1,

};

static struct platform_device mmc1_device = {
	.name = "davinci-mmc",
	.id = 1,
	.dev = {
		.platform_data = &mmc1_platform_data,
		},
	.num_resources = ARRAY_SIZE(mmc1_resources),
	.resource = mmc1_resources,
};

/*
 * The NAND partition map used by UBL/U-Boot is a function of the NAND block
 * size.  We support NAND components with either a 128KB or 256KB block size.
*/
#ifdef CONFIG_DAVINCI_NAND_256KB_BLOCKS
#define NAND_BLOCK_SIZE	(SZ_256K)
#else
#define NAND_BLOCK_SIZE	(SZ_128K)
#endif

/* flash bbt decriptors */
static uint8_t nand_davinci_bbt_pattern[] = { 'B', 'b', 't', '0' };
static uint8_t nand_davinci_mirror_pattern[] = { '1', 't', 'b', 'B' };

static struct nand_bbt_descr nand_davinci_bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
	| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs = 2,
	.len = 4,
	.veroffs = 16,
	.maxblocks = 4,
	.pattern = nand_davinci_bbt_pattern
};

static struct nand_bbt_descr nand_davinci_bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
	| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs = 2,
	.len = 4,
	.veroffs = 16,
	.maxblocks = 4,
	.pattern = nand_davinci_mirror_pattern
};

static struct nand_davinci_platform_data nand_data = {
	.options = 0,
	.ecc_mode = NAND_ECC_HW_SYNDROME,
	.cle_mask = 0x10,
	.ale_mask = 0x08,
	.bbt_td = &nand_davinci_bbt_main_descr,
	.bbt_md = &nand_davinci_bbt_mirror_descr,
};

static struct resource nand_resources[] = {
	[0] = {
		/* First memory resource is AEMIF control registers */
		.start = DM365_ASYNC_EMIF_CNTRL_BASE,
		.end = DM365_ASYNC_EMIF_CNTRL_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
		},
	[1] = {
		/* Second memory resource is NAND I/O window */
		.start = DAVINCI_ASYNC_EMIF_DATA_CE0_BASE,
		.end = DAVINCI_ASYNC_EMIF_DATA_CE0_BASE + SZ_16K - 1,
		.flags = IORESOURCE_MEM,
		},
};

static struct platform_device nand_device = {
	.name = "nand_davinci",
	.id = 0,
	.dev = {
		.platform_data = &nand_data
		},
	.num_resources = ARRAY_SIZE(nand_resources),
	.resource = nand_resources,
};

static struct resource rtc_resources[] = {
	[0] = {
		/* registers */
		.start = DM365_RTC_BASE,
		.end = DM365_RTC_BASE + SZ_1K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		/* interrupt */
		.start = IRQ_DM365_RTCINT,
		.end = IRQ_DM365_RTCINT,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device rtc_device = {
	.name = "rtc_davinci_dm365",
	.id = 0,
	.num_resources = ARRAY_SIZE(rtc_resources),
	.resource = rtc_resources,
};

static struct platform_device *dm365_leopard_devices[] __initdata = {
	&serial_device,
	&mmc0_device,
	&mmc1_device,
	&nand_device,
	&rtc_device,
};

/* FIQ are pri 0-1; otherwise 2-7, with 7 lowest priority */
static const u8 dm365_default_priorities[DAVINCI_N_AINTC_IRQ] = {
	[IRQ_DM3XX_VPSSINT0]	= 2,
	[IRQ_DM3XX_VPSSINT1]	= 6,
	[IRQ_DM3XX_VPSSINT2]	= 6,
	[IRQ_DM3XX_VPSSINT3]	= 6,
	[IRQ_DM3XX_VPSSINT4]	= 6,
	[IRQ_DM3XX_VPSSINT5]	= 6,
	[IRQ_DM3XX_VPSSINT6]	= 6,
	[IRQ_DM3XX_VPSSINT7]	= 7,
	[IRQ_DM3XX_VPSSINT8]	= 6,
	[IRQ_ASQINT]		= 6,
	[IRQ_DM365_IMXINT0]	= 6,
	[IRQ_DM3XX_IMCOPINT]	= 6,
	[IRQ_USBINT]		= 4,
	[IRQ_DM3XX_RTOINT]	= 4,
	[IRQ_DM3XX_TINT5]	= 7,
	[IRQ_DM3XX_TINT6]	= 7,
	[IRQ_CCINT0]		= 5,	/* dma */
	[IRQ_DM3XX_SPINT1_0]	= 5,	/* dma */
	[IRQ_DM3XX_SPINT1_1]	= 5,	/* dma */
	[IRQ_DM3XX_SPINT2_0]	= 5,	/* dma */
	[IRQ_DM365_PSCINT]	= 7,
	[IRQ_DM3XX_SPINT2_1]	= 7,
	[IRQ_DM3XX_TINT7]	= 4,
	[IRQ_DM3XX_SDIOINT0]	= 7,
	[IRQ_DM365_MBXINT]	= 7,
	[IRQ_DM365_MBRINT]	= 7,
	[IRQ_DM3XX_MMCINT0]	= 7,
	[IRQ_DM3XX_MMCINT1]	= 7,
	[IRQ_DM3XX_PWMINT3]	= 7,
	[IRQ_DM365_DDRINT]	= 7,
	[IRQ_DM365_AEMIFINT]	= 7,
	[IRQ_DM3XX_SDIOINT1]	= 4,
	[IRQ_DM365_TINT0]	= 2,	/* clockevent */
	[IRQ_DM365_TINT1]	= 2,	/* clocksource */
	[IRQ_DM365_TINT2]	= 7,	/* DSP timer */
	[IRQ_DM365_TINT3]	= 7,	/* system tick */
	[IRQ_PWMINT0]		= 7,
	[IRQ_PWMINT1]		= 7,
	[IRQ_DM365_PWMINT2]	= 7,
	[IRQ_DM365_IICINT]	= 3,
	[IRQ_UARTINT0]		= 3,
	[IRQ_UARTINT1]		= 3,
	[IRQ_DM3XX_SPINT0_0]	= 3,
	[IRQ_DM3XX_SPINT0_1]	= 3,
	[IRQ_DM3XX_GPIO0]	= 3,
	[IRQ_DM3XX_GPIO1]	= 7,
	[IRQ_DM3XX_GPIO2]	= 4,
	[IRQ_DM3XX_GPIO3]	= 4,
	[IRQ_DM3XX_GPIO4]	= 7,
	[IRQ_DM3XX_GPIO5]	= 7,
	[IRQ_DM3XX_GPIO6]	= 7,
	[IRQ_DM3XX_GPIO7]	= 7,
	[IRQ_DM3XX_GPIO8]	= 7,
	[IRQ_DM3XX_GPIO9]	= 7,
	[IRQ_DM365_GPIO10]	= 7,
	[IRQ_DM365_GPIO11]	= 7,
	[IRQ_DM365_GPIO12]	= 7,
	[IRQ_DM365_GPIO13]	= 7,
	[IRQ_DM365_GPIO14]	= 7,
	[IRQ_DM365_GPIO15]	= 7,
	[IRQ_DM365_KEYINT]	= 7,
	[IRQ_DM365_COMMTX]	= 7,
	[IRQ_DM365_COMMRX]	= 7,
	[IRQ_EMUINT]		= 7,
};
static void board_init(void)
{
	int val;
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN,
			DAVINCI_DM365_LPSC_VPSSMSTR, 1);
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM365_LPSC_TPCC, 1);
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM365_LPSC_TPTC0, 1);
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM365_LPSC_TPTC1, 1);
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM365_LPSC_TPTC2, 1);
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM365_LPSC_TPTC3, 1);
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM365_LPSC_GPIO, 1);
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM365_LPSC_McBSP, 1);
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM365_LPSC_SPI0, 1);

	val = davinci_readl(DAVINCI_SYSTEM_MODULE_BASE + 0x48) & ~0x7;
	davinci_writel(val, DAVINCI_SYSTEM_MODULE_BASE + 0x48);

	// Enable CLKOUT1 as the clock source for the AIC chip at 24Mhz
	davinci_cfg_reg(DM365_CLKOUT1, PINMUX_RESV);
	val = davinci_readl(DAVINCI_PLL_CNTRL1_BASE + 0x104); // OCSEL 
	val |= 0x10; // Enable divider output
	davinci_writel(val, DAVINCI_PLL_CNTRL1_BASE + 0x104);
	val = davinci_readl(DAVINCI_PLL_CNTRL1_BASE + 0x148); // CKEN 
	val |= 0x2; // Enable OBSCLK 
	davinci_writel(val, DAVINCI_PLL_CNTRL1_BASE + 0x148);

	/* Turn on WatchDog timer LPSC.  Needed for RESET to work */
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN,
			DAVINCI_DM365_LPSC_TIMER2, 1);
	davinci_serial_init(&serial_device);
	val = davinci_readl(0x1c40018);
	val |= 0x3c400; /* For EMAC Rx/Tx and RTC */
	davinci_writel(val, 0x1c40018);
	val = davinci_readl(0x1c4001c);
	val |= 0x3c000;
	davinci_writel(val, 0x1c4001c);
}

static void dm365_setup_pinmux(unsigned int domain, unsigned int id)
{
	switch (id) {
	case DAVINCI_DM365_LPSC_MMC_SD0:
		davinci_cfg_reg(DM365_MMCSD0);
		break;
	case DAVINCI_DM365_LPSC_MMC_SD1:
		davinci_cfg_reg(DM365_SD1_CLK);
		davinci_cfg_reg(DM365_SD1_CMD);
		davinci_cfg_reg(DM365_SD1_DATA3);
		davinci_cfg_reg(DM365_SD1_DATA2);
		davinci_cfg_reg(DM365_SD1_DATA1);
		davinci_cfg_reg(DM365_SD1_DATA0);
		break;
	case DAVINCI_DM365_LPSC_McBSP:
		davinci_cfg_reg(DM365_MCBSP_DX);
		davinci_cfg_reg(DM365_MCBSP_CLKX);
		davinci_cfg_reg(DM365_MCBSP_FSX);
		davinci_cfg_reg(DM365_MCBSP_DR);
		davinci_cfg_reg(DM365_MCBSP_CLKR);
		davinci_cfg_reg(DM365_MCBSP_FSR);
		break;
	case DAVINCI_DM365_LPSC_PWM0:
		davinci_cfg_reg(DM365_PWM0);
		break;
	case DAVINCI_DM365_LPSC_PWM1:
		davinci_cfg_reg(DM365_PWM1);
		break;
	case DAVINCI_DM365_LPSC_PWM2:
		davinci_cfg_reg(DM365_PWM2_G87);
		davinci_cfg_reg(DM365_PWM2_G88);
		davinci_cfg_reg(DM365_PWM2_G89);
		davinci_cfg_reg(DM365_PWM2_G90);
		break;
	case DAVINCI_DM365_LPSC_PWM3:
		davinci_cfg_reg(DM365_PWM3_G80);
		davinci_cfg_reg(DM365_PWM3_G81);
		davinci_cfg_reg(DM365_PWM3_G85);
		davinci_cfg_reg(DM365_PWM3_G86);
		break;
	case DAVINCI_DM365_LPSC_AEMIF:
		davinci_cfg_reg(DM365_EM_CE0);
		davinci_cfg_reg(DM365_EM_D15_8);
		davinci_cfg_reg(DM365_EM_A7);
		davinci_cfg_reg(DM365_EM_A3);
		davinci_cfg_reg(DM365_EM_AR);
		break;
	case DAVINCI_DM365_LPSC_I2C:
		davinci_cfg_reg(DM365_I2C_SDA);
		davinci_cfg_reg(DM365_I2C_SCL);
		break;
	case DAVINCI_DM365_LPSC_UART0:
		davinci_cfg_reg(DM365_UART0_RXD);
		davinci_cfg_reg(DM365_UART0_TXD);
		break;
	case DAVINCI_DM365_LPSC_CPGMAC:
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
		break;
	case DAVINCI_DM365_LPSC_SPI0:
		davinci_cfg_reg(DM365_SPI0_SCLK);
		davinci_cfg_reg(DM365_SPI0_SDO);
		davinci_cfg_reg(DM365_SPI0_SDI);
		davinci_cfg_reg(DM365_SPI0_SDENA0);
		break;
	}
}

static void __init davinci_map_io(void)
{
	davinci_pinmux_setup = dm365_setup_pinmux;
	davinci_irq_priorities = dm365_default_priorities;
	davinci_map_common_io();
	davinci_init_common_hw();

#ifdef CONFIG_KGDB_8250
#define kgdb8250_ttyS 0
	early_serial_setup((struct uart_port *)
			&serial_platform_data[kgdb8250_ttyS]);
	kgdb8250_add_platform_port(kgdb8250_ttyS,
				 &serial_platform_data[kgdb8250_ttyS]);
#endif

	/* Board-specific initialization */
	board_init();
}

static void setup_mmc(void)
{
	unsigned int temp1 = 0;
	void __iomem *pupdctl1 = (void __iomem *) IO_ADDRESS(0x01C4007c);

	temp1 = __raw_readl(pupdctl1);
	temp1 &= ~0x00000400;
	__raw_writel(temp1, pupdctl1);
}

static void setup_i2c(void)
{
	unsigned int temp1 = 0;
	int i;
	void __iomem *pinmux3 = (void __iomem *) IO_ADDRESS(0x01C4000c);

	temp1 = __raw_readl(pinmux3);
	temp1 = temp1 & 0xFF9FFFFF;
	__raw_writel(temp1, pinmux3);

	/* Configure GPIO20 as an output */
	gpio_direction_output(20, 0);

	for (i = 0; i < 20; i++) {
		gpio_set_value(20, 0);
		mdelay(100);
		gpio_set_value(20, 1);
	}
	temp1 = __raw_readl(pinmux3);
	temp1 = temp1 | 0x01400000;
	__raw_writel(temp1, pinmux3);
}

static __init void davinci_init(void)
{
	davinci_gpio_init();

	setup_i2c();
	setup_mmc();

	platform_add_devices(dm365_leopard_devices, ARRAY_SIZE(dm365_leopard_devices));
}

static __init void davinci_dm365_leopard_irq_init(void)
{
	davinci_irq_init();
}

MACHINE_START(DAVINCI_DM365_LEOPARD, "Leopard Board DM365")
	/* Maintainer: RidgeRun <www.ridgerun.com> */
	.phys_io	= IO_PHYS,
	.io_pg_offst	= (io_p2v(IO_PHYS) >> 18) & 0xfffc,
	.boot_params	= (DAVINCI_DDR_BASE + 0x100),
	.map_io		= davinci_map_io,
	.init_irq	= davinci_dm365_leopard_irq_init,
	.timer		= &davinci_timer,
	.init_machine	= davinci_init,
MACHINE_END
