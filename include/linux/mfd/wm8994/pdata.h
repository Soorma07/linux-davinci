/*
 * include/linux/mfd/wm8994/pdata.h -- Platform data for WM8994
 *
 * Copyright 2009 Wolfson Microelectronics PLC.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __MFD_WM8994_PDATA_H__
#define __MFD_WM8994_PDATA_H__

#define WM8994_NUM_LDO   2
#define WM8994_NUM_GPIO 11

struct wm8994_ldo_pdata {
	/** GPIOs to enable regulator, 0 or less if not available */
	int enable;

	const char *supply;
	const struct regulator_init_data *init_data;
};

#define WM8994_CONFIGURE_GPIO 0x10000

#define WM8994_DRC_REGS 5
#define WM8994_EQ_REGS  20
#define WM8958_MBC_CUTOFF_REGS 20
#define WM8958_MBC_COEFF_REGS  48
#define WM8958_MBC_COMBINED_REGS 56
#define WM8958_VSS_HPF_REGS 2
#define WM8958_VSS_REGS 148
#define WM8958_ENH_EQ_REGS 32

/**
 * DRC configurations are specified with a label and a set of register
 * values to write (the enable bits will be ignored).  At runtime an
 * enumerated control will be presented for each DRC block allowing
 * the user to choose the configration to use.
 *
 * Configurations may be generated by hand or by using the DRC control
 * panel provided by the WISCE - see  http://www.wolfsonmicro.com/wisce/
 * for details.
 */
struct wm8994_drc_cfg {
        const char *name;
        u16 regs[WM8994_DRC_REGS];
};

/**
 * ReTune Mobile configurations are specified with a label, sample
 * rate and set of values to write (the enable bits will be ignored).
 *
 * Configurations are expected to be generated using the ReTune Mobile
 * control panel in WISCE - see http://www.wolfsonmicro.com/wisce/
 */
struct wm8994_retune_mobile_cfg {
        const char *name;
        unsigned int rate;
        u16 regs[WM8994_EQ_REGS];
};

/**
 * Multiband compressor configurations are specified with a label and
 * two sets of values to write.  Configurations are expected to be
 * generated using the multiband compressor configuration panel in
 * WISCE - see http://www.wolfsonmicro.com/wisce/
 */
struct wm8958_mbc_cfg {
	const char *name;
	u16 cutoff_regs[WM8958_MBC_CUTOFF_REGS];
	u16 coeff_regs[WM8958_MBC_COEFF_REGS];

	/* Coefficient layout when using MBC+VSS firmware */
	u16 combined_regs[WM8958_MBC_COMBINED_REGS];
};

/**
 * VSS HPF configurations are specified with a label and two values to
 * write.  Configurations are expected to be generated using the
 * multiband compressor configuration panel in WISCE - see
 * http://www.wolfsonmicro.com/wisce/
 */
struct wm8958_vss_hpf_cfg {
	const char *name;
	u16 regs[WM8958_VSS_HPF_REGS];
};

/**
 * VSS configurations are specified with a label and array of values
 * to write.  Configurations are expected to be generated using the
 * multiband compressor configuration panel in WISCE - see
 * http://www.wolfsonmicro.com/wisce/
 */
struct wm8958_vss_cfg {
	const char *name;
	u16 regs[WM8958_VSS_REGS];
};

/**
 * Enhanced EQ configurations are specified with a label and array of
 * values to write.  Configurations are expected to be generated using
 * the multiband compressor configuration panel in WISCE - see
 * http://www.wolfsonmicro.com/wisce/
 */
struct wm8958_enh_eq_cfg {
	const char *name;
	u16 regs[WM8958_ENH_EQ_REGS];
};

/**
 * Microphone detection rates, used to tune response rates and power
 * consumption for WM8958/WM1811 microphone detection.
 *
 * @sysclk: System clock rate to use this configuration for.
 * @idle: True if this configuration should use when no accessory is detected,
 *        false otherwise.
 * @start: Value for MICD_BIAS_START_TIME register field (not shifted).
 * @rate: Value for MICD_RATE register field (not shifted).
 */
struct wm8958_micd_rate {
	int sysclk;
	bool idle;
	int start;
	int rate;
};

struct wm8994_pdata {
	int gpio_base;

	/**
	 * Default values for GPIOs if non-zero, WM8994_CONFIGURE_GPIO
	 * can be used for all zero values.
	 */
	int gpio_defaults[WM8994_NUM_GPIO];

	struct wm8994_ldo_pdata ldo[WM8994_NUM_LDO];

	int irq_base;  /** Base IRQ number for WM8994, required for IRQs */

        int num_drc_cfgs;
        struct wm8994_drc_cfg *drc_cfgs;

        int num_retune_mobile_cfgs;
        struct wm8994_retune_mobile_cfg *retune_mobile_cfgs;

	int num_mbc_cfgs;
	struct wm8958_mbc_cfg *mbc_cfgs;

	int num_vss_cfgs;
	struct wm8958_vss_cfg *vss_cfgs;

	int num_vss_hpf_cfgs;
	struct wm8958_vss_hpf_cfg *vss_hpf_cfgs;

	int num_enh_eq_cfgs;
	struct wm8958_enh_eq_cfg *enh_eq_cfgs;

	int num_micd_rates;
	struct wm8958_micd_rate *micd_rates;

        /* LINEOUT can be differential or single ended */
        unsigned int lineout1_diff:1;
        unsigned int lineout2_diff:1;

        /* Common mode feedback */
        unsigned int lineout1fb:1;
        unsigned int lineout2fb:1;

	/* IRQ for microphone detection if brought out directly as a
	 * signal.
	 */
	int micdet_irq;

        /* WM8994 microphone biases: 0=0.9*AVDD1 1=0.65*AVVD1 */
        unsigned int micbias1_lvl:1;
        unsigned int micbias2_lvl:1;

        /* WM8994 jack detect threashold levels, see datasheet for values */
        unsigned int jd_scthr:2;
        unsigned int jd_thr:2;

	/* WM8958 microphone bias configuration */
	int micbias[2];

	/* WM8958 microphone detection ranges */
	u16 micd_lvl_sel;

	/* Disable the internal pull downs on the LDOs if they are
	 * always driven (eg, connected to an always on supply or
	 * GPIO that always drives an output.  If they float power
	 * consumption will rise.
	 */
	bool ldo_ena_always_driven;

	/*
	 * SPKMODE must be pulled internally by the device on this
	 * system.
	 */
	bool spkmode_pu;
};

#endif
