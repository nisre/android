/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/spinlock.h>
#include <linux/syscore_ops.h>
#include "pinctrl-msm.h"

/* config translations */
#define drv_str_to_rval(drv)	((drv >> 1) - 1)
#define rval_to_drv_str(val)	((val + 1) << 1)
#define dir_to_inout_val(dir)	(dir << 1)
#define inout_val_to_dir(val)	(val >> 1)
#define rval_to_pull(val)	((val > 2) ? 1 : val)
#define TLMMV4_NO_PULL		0
#define TLMMV4_PULL_DOWN	1
#define TLMMV4_PULL_UP		3
/* GP PIN TYPE REG MASKS */
#define TLMMV4_GP_DRV_SHFT		6
#define TLMMV4_GP_DRV_MASK		0x7
#define TLMMV4_GP_PULL_SHFT		0
#define TLMMV4_GP_PULL_MASK		0x3
#define TLMMV4_GP_DIR_SHFT		9
#define TLMMV4_GP_DIR_MASK		1
#define TLMMV4_GP_FUNC_SHFT		2
#define TLMMV4_GP_FUNC_MASK		0xF
#define GPIO_OUT_BIT			1
#define GPIO_IN_BIT			0
#define GPIO_OE_BIT			9
/* SDC1 PIN TYPE REG MASKS */
#define TLMMV4_SDC1_CLK_DRV_SHFT	6
#define TLMMV4_SDC1_CLK_DRV_MASK	0x7
#define TLMMV4_SDC1_DATA_DRV_SHFT	0
#define TLMMV4_SDC1_DATA_DRV_MASK	0x7
#define TLMMV4_SDC1_CMD_DRV_SHFT	3
#define TLMMV4_SDC1_CMD_DRV_MASK	0x7
#define TLMMV4_SDC1_CLK_PULL_SHFT	13
#define TLMMV4_SDC1_CLK_PULL_MASK	0x3
#define TLMMV4_SDC1_DATA_PULL_SHFT	9
#define TLMMV4_SDC1_DATA_PULL_MASK	0x3
#define TLMMV4_SDC1_CMD_PULL_SHFT	11
#define TLMMV4_SDC1_CMD_PULL_MASK	0x3
#define TLMMV3_SDC1_RCLK_PULL_SHFT	15
#define TLMMV3_SDC1_RCLK_PULL_MASK	0x3
/* SDC2 PIN TYPE REG MASKS */
#define TLMMV4_SDC2_CLK_DRV_SHFT	6
#define TLMMV4_SDC2_CLK_DRV_MASK	0x7
#define TLMMV4_SDC2_DATA_DRV_SHFT	0
#define TLMMV4_SDC2_DATA_DRV_MASK	0x7
#define TLMMV4_SDC2_CMD_DRV_SHFT	3
#define TLMMV4_SDC2_CMD_DRV_MASK	0x7
#define TLMMV4_SDC2_CLK_PULL_SHFT	14
#define TLMMV4_SDC2_CLK_PULL_MASK	0x3
#define TLMMV4_SDC2_DATA_PULL_SHFT	9
#define TLMMV4_SDC2_DATA_PULL_MASK	0x3
#define TLMMV4_SDC2_CMD_PULL_SHFT	11
#define TLMMV4_SDC2_CMD_PULL_MASK	0x3
/* TLMM V4 IRQ REG fields */
#define INTR_ENABLE_BIT		0
#define	INTR_POL_CTL_BIT	1
#define	INTR_DECT_CTL_BIT	2
#define	INTR_RAW_STATUS_EN_BIT	4
#define	INTR_TARGET_PROC_BIT	5
#define	INTR_DIR_CONN_EN_BIT	8
#define INTR_STATUS_BIT		0
#define DC_POLARITY_BIT		8

/* Target processors for TLMM pin based interrupts */
#define INTR_TARGET_PROC_APPS(core_id)	((core_id) << INTR_TARGET_PROC_BIT)
#define TLMMV4_APPS_ID_DEFAULT	4
#define INTR_TARGET_PROC_NONE	(7 << INTR_TARGET_PROC_BIT)
/* Interrupt flag bits */
#define DC_POLARITY_HI		BIT(DC_POLARITY_BIT)
#define INTR_POL_CTL_HI		BIT(INTR_POL_CTL_BIT)
#define INTR_DECT_CTL_LEVEL	(0 << INTR_DECT_CTL_BIT)
#define INTR_DECT_CTL_POS_EDGE	(1 << INTR_DECT_CTL_BIT)
#define INTR_DECT_CTL_NEG_EDGE	(2 << INTR_DECT_CTL_BIT)
#define INTR_DECT_CTL_DUAL_EDGE	(3 << INTR_DECT_CTL_BIT)
#define INTR_DECT_CTL_MASK	(3 << INTR_DECT_CTL_BIT)

#define TLMMV4_GP_INOUT_BIT		1
#define TLMMV4_GP_OUT			BIT(TLMMV4_GP_INOUT_BIT)
#define TLMMV4_GP_IN			0

#define gc_to_pintype(gc) \
		container_of(gc, struct msm_pintype_info, gc)
#define ic_to_pintype(ic) \
		((struct msm_pintype_info *)ic->pinfo)
#define pintype_get_gc(pinfo)	(&pinfo->gc)
#define pintype_get_ic(pinfo)	(pinfo->irq_chip)

/* SDC Pin type register offsets */
#define TLMMV4_SDC_OFFSET		0x0010A000
#define TLMMV4_SDC1_CFG(base)		(base)
#define TLMMV4_SDC2_CFG(base)		(TLMMV4_SDC1_CFG(base) - 0x1000)

/* GP pin type register offsets */
#define TLMMV4_GP_CFG(base, pin)        (base + 0x0 + 0x1000 * (pin))
#define TLMMV4_GP_INOUT(base, pin)      (base + 0x4 + 0x1000 * (pin))
#define TLMMV4_GP_INTR_CFG(base, pin)		(base + 0x8 + 0x1000 * (pin))
#define TLMMV4_GP_INTR_STATUS(base, pin)	(base + 0xc + 0x1000 * (pin))

/* QDSD Pin type register offsets */
#define TLMMV4_QDSD_OFFSET		0x0019C000
#define TLMMV4_QDSD_PULL_MASK	0x3
#define TLMMV4_QDSD_PULL_OFFSET	0x3
#define TLMMV4_QDSD_CONFIG_WIDTH	0x5
#define TLMMV4_QDSD_DRV_MASK	0x7

//yangjq, 20140629, Add to save gpio irq's to log when resume
extern int msm_show_resume_irq_mask;

struct msm_sdc_regs {
	unsigned int offset;
	unsigned long pull_mask;
	unsigned long pull_shft;
	unsigned long drv_mask;
	unsigned long drv_shft;
};

static struct msm_sdc_regs sdc_regs[] = {
	/* SDC1 CLK */
	{
		.offset = 0,
		.pull_mask = TLMMV4_SDC1_CLK_PULL_MASK,
		.pull_shft = TLMMV4_SDC1_CLK_PULL_SHFT,
		.drv_mask = TLMMV4_SDC1_CLK_DRV_MASK,
		.drv_shft = TLMMV4_SDC1_CLK_DRV_SHFT,
	},
	/* SDC1 CMD */
	{
		.offset = 0,
		.pull_mask = TLMMV4_SDC1_CMD_PULL_MASK,
		.pull_shft = TLMMV4_SDC1_CMD_PULL_SHFT,
		.drv_mask = TLMMV4_SDC1_CMD_DRV_MASK,
		.drv_shft = TLMMV4_SDC1_CMD_DRV_SHFT,
	},
	/* SDC1 DATA */
	{
		.offset = 0,
		.pull_mask = TLMMV4_SDC1_DATA_PULL_MASK,
		.pull_shft = TLMMV4_SDC1_DATA_PULL_SHFT,
		.drv_mask = TLMMV4_SDC1_DATA_DRV_MASK,
		.drv_shft = TLMMV4_SDC1_DATA_DRV_SHFT,
	},
	/* SDC1 RCLK */
	{
		.offset = 0,
		.pull_mask = TLMMV3_SDC1_RCLK_PULL_MASK,
		.pull_shft = TLMMV3_SDC1_RCLK_PULL_SHFT,
	},
	/* SDC2 CLK */
	{
		.offset = 0x1000,
		.pull_mask = TLMMV4_SDC2_CLK_PULL_MASK,
		.pull_shft = TLMMV4_SDC2_CLK_PULL_SHFT,
		.drv_mask = TLMMV4_SDC2_CLK_DRV_MASK,
		.drv_shft = TLMMV4_SDC2_CLK_DRV_SHFT,
	},
	/* SDC2 CMD */
	{
		.offset = 0x1000,
		.pull_mask = TLMMV4_SDC2_CMD_PULL_MASK,
		.pull_shft = TLMMV4_SDC2_CMD_PULL_SHFT,
		.drv_mask = TLMMV4_SDC2_CMD_DRV_MASK,
		.drv_shft = TLMMV4_SDC2_CMD_DRV_SHFT,
	},
	/* SDC2 DATA */
	{
		.offset = 0x1000,
		.pull_mask = TLMMV4_SDC2_DATA_PULL_MASK,
		.pull_shft = TLMMV4_SDC2_DATA_PULL_SHFT,
		.drv_mask = TLMMV4_SDC2_DATA_DRV_MASK,
		.drv_shft = TLMMV4_SDC2_DATA_DRV_SHFT,
	},
};

static int msm_tlmm_v4_sdc_cfg(uint pin_no, unsigned long *config,
						void __iomem *reg_base,
						bool write)
{
	unsigned int val, id, data;
	u32 mask, shft;
	void __iomem *cfg_reg;

	if (pin_no >= ARRAY_SIZE(sdc_regs))
		return -EINVAL;

	cfg_reg = reg_base - sdc_regs[pin_no].offset;
	id = pinconf_to_config_param(*config);
	val = readl_relaxed(cfg_reg);
	/* Get mask and shft values for this config type */
	switch (id) {
	case PIN_CONFIG_BIAS_DISABLE:
		mask = sdc_regs[pin_no].pull_mask;
		shft = sdc_regs[pin_no].pull_shft;
		data = TLMMV4_NO_PULL;
		if (!write) {
			val >>= shft;
			val &= mask;
			data = rval_to_pull(val);
		}
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		mask = sdc_regs[pin_no].pull_mask;
		shft = sdc_regs[pin_no].pull_shft;
		data = TLMMV4_PULL_DOWN;
		if (!write) {
			val >>= shft;
			val &= mask;
			data = rval_to_pull(val);
		}
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		mask = sdc_regs[pin_no].pull_mask;
		shft = sdc_regs[pin_no].pull_shft;
		data = TLMMV4_PULL_UP;
		if (!write) {
			val >>= shft;
			val &= mask;
			data = rval_to_pull(val);
		}
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		mask = sdc_regs[pin_no].drv_mask;
		shft = sdc_regs[pin_no].drv_shft;
		if (write) {
			data = pinconf_to_config_argument(*config);
			data = drv_str_to_rval(data);
		} else {
			val >>= shft;
			val &= mask;
			data = rval_to_drv_str(val);
		}
		break;
	default:
		return -EINVAL;
	};

	if (write) {
		val &= ~(mask << shft);
		val |= (data << shft);
		writel_relaxed(val, cfg_reg);
	} else
		*config = pinconf_to_config_packed(id, data);
	return 0;
}

static void msm_tlmm_v4_sdc_set_reg_base(void __iomem **ptype_base,
							void __iomem *tlmm_base)
{
	*ptype_base = tlmm_base + TLMMV4_SDC_OFFSET;
}

static int msm_tlmm_v4_qdsd_cfg(uint pin_no, unsigned long *config,
						void __iomem *reg_base,
						bool write)
{
	unsigned int val, id, data;
	u32 mask, shft;
	void __iomem *cfg_reg;

	cfg_reg = reg_base;
	id = pinconf_to_config_param(*config);
	val = readl_relaxed(cfg_reg);
	/* Get mask and shft values for this config type */
	switch (id) {
	case PIN_CONFIG_BIAS_DISABLE:
		mask = TLMMV4_QDSD_PULL_MASK;
		shft = pin_no * TLMMV4_QDSD_CONFIG_WIDTH
					+ TLMMV4_QDSD_PULL_OFFSET;
		data = TLMMV4_NO_PULL;
		if (!write) {
			val >>= shft;
			data = val & mask;
		}
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		mask = TLMMV4_QDSD_PULL_MASK;
		shft = pin_no * TLMMV4_QDSD_CONFIG_WIDTH
					+ TLMMV4_QDSD_PULL_OFFSET;
		data = TLMMV4_PULL_DOWN;
		if (!write) {
			val >>= shft;
			data = val & mask;
		}
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		mask = TLMMV4_QDSD_PULL_MASK;
		shft = pin_no * TLMMV4_QDSD_CONFIG_WIDTH
					+ TLMMV4_QDSD_PULL_OFFSET;
		data = TLMMV4_PULL_UP;
		if (!write) {
			val >>= shft;
			data = val & mask;
		}
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		mask = TLMMV4_QDSD_DRV_MASK;
		shft = pin_no * TLMMV4_QDSD_CONFIG_WIDTH;
		if (write) {
			data = pinconf_to_config_argument(*config);
		} else {
			val >>= shft;
			data = val & mask;
		}
		break;
	default:
		return -EINVAL;
	};

	if (write) {
		val &= ~(mask << shft);
		/* QDSD software override bit */
		val |= ((data << shft) | BIT(31));
		writel_relaxed(val, cfg_reg);
	} else {
		*config = pinconf_to_config_packed(id, data);
	}
	return 0;
}

static void msm_tlmm_v4_qdsd_set_reg_base(void __iomem **ptype_base,
							void __iomem *tlmm_base)
{
	*ptype_base = tlmm_base + TLMMV4_QDSD_OFFSET;
}

/* yangjq, 20130515, Add sysfs for gpio's debug, START */
void * tlmm_reg_base = NULL;
/* yangjq, 20130515, Add sysfs for gpio's debug, END */

static int msm_tlmm_v4_gp_cfg(uint pin_no, unsigned long *config,
						void *reg_base, bool write)
{
	unsigned int val, id, data, inout_val;
	u32 mask = 0, shft = 0;
	void __iomem *inout_reg = NULL;
	void __iomem *cfg_reg = TLMMV4_GP_CFG(reg_base, pin_no);

	id = pinconf_to_config_param(*config);
	val = readl_relaxed(cfg_reg);
	/* Get mask and shft values for this config type */
	switch (id) {
	case PIN_CONFIG_BIAS_DISABLE:
		mask = TLMMV4_GP_PULL_MASK;
		shft = TLMMV4_GP_PULL_SHFT;
		data = TLMMV4_NO_PULL;
		if (!write) {
			val >>= shft;
			val &= mask;
			data = rval_to_pull(val);
		}
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		mask = TLMMV4_GP_PULL_MASK;
		shft = TLMMV4_GP_PULL_SHFT;
		data = TLMMV4_PULL_DOWN;
		if (!write) {
			val >>= shft;
			val &= mask;
			data = rval_to_pull(val);
		}
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		mask = TLMMV4_GP_PULL_MASK;
		shft = TLMMV4_GP_PULL_SHFT;
		data = TLMMV4_PULL_UP;
		if (!write) {
			val >>= shft;
			val &= mask;
			data = rval_to_pull(val);
		}
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		mask = TLMMV4_GP_DRV_MASK;
		shft = TLMMV4_GP_DRV_SHFT;
		if (write) {
			data = pinconf_to_config_argument(*config);
			data = drv_str_to_rval(data);
		} else {
			val >>= shft;
			val &= mask;
			data = rval_to_drv_str(val);
		}
		break;
	case PIN_CONFIG_OUTPUT:
		mask = TLMMV4_GP_DIR_MASK;
		shft = TLMMV4_GP_DIR_SHFT;
		inout_reg = TLMMV4_GP_INOUT(reg_base, pin_no);
		if (write) {
			data = pinconf_to_config_argument(*config);
			inout_val = dir_to_inout_val(data);
			writel_relaxed(inout_val, inout_reg);
			data = mask;
		} else {
			inout_val = readl_relaxed(inout_reg);
			data = inout_val_to_dir(inout_val);
		}
		break;
	default:
		return -EINVAL;
	};

	if (write) {
		val &= ~(mask << shft);
		val |= (data << shft);
		writel_relaxed(val, cfg_reg);
	} else
		*config = pinconf_to_config_packed(id, data);
	return 0;
}

static void msm_tlmm_v4_gp_fn(uint pin_no, u32 func, void *reg_base,
								bool enable)
{
	unsigned int val;
	void __iomem *cfg_reg = TLMMV4_GP_CFG(reg_base, pin_no);
	val = readl_relaxed(cfg_reg);
	val &= ~(TLMMV4_GP_FUNC_MASK << TLMMV4_GP_FUNC_SHFT);
	if (enable)
		val |= (func << TLMMV4_GP_FUNC_SHFT);
	writel_relaxed(val, cfg_reg);
}

static void msm_tlmm_v4_gp_set_reg_base(void __iomem **ptype_base,
						void __iomem *tlmm_base)
{
	*ptype_base = tlmm_base;
}

/* GPIO CHIP */
static int msm_tlmm_v4_gp_get(struct gpio_chip *gc, unsigned offset)
{
	struct msm_pintype_info *pinfo = gc_to_pintype(gc);
	void __iomem *inout_reg = TLMMV4_GP_INOUT(pinfo->reg_base, offset);

	return readl_relaxed(inout_reg) & BIT(GPIO_IN_BIT);
}

static void msm_tlmm_v4_gp_set(struct gpio_chip *gc, unsigned offset, int val)
{
	struct msm_pintype_info *pinfo = gc_to_pintype(gc);
	void __iomem *inout_reg = TLMMV4_GP_INOUT(pinfo->reg_base, offset);

	writel_relaxed(val ? BIT(GPIO_OUT_BIT) : 0, inout_reg);
}

static int msm_tlmm_v4_gp_dir_in(struct gpio_chip *gc, unsigned offset)
{
	unsigned int val;
	struct msm_pintype_info *pinfo = gc_to_pintype(gc);
	void __iomem *cfg_reg = TLMMV4_GP_CFG(pinfo->reg_base, offset);

	val = readl_relaxed(cfg_reg);
	val &= ~BIT(GPIO_OE_BIT);
	writel_relaxed(val, cfg_reg);
	return 0;
}

static int msm_tlmm_v4_gp_dir_out(struct gpio_chip *gc, unsigned offset,
								int val)
{
	struct msm_pintype_info *pinfo = gc_to_pintype(gc);
	void __iomem *cfg_reg = TLMMV4_GP_CFG(pinfo->reg_base, offset);

	msm_tlmm_v4_gp_set(gc, offset, val);
	val = readl_relaxed(cfg_reg);
	val |= BIT(GPIO_OE_BIT);
	writel_relaxed(val, cfg_reg);
	return 0;
}

static int msm_tlmm_v4_gp_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct msm_pintype_info *pinfo = gc_to_pintype(gc);
	struct msm_tlmm_irq_chip *ic = pintype_get_ic(pinfo);
	return irq_create_mapping(ic->domain, offset);
}

/* Irq reg ops */
static void msm_tlmm_v4_set_intr_status(struct msm_tlmm_irq_chip *ic,
								unsigned pin)
{
	void __iomem *status_reg = TLMMV4_GP_INTR_STATUS(ic->chip_base, pin);
	writel_relaxed(0, status_reg);
}

static int msm_tlmm_v4_get_intr_status(struct msm_tlmm_irq_chip *ic,
								unsigned pin)
{
	void __iomem *status_reg = TLMMV4_GP_INTR_STATUS(ic->chip_base, pin);
	return readl_relaxed(status_reg) & BIT(INTR_STATUS_BIT);
}

static void msm_tlmm_v4_set_intr_cfg_enable(struct msm_tlmm_irq_chip *ic,
								unsigned pin,
								int enable)
{
	unsigned int val;
	void __iomem *cfg_reg = TLMMV4_GP_INTR_CFG(ic->chip_base, pin);

	val = readl_relaxed(cfg_reg);
	if (enable) {
		val &= ~BIT(INTR_DIR_CONN_EN_BIT);
		val |= BIT(INTR_ENABLE_BIT);
	} else
		val &= ~BIT(INTR_ENABLE_BIT);
	writel_relaxed(val, cfg_reg);
}

static int msm_tlmm_v4_get_intr_cfg_enable(struct msm_tlmm_irq_chip *ic,
								unsigned pin)
{
	void __iomem *cfg_reg = TLMMV4_GP_INTR_CFG(ic->chip_base, pin);
	return readl_relaxed(cfg_reg) & BIT(INTR_ENABLE_BIT);
}

static void msm_tlmm_v4_set_intr_cfg_type(struct msm_tlmm_irq_chip *ic,
							struct irq_data *d,
							unsigned int type)
{
	unsigned cfg;
	void __iomem *cfg_reg = TLMMV4_GP_INTR_CFG(ic->chip_base,
							(irqd_to_hwirq(d)));
	/*
	 * RAW_STATUS_EN is left on for all gpio irqs. Due to the
	 * internal circuitry of TLMM, toggling the RAW_STATUS
	 * could cause the INTR_STATUS to be set for EDGE interrupts.
	 */
	cfg = BIT(INTR_RAW_STATUS_EN_BIT) | INTR_TARGET_PROC_APPS(ic->apps_id);
	writel_relaxed(cfg, cfg_reg);
	cfg &= ~INTR_DECT_CTL_MASK;
	if (type == IRQ_TYPE_EDGE_RISING)
		cfg |= INTR_DECT_CTL_POS_EDGE;
	else if (type == IRQ_TYPE_EDGE_FALLING)
		cfg |= INTR_DECT_CTL_NEG_EDGE;
	else if (type == IRQ_TYPE_EDGE_BOTH)
		cfg |= INTR_DECT_CTL_DUAL_EDGE;
	else
		cfg |= INTR_DECT_CTL_LEVEL;

	if (type & IRQ_TYPE_LEVEL_LOW)
		cfg &= ~INTR_POL_CTL_HI;
	else
		cfg |= INTR_POL_CTL_HI;

	writel_relaxed(cfg, cfg_reg);
	/*
	 * Sometimes it might take a little while to update
	 * the interrupt status after the RAW_STATUS is enabled
	 * We clear the interrupt status before enabling the
	 * interrupt in the unmask call-back.
	 */
	udelay(5);
}

static irqreturn_t msm_tlmm_v4_gp_handle_irq(int irq,
						struct  msm_tlmm_irq_chip *ic)
{
	unsigned long i;
	unsigned int virq = 0;
	struct irq_desc *desc = irq_to_desc(irq);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct msm_pintype_info *pinfo = ic_to_pintype(ic);
	struct gpio_chip *gc = pintype_get_gc(pinfo);

	chained_irq_enter(chip, desc);
	for_each_set_bit(i, ic->enabled_irqs, ic->num_irqs) {
		dev_dbg(ic->dev, "hwirq in bit mask %d\n", (unsigned int)i);
		if (msm_tlmm_v4_get_intr_status(ic, i)) {
			dev_dbg(ic->dev, "hwirw %d fired\n", (unsigned int)i);
			virq = msm_tlmm_v4_gp_to_irq(gc, i);
			if (!virq) {
				dev_dbg(ic->dev, "invalid virq\n");
				return IRQ_NONE;
			}
			generic_handle_irq(virq);
		}

	}
	chained_irq_exit(chip, desc);
	return IRQ_HANDLED;
}

static void msm_tlmm_v4_irq_ack(struct irq_data *d)
{
	struct msm_tlmm_irq_chip *ic = irq_data_get_irq_chip_data(d);

	msm_tlmm_v4_set_intr_status(ic, irqd_to_hwirq(d));
	mb();
}

static void msm_tlmm_v4_irq_mask(struct irq_data *d)
{
	unsigned long irq_flags;
	struct msm_tlmm_irq_chip *ic = irq_data_get_irq_chip_data(d);

	spin_lock_irqsave(&ic->irq_lock, irq_flags);
	msm_tlmm_v4_set_intr_cfg_enable(ic, irqd_to_hwirq(d), 0);
	__clear_bit(irqd_to_hwirq(d), ic->enabled_irqs);
	mb();
	spin_unlock_irqrestore(&ic->irq_lock, irq_flags);
	if (ic->irq_chip_extn->irq_mask)
		ic->irq_chip_extn->irq_mask(d);
}

static void msm_tlmm_v4_irq_unmask(struct irq_data *d)
{
	unsigned long irq_flags;
	struct msm_tlmm_irq_chip *ic = irq_data_get_irq_chip_data(d);

	spin_lock_irqsave(&ic->irq_lock, irq_flags);
	__set_bit(irqd_to_hwirq(d), ic->enabled_irqs);
	if (!msm_tlmm_v4_get_intr_cfg_enable(ic, irqd_to_hwirq(d))) {
		msm_tlmm_v4_set_intr_status(ic, irqd_to_hwirq(d));
		msm_tlmm_v4_set_intr_cfg_enable(ic, irqd_to_hwirq(d), 1);
		mb();
	}
	spin_unlock_irqrestore(&ic->irq_lock, irq_flags);
	if (ic->irq_chip_extn->irq_unmask)
		ic->irq_chip_extn->irq_unmask(d);
}

static void msm_tlmm_v4_irq_disable(struct irq_data *d)
{
	struct msm_tlmm_irq_chip *ic = irq_data_get_irq_chip_data(d);
	if (ic->irq_chip_extn->irq_disable)
		ic->irq_chip_extn->irq_disable(d);
}

static int msm_tlmm_v4_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
	unsigned long irq_flags;
	unsigned int pin = irqd_to_hwirq(d);
	struct msm_tlmm_irq_chip *ic = irq_data_get_irq_chip_data(d);


	spin_lock_irqsave(&ic->irq_lock, irq_flags);

	if (flow_type & IRQ_TYPE_EDGE_BOTH) {
		__irq_set_handler_locked(d->irq, handle_edge_irq);
		if ((flow_type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH)
			__set_bit(pin, ic->dual_edge_irqs);
		else
			__clear_bit(pin, ic->dual_edge_irqs);
	} else {
		__irq_set_handler_locked(d->irq, handle_level_irq);
		__clear_bit(pin, ic->dual_edge_irqs);
	}

	msm_tlmm_v4_set_intr_cfg_type(ic, d, flow_type);

	mb();
	spin_unlock_irqrestore(&ic->irq_lock, irq_flags);

	if (ic->irq_chip_extn->irq_set_type)
		ic->irq_chip_extn->irq_set_type(d, flow_type);

	return 0;
}

static int msm_tlmm_v4_irq_set_wake(struct irq_data *d, unsigned int on)
{
	unsigned int pin = irqd_to_hwirq(d);
	struct msm_tlmm_irq_chip *ic = irq_data_get_irq_chip_data(d);

	if (on) {
		if (bitmap_empty(ic->wake_irqs, ic->num_irqs))
			irq_set_irq_wake(ic->irq, 1);
		set_bit(pin, ic->wake_irqs);
	} else {
		clear_bit(pin, ic->wake_irqs);
		if (bitmap_empty(ic->wake_irqs, ic->num_irqs))
			irq_set_irq_wake(ic->irq, 0);
	}

	if (ic->irq_chip_extn->irq_set_wake)
		ic->irq_chip_extn->irq_set_wake(d, on);

	return 0;
}

static struct lock_class_key msm_tlmm_irq_lock_class;

static int msm_tlmm_v4_irq_map(struct irq_domain *h, unsigned int virq,
					irq_hw_number_t hw)
{
	struct msm_tlmm_irq_chip *ic = h->host_data;

	irq_set_lockdep_class(virq, &msm_tlmm_irq_lock_class);
	irq_set_chip_data(virq, ic);
	irq_set_chip_and_handler(virq, &ic->chip,
					handle_level_irq);
	set_irq_flags(virq, IRQF_VALID);
	return 0;
}

/*
 * irq domain callbacks for interrupt controller.
 */
static const struct irq_domain_ops msm_tlmm_v4_gp_irqd_ops = {
	.map	= msm_tlmm_v4_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};

static struct irq_chip mpm_tlmm_irq_extn;

static struct msm_tlmm_irq_chip msm_tlmm_v4_gp_irq = {
	.irq_chip_extn = &mpm_tlmm_irq_extn,
	.chip = {
		.name		= "msm_tlmm_v4_irq",
		.irq_mask	= msm_tlmm_v4_irq_mask,
		.irq_unmask	= msm_tlmm_v4_irq_unmask,
		.irq_ack	= msm_tlmm_v4_irq_ack,
		.irq_set_type	= msm_tlmm_v4_irq_set_type,
		.irq_set_wake	= msm_tlmm_v4_irq_set_wake,
		.irq_disable	= msm_tlmm_v4_irq_disable,
	},
	.apps_id = TLMMV4_APPS_ID_DEFAULT,
	.domain_ops = &msm_tlmm_v4_gp_irqd_ops,
	.handler = msm_tlmm_v4_gp_handle_irq,
};

/* Power management core operations */

static int msm_tlmm_v4_gp_irq_suspend(void)
{
	unsigned long irq_flags;
	unsigned long i;
	struct msm_tlmm_irq_chip *ic = &msm_tlmm_v4_gp_irq;
	int num_irqs = ic->num_irqs;

	spin_lock_irqsave(&ic->irq_lock, irq_flags);
	for_each_set_bit(i, ic->enabled_irqs, num_irqs)
		msm_tlmm_v4_set_intr_cfg_enable(ic, i, 0);

	for_each_set_bit(i, ic->wake_irqs, num_irqs)
		msm_tlmm_v4_set_intr_cfg_enable(ic, i, 1);
	mb();
	spin_unlock_irqrestore(&ic->irq_lock, irq_flags);
	return 0;
}

//yangjq, 20140629, Add to save gpio irq's to log when resume, START
void msm_gpio_show_resume_irq(void)
{
	extern int save_irq_wakeup_gpio(int irq, int gpio);
	unsigned long irq_flags;
	unsigned long i;
	unsigned int irq = 0;
	int intstat;
	struct msm_tlmm_irq_chip *ic = &msm_tlmm_v4_gp_irq;

	if (!msm_show_resume_irq_mask)
		return;

	spin_lock_irqsave(&ic->irq_lock, irq_flags);
	for_each_set_bit(i, ic->wake_irqs, ic->num_irqs) {
	//for_each_set_bit(i, ic->enabled_irqs, ic->num_irqs) {
		intstat = msm_tlmm_v4_get_intr_status(ic, i);
		//printk("%s(), intstat=%d, i=%lu\n", __func__, intstat, i);
		if (intstat) {
			struct irq_desc *desc;
			const char *name = "null";
			struct msm_pintype_info *pinfo = ic_to_pintype(ic);
			struct gpio_chip *gc = pintype_get_gc(pinfo);
			irq = msm_tlmm_v4_gp_to_irq(gc, i);
			if (!irq)
				break;
			desc = irq_to_desc(irq);
			if (desc == NULL)
				name = "stray irq";
			else if (desc->action && desc->action->name)
				name = desc->action->name;

			pr_warning("%s: %d triggered %s\n",
					__func__, irq, name);
			save_irq_wakeup_gpio(irq, i);
		}
	}
	spin_unlock_irqrestore(&ic->irq_lock, irq_flags);
}
//yangjq, 20140629, Add to save gpio irq's to log when resume, END

static void msm_tlmm_v4_gp_irq_resume(void)
{
	unsigned long irq_flags;
	unsigned long i;
	struct msm_tlmm_irq_chip *ic = &msm_tlmm_v4_gp_irq;
	int num_irqs = ic->num_irqs;

	//yangjq, 20140629, Add to save gpio irq's to log when resume
	msm_gpio_show_resume_irq();

	spin_lock_irqsave(&ic->irq_lock, irq_flags);
	for_each_set_bit(i, ic->wake_irqs, num_irqs)
		msm_tlmm_v4_set_intr_cfg_enable(ic, i, 0);

	for_each_set_bit(i, ic->enabled_irqs, num_irqs)
		msm_tlmm_v4_set_intr_cfg_enable(ic, i, 1);
	mb();
	spin_unlock_irqrestore(&ic->irq_lock, irq_flags);
}

static struct syscore_ops msm_tlmm_v4_irq_syscore_ops = {
	.suspend = msm_tlmm_v4_gp_irq_suspend,
	.resume = msm_tlmm_v4_gp_irq_resume,
};

#ifdef CONFIG_USE_PINCTRL_IRQ
int msm_tlmm_v4_of_irq_init(struct device_node *controller,
						struct irq_chip *chip_extn)
{
	int ret, num_irqs, apps_id;
	struct msm_tlmm_irq_chip *ic = &msm_tlmm_v4_gp_irq;

	ret = of_property_read_u32(controller, "num_irqs", &num_irqs);
	if (ret) {
		WARN(1, "Cannot get numirqs from device tree\n");
		return ret;
	}
	ret = of_property_read_u32(controller, "apps_id", &apps_id);
	if (!ret) {
		pr_info("processor id specified, in device tree %d\n", apps_id);
		ic->apps_id = apps_id;
	}
	ic->num_irqs = num_irqs;
	ic->domain = irq_domain_add_linear(controller, ic->num_irqs,
						ic->domain_ops,
						ic);
	if (IS_ERR(ic->domain))
			return -ENOMEM;
	ic->irq_chip_extn = chip_extn;
	return 0;
}
#endif

static int msm_tlmm_v4_gp_irq_init(int irq, struct msm_pintype_info *pinfo,
						struct device *tlmm_dev)
{
	int num_irqs;
	struct msm_tlmm_irq_chip *ic = pinfo->irq_chip;

	if (!ic->domain)
		return 0;

	num_irqs = ic->num_irqs;
	ic->enabled_irqs = devm_kzalloc(tlmm_dev, sizeof(unsigned long)
					* BITS_TO_LONGS(num_irqs), GFP_KERNEL);
	if (IS_ERR(ic->enabled_irqs)) {
		dev_err(tlmm_dev, "Unable to allocate enabled irqs bitmap\n");
		return PTR_ERR(ic->enabled_irqs);
	}
	ic->dual_edge_irqs = devm_kzalloc(tlmm_dev, sizeof(unsigned long)
					* BITS_TO_LONGS(num_irqs), GFP_KERNEL);
	if (IS_ERR(ic->dual_edge_irqs)) {
		dev_err(tlmm_dev, "Unable to allocate dual edge irqs bitmap\n");
		return PTR_ERR(ic->dual_edge_irqs);
	}
	ic->wake_irqs = devm_kzalloc(tlmm_dev, sizeof(unsigned long)
					* BITS_TO_LONGS(num_irqs), GFP_KERNEL);
	if (IS_ERR(ic->wake_irqs)) {
		dev_err(tlmm_dev, "Unable to allocate wake irqs bitmap\n");
		return PTR_ERR(ic->wake_irqs);
	}
	spin_lock_init(&ic->irq_lock);
	ic->chip_base = pinfo->reg_base;
	ic->irq = irq;
	ic->dev = tlmm_dev;
	ic->num_irqs = pinfo->num_pins;
	ic->pinfo = pinfo;
	register_syscore_ops(&msm_tlmm_v4_irq_syscore_ops);
	return 0;
}

static irqreturn_t msm_tlmm_v4_handle_irq(int irq, void *data)
{
	int i, num_pintypes;
	struct msm_pintype_info *pintypes, *pintype;
	struct msm_tlmm_irq_chip *ic;
	struct msm_tlmm_desc *tlmm_desc = (struct msm_tlmm_desc *)data;
	irqreturn_t ret = IRQ_NONE;

	pintypes = tlmm_desc->pintypes;
	num_pintypes = tlmm_desc->num_pintypes;
	for (i = 0; i < num_pintypes; i++) {
		pintype = &pintypes[i];
		if (!pintype->irq_chip)
			continue;
		ic = pintype->irq_chip;
		if (!ic->node)
			continue;
		ret = ic->handler(irq, ic);
		if (ret != IRQ_HANDLED)
			break;
	}
	return ret;
}

/* yangjq, 20130515, Add sysfs for gpio's debug, START */
#define TLMM_NUM_GPIO 122

#define HAL_OUTPUT_VAL(config)    \
         (((config)&0x40000000)>>30)

static int tlmm_get_cfg(unsigned gpio, unsigned* cfg)
{
	unsigned flags;

	if(tlmm_reg_base == NULL)
		return -1;
	BUG_ON(gpio >= TLMM_NUM_GPIO);
	//printk("%s(), gpio=%d, addr=0x%08x\n", __func__, gpio, (unsigned int)GPIO_CONFIG(gpio));

#if 0
	flags = ((GPIO_DIR(config) << 9) & (0x1 << 9)) |
		((GPIO_DRVSTR(config) << 6) & (0x7 << 6)) |
		((GPIO_FUNC(config) << 2) & (0xf << 2)) |
		((GPIO_PULL(config) & 0x3));
#else
	flags = readl_relaxed(TLMMV4_GP_CFG(tlmm_reg_base, gpio));
#endif
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	*cfg = GPIO_CFG(gpio, (flags >> 2) & 0xf, (flags >> 9) & 0x1, flags & 0x3, (flags >> 6) & 0x7);

	return 0;
}

int tlmm_set_config(unsigned config)
{
	unsigned int flags;
	unsigned gpio = GPIO_PIN(config);
	void __iomem *cfg_reg = TLMMV4_GP_CFG(tlmm_reg_base, gpio );
	
	if(tlmm_reg_base == NULL)
		return -1;
	if (gpio > TLMM_NUM_GPIO)
		return -EINVAL;

	printk("%s(), %d,gpio=%d\n", __func__, __LINE__, gpio);

	config = (config & ~0x40000000);
	flags = readl_relaxed(cfg_reg);
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	
	flags = ((GPIO_DIR(config) & TLMMV4_GP_DIR_MASK) << TLMMV4_GP_DIR_SHFT) |
		((GPIO_DRVSTR(config) & TLMMV4_GP_DRV_MASK) << TLMMV4_GP_DRV_SHFT) |
		((GPIO_FUNC(config) & TLMMV4_GP_FUNC_MASK) << TLMMV4_GP_FUNC_SHFT) |
		((GPIO_PULL(config) & TLMMV4_GP_PULL_MASK));

	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	writel_relaxed(flags, cfg_reg);

#if 0
	/*set func*/
	cfg_reg = TLMMV4_GP_CFG(tlmm_reg_base, gpio);
	flags = readl_relaxed(cfg_reg);
	flags &= ~(TLMMV4_GP_FUNC_MASK << TLMMV4_GP_FUNC_SHFT);
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	
	flags |= (GPIO_FUNC(config) << TLMMV4_GP_FUNC_SHFT);
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	writel_relaxed(flags, cfg_reg);

	/* set DIR */
	cfg_reg = TLMMV4_GP_CFG(tlmm_reg_base, gpio);
	flags = readl_relaxed(cfg_reg);
	if (GPIO_DIR(config))
	{
		flags |= BIT(GPIO_OE_BIT);
	}
	else
	{
		flags &= ~BIT(GPIO_OE_BIT);
	}
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	writel_relaxed(flags, cfg_reg);

	/* set PULL */
	flags = readl_relaxed(cfg_reg);
	flags |= GPIO_PULL(config) & 0x3;
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	writel_relaxed(flags, cfg_reg);

	/* set DRVSTR */
	flags = readl_relaxed(cfg_reg);
	flags |= drv_str_to_rval(GPIO_DRVSTR(config));
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	writel_relaxed(flags, cfg_reg);
#endif
	return 0;
}
static int tlmm_dump_cfg(char* buf,unsigned gpio, unsigned cfg, int output_val)
{
	static char* drvstr_str[] = { "2", "4", "6", "8", "10", "12", "14", "16" }; // mA
	static char*   pull_str[] = { "N", "D", "K", "U" };	 // "NO_PULL", "PULL_DOWN", "KEEPER", "PULL_UP"
	static char*    dir_str[] = { "I", "O" }; // "Input", "Output"	 
	char func_str[20];
	
	char* p = buf;

	int drvstr   = GPIO_DRVSTR(cfg);
	int pull     = GPIO_PULL(cfg);
	int dir      = GPIO_DIR(cfg);
	int func     = GPIO_FUNC(cfg);

	//printk("%s(), drvstr=%d, pull=%d, dir=%d, func=%d\n", __func__, drvstr, pull, dir, func);
	sprintf(func_str, "%d", func);

	p += sprintf(p, "%d:0x%x %s%s%s%s", gpio, cfg,
			func_str, pull_str[pull], dir_str[dir], drvstr_str[drvstr]);

	p += sprintf(p, " = %d", output_val);

	p += sprintf(p, "\n");	
			
	return p - buf;		
}

static int tlmm_dump_header(char* buf)
{
	char* p = buf;
	p += sprintf(p, "bit   0~3: function. (0 is GPIO)\n");
	p += sprintf(p, "bit  4~13: gpio number\n");
	p += sprintf(p, "bit    14: 0: input, 1: output\n");
	p += sprintf(p, "bit 15~16: pull: NO_PULL, PULL_DOWN, KEEPER, PULL_UP\n");
	p += sprintf(p, "bit 17~20: driver strength. \n");
	p += sprintf(p, "0:GPIO\n");
	p += sprintf(p, "N:NO_PULL  D:PULL_DOWN  K:KEEPER  U:PULL_UP\n");
	p += sprintf(p, "I:Input  O:Output\n");
	p += sprintf(p, "2:2, 4, 6, 8, 10, 12, 14, 16 mA (driver strength)\n\n");
	return p - buf;
}

static int tlmm_get_inout(unsigned gpio)
{
	void __iomem *inout_reg = TLMMV4_GP_INOUT(tlmm_reg_base, gpio);

	if(tlmm_reg_base == NULL)
		return -1;
	return readl_relaxed(inout_reg) & BIT(GPIO_IN_BIT);
}

void tlmm_set_inout(unsigned gpio, unsigned val)
{
	void __iomem *inout_reg = TLMMV4_GP_INOUT(tlmm_reg_base, gpio);

	if(tlmm_reg_base == NULL)
		return;
	writel_relaxed(val ? BIT(GPIO_OUT_BIT) : 0, inout_reg);
}

int tlmm_dump_info(char* buf, int tlmm_num)
{
	unsigned i;
	char* p = buf;
	unsigned cfg;
	int output_val = 0;

	if(tlmm_num >= 0 && tlmm_num < TLMM_NUM_GPIO) {
		tlmm_get_cfg(tlmm_num, &cfg);
		output_val = tlmm_get_inout(tlmm_num);
			
		p += tlmm_dump_cfg(p, tlmm_num, cfg, output_val);
	} else {
		p += tlmm_dump_header(p);
		p += sprintf(p, "Standard Format: gpio_num  function  pull  direction  strength [output_value]\n");
		p += sprintf(p, "Shortcut Format: gpio_num  output_value\n");
		p += sprintf(p, " e.g.  'echo  20 0 D O 2 1'  ==> set pin 20 as GPIO output and the output = 1 \n");
		p += sprintf(p, " e.g.  'echo  20 1'  ==> set output gpio pin 20 output = 1 \n");
	
		printk("%s(), %d, TLMM_BASE=%x\n", __func__, __LINE__, (unsigned int)tlmm_reg_base);
		for(i = 0; i < TLMM_NUM_GPIO; ++i) {
			tlmm_get_cfg(i, &cfg);
			output_val = tlmm_get_inout(i);
			
			p += tlmm_dump_cfg(p, i, cfg, output_val);
		}
		printk("%s(), %d\n", __func__, __LINE__);
		p+= sprintf(p, "(%d)\n", p - buf); // only for debug reference	
	}
	return p - buf;	
}

/* save tlmm config before sleep */
static unsigned before_sleep_fetched;
static unsigned before_sleep_configs[TLMM_NUM_GPIO];
void tlmm_before_sleep_save_configs(void)
{
	unsigned i;

	//only save tlmm configs when it has been fetched
	if (!before_sleep_fetched)
		return;

	printk("%s(), before_sleep_fetched=%d\n", __func__, before_sleep_fetched);
	before_sleep_fetched = false;
	for(i = 0; i < TLMM_NUM_GPIO; ++i) {
		unsigned cfg;
		int output_val = 0;

		tlmm_get_cfg(i, &cfg);
		output_val = tlmm_get_inout(i);

		before_sleep_configs[i] = cfg | (output_val << 30);
	}
}

int tlmm_before_sleep_dump_info(char* buf)
{
	unsigned i;
	char* p = buf;

	p += sprintf(p, "tlmm_before_sleep:\n");
	if (!before_sleep_fetched) {
		before_sleep_fetched = true;

		p += tlmm_dump_header(p);
		
		for(i = 0; i < TLMM_NUM_GPIO; ++i) {
			unsigned cfg;
			int output_val = 0;

			cfg = before_sleep_configs[i];
			output_val = HAL_OUTPUT_VAL(cfg);
			//cfg &= ~0x40000000;
			p += tlmm_dump_cfg(p, i, cfg, output_val);
		}
		p+= sprintf(p, "(%d)\n", p - buf); // only for debug reference
	}
	return p - buf;	
}

/* set tlmms config before sleep */
static unsigned before_sleep_table_enabled;
static unsigned before_sleep_table_configs[TLMM_NUM_GPIO];
void tlmm_before_sleep_set_configs(void)
{
	int res;
	unsigned i;

	//only set tlmms before sleep when it's enabled
	if (!before_sleep_table_enabled)
		return;

	printk("%s(), before_sleep_table_enabled=%d\n", __func__, before_sleep_table_enabled);
	for(i = 0; i < TLMM_NUM_GPIO; ++i) {
		unsigned cfg;
		int gpio;
		int dir;
		int func;
		int output_val = 0;

		cfg = before_sleep_table_configs[i];

		gpio = GPIO_PIN(cfg);
		if(gpio != i)//(cfg & ~0x20000000) == 0 || 
			continue;

		output_val = HAL_OUTPUT_VAL(cfg);
		//Clear the output value
		//cfg &= ~0x40000000;
		dir = GPIO_DIR(cfg);
		func = GPIO_FUNC(cfg);

		printk("%s(), [%d]: 0x%x\n", __func__, i, cfg);
		res = tlmm_set_config(cfg & ~0x40000000);
		if(res < 0) {
			printk("Error: Config failed.\n");
		}
		
		if((func == 0) && (dir == 1)) // gpio output
			tlmm_set_inout(i, output_val);
	}
}

int tlmm_before_sleep_table_set_cfg(unsigned gpio, unsigned cfg)
{
	//BUG_ON(gpio >= TLMM_NUM_GPIO && GPIO_PIN(cfg) != 0xff);
	if (gpio >= TLMM_NUM_GPIO && gpio != 255 && gpio != 256) {
		printk("gpio >= TLMM_NUM_GPIO && gpio != 255 && gpio != 256!\n");
		return -1;
	}

	if(gpio < TLMM_NUM_GPIO)
	{
		before_sleep_table_configs[gpio] = cfg;// | 0x20000000
		before_sleep_table_enabled = true;
	}
	else if(gpio == 255)
		before_sleep_table_enabled = true;
	else if(gpio == 256)
		before_sleep_table_enabled = false;

	return 0;
}

int tlmm_before_sleep_table_dump_info(char* buf)
{
	unsigned i;
	char* p = buf;

	p += tlmm_dump_header(p);
	p += sprintf(p, "Format: gpio_num  function  pull  direction  strength [output_value]\n");
	p += sprintf(p, " e.g.  'echo  20 0 D O 2 1'  ==> set pin 20 as GPIO output and the output = 1 \n");
	p += sprintf(p, " e.g.  'echo  20'  ==> disable pin 20's setting \n");
	p += sprintf(p, " e.g.  'echo  255'  ==> enable sleep table's setting \n");
	p += sprintf(p, " e.g.  'echo  256'  ==> disable sleep table's setting \n");

	for(i = 0; i < TLMM_NUM_GPIO; ++i) {
		unsigned cfg;
		int output_val = 0;

		cfg = before_sleep_table_configs[i];
		output_val = HAL_OUTPUT_VAL(cfg);
		//cfg &= ~0x40000000;
		p += tlmm_dump_cfg(p, i, cfg, output_val);
	}
	p+= sprintf(p, "(%d)\n", p - buf); // only for debug reference
	return p - buf;
}
/* yangjq, 20130515, Add sysfs for gpio's debug, END */


static struct msm_pintype_info tlmm_v4_pininfo[] = {
	{
		.prg_cfg = msm_tlmm_v4_gp_cfg,
		.prg_func = msm_tlmm_v4_gp_fn,
		.set_reg_base = msm_tlmm_v4_gp_set_reg_base,
		.reg_base = NULL,
		.prop_name = "qcom,pin-type-gp",
		.name = "gp",
		.gc = {
				.label		  = "msm_tlmm_v4_gpio",
				.direction_input  = msm_tlmm_v4_gp_dir_in,
				.direction_output = msm_tlmm_v4_gp_dir_out,
				.get              = msm_tlmm_v4_gp_get,
				.set              = msm_tlmm_v4_gp_set,
				.to_irq           = msm_tlmm_v4_gp_to_irq,
		},
		.init_irq = msm_tlmm_v4_gp_irq_init,
		.irq_chip = &msm_tlmm_v4_gp_irq,
	},
	{
		.prg_cfg = msm_tlmm_v4_sdc_cfg,
		.set_reg_base = msm_tlmm_v4_sdc_set_reg_base,
		.reg_base = NULL,
		.prop_name = "qcom,pin-type-sdc",
		.name = "sdc",
	},
	{
		.prg_cfg = msm_tlmm_v4_qdsd_cfg,
		.set_reg_base = msm_tlmm_v4_qdsd_set_reg_base,
		.reg_base = NULL,
		.prop_name = "qcom,pin-type-qdsd",
		.name = "qdsd",
	}
};

struct msm_tlmm_pintype tlmm_v4_pintypes = {
	.num_entries = ARRAY_SIZE(tlmm_v4_pininfo),
	.pintype_info = tlmm_v4_pininfo,
};

static const struct of_device_id msm_tlmm_v4_dt_match[] = {
	{ .compatible = "qcom,msm-tlmm-v4",
		.data = &tlmm_v4_pintypes, },
	{},
};
MODULE_DEVICE_TABLE(of, msm_tlmm_v4_dt_match);

static int msm_tlmm_v4_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	const struct msm_tlmm_pintype *pinfo;
	struct msm_tlmm_desc *tlmm_desc;
	int irq, ret;
	struct resource *res;
	struct device_node *node = pdev->dev.of_node;

	match = of_match_node(msm_tlmm_v4_dt_match, node);
	if (IS_ERR(match))
		return PTR_ERR(match);
	pinfo = match->data;
	tlmm_desc = devm_kzalloc(&pdev->dev, sizeof(*tlmm_desc), GFP_KERNEL);
	if (!tlmm_desc) {
		dev_err(&pdev->dev, "Alloction failed for tlmm desc\n");
		return -ENOMEM;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot find IO resource\n");
		return -ENOENT;
	}
	tlmm_desc->base = devm_ioremap(&pdev->dev, res->start,
							resource_size(res));
	if (IS_ERR(tlmm_desc->base))
		return PTR_ERR(tlmm_desc->base);
/* yangjq, 20130515, Add sysfs for gpio's debug, START */
	tlmm_reg_base = tlmm_desc->base;
	//printk("%s(), %d, TLMM_BASE=%x\n", __func__, __LINE__, (unsigned int)tlmm_reg_base);
/* yangjq, 20130515, Add sysfs for gpio's debug, END */
	tlmm_desc->irq = -EINVAL;
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res) {
		irq = res->start;
		ret = devm_request_irq(&pdev->dev, irq, msm_tlmm_v4_handle_irq,
							IRQF_TRIGGER_HIGH,
							dev_name(&pdev->dev),
							tlmm_desc);
		if (ret) {
			dev_err(&pdev->dev, "register for irq failed\n");
			return ret;
		}
		tlmm_desc->irq = irq;
	}
	tlmm_desc->pintypes = pinfo->pintype_info;
	tlmm_desc->num_pintypes = pinfo->num_entries;
	return msm_pinctrl_probe(pdev, tlmm_desc);
}

static struct platform_driver msm_tlmm_v4_drv = {
	.probe		= msm_tlmm_v4_probe,
	.driver = {
		.name	= "msm-tlmmv4-pinctrl",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(msm_tlmm_v4_dt_match),
	},
};

static int __init msm_tlmm_v4_drv_register(void)
{
	return platform_driver_register(&msm_tlmm_v4_drv);
}
postcore_initcall(msm_tlmm_v4_drv_register);

static void __exit msm_tlmm_v4_drv_unregister(void)
{
	platform_driver_unregister(&msm_tlmm_v4_drv);
}
module_exit(msm_tlmm_v4_drv_unregister);

MODULE_LICENSE("GPLv2");
