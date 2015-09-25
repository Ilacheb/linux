/*
 * Copyright (c) 2010 Sascha Hauer <s.hauer@pengutronix.de>
 * Copyright (C) 2005-2009 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/export.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include <video/imx-ipu-v3.h>
#include "ipu-prv.h"

#define DC_MAP_CONF_PTR(n)	(0x108 + ((n) & ~0x1) * 2)
#define DC_MAP_CONF_VAL(n)	(0x144 + ((n) & ~0x1) * 2)

#define DC_EVT_NF		0
#define DC_EVT_NL		1
#define DC_EVT_EOF		2
#define DC_EVT_NFIELD		3
#define DC_EVT_EOL		4
#define DC_EVT_EOFIELD		5
#define DC_EVT_NEW_ADDR		6
#define DC_EVT_NEW_CHAN		7
#define DC_EVT_NEW_DATA		8

#define DC_EVT_NEW_ADDR_W_0	0
#define DC_EVT_NEW_ADDR_W_1	1
#define DC_EVT_NEW_CHAN_W_0	2
#define DC_EVT_NEW_CHAN_W_1	3
#define DC_EVT_NEW_DATA_W_0	4
#define DC_EVT_NEW_DATA_W_1	5
#define DC_EVT_NEW_ADDR_R_0	6
#define DC_EVT_NEW_ADDR_R_1	7
#define DC_EVT_NEW_CHAN_R_0	8
#define DC_EVT_NEW_CHAN_R_1	9
#define DC_EVT_NEW_DATA_R_0	10
#define DC_EVT_NEW_DATA_R_1	11

#define DC_WR_CH_CONF		0x0
#define DC_WR_CH_ADDR		0x4
#define DC_RL_CH(evt)		(8 + ((evt) & ~0x1) * 2)

#define DC_GEN			0xd4
#define DC_DISP_CONF1(disp)	(0xd8 + (disp) * 4)
#define DC_DISP_CONF2(disp)	(0xe8 + (disp) * 4)
#define DC_STAT			0x1c8

#define WROD(lf)		(0x18 | ((lf) << 1))
#define WRG			0x01
#define WCLK			0xc9

#define SYNC_WAVE 0
#define NULL_WAVE (-1)

#define DC_GEN_SYNC_1_6_SYNC	(2 << 1)
#define DC_GEN_SYNC_PRIORITY_1	(1 << 7)

#define DC_WR_CH_CONF_WORD_SIZE_8		(0 << 0)
#define DC_WR_CH_CONF_WORD_SIZE_16		(1 << 0)
#define DC_WR_CH_CONF_WORD_SIZE_24		(2 << 0)
#define DC_WR_CH_CONF_WORD_SIZE_32		(3 << 0)
#define DC_WR_CH_CONF_DISP_ID_PARALLEL(i)	(((i) & 0x1) << 3)
#define DC_WR_CH_CONF_DISP_ID_SERIAL		(2 << 3)
#define DC_WR_CH_CONF_DISP_ID_ASYNC		(3 << 4)
#define DC_WR_CH_CONF_FIELD_MODE		(1 << 9)
#define DC_WR_CH_CONF_PROG_TYPE_NORMAL		(4 << 5)
#define DC_WR_CH_CONF_PROG_TYPE_MASK		(7 << 5)
#define DC_WR_CH_CONF_PROG_DI_ID		(1 << 2)
#define DC_WR_CH_CONF_PROG_DISP_ID(i)		(((i) & 0x1) << 3)

#define IPU_DC_NUM_CHANNELS	10
/* the number of mapping units; see IPUx_DC_MAP_CONF_{15..26} */
#define IPU_DC_NUM_MAPS		24
/* the (maximum) number of mapping pointers; see IPUx_DC_MAP_CONF_{0..14} */
#define IPU_DC_NUM_MAP_PNTR	30

struct ipu_dc_priv;

enum ipu_dc_map {
	IPU_DC_MAP_RGB24,
	IPU_DC_MAP_RGB565,
	IPU_DC_MAP_GBR24, /* TVEv2 */
	IPU_DC_MAP_BGR666,
	IPU_DC_MAP_LVDS666,
	IPU_DC_MAP_BGR24,
};

struct ipu_dc {
	/* The display interface number assigned to this dc channel */
	unsigned int		di;
	void __iomem		*base;
	struct ipu_dc_priv	*priv;
	int			chno;
	bool			in_use;
};

struct ipu_dc_priv {
	void __iomem		*dc_reg;
	void __iomem		*dc_tmpl_reg;
	struct ipu_soc		*ipu;
	struct device		*dev;
	struct ipu_dc		channels[IPU_DC_NUM_CHANNELS];
	struct mutex		mutex;
	struct completion	comp;
	int			dc_irq;
	int			dp_irq;
	int			use_count;
};

/* corresponds to IPUx_DC_MAP_CONF_{15..26} entry */
struct ipu_dc_map_conf {
	unsigned int	offset:5;
	unsigned int	mask:8;
};

typedef struct ipu_dc_map_conf		ipu_dc_map_ptr_t[3];

static ipu_dc_map_ptr_t const		IPU_DC_MAPPINGS[] = {
	[IPU_DC_MAP_RGB24] = {
		[0] = {  7, 0xff }, /* blue */
		[1] = { 15, 0xff }, /* green */
		[2] = { 23, 0xff }, /* red */
	},
	[IPU_DC_MAP_RGB565] = {
		[0] = {  4, 0xf8 }, /* blue */
		[1] = { 10, 0xfc }, /* green */
		[2] = { 15, 0xf8 }, /* red */
	},
	[IPU_DC_MAP_GBR24] = {
		[2] = { 15, 0xff }, /* green */
		[1] = {  7, 0xff }, /* blue */
		[0] = { 23, 0xff }, /* red */
	},
	[IPU_DC_MAP_BGR666] = {
		[0] = {  5, 0xfc }, /* blue */
		[1] = { 11, 0xfc }, /* green */
		[2] = { 17, 0xfc }, /* red */
	},
	[IPU_DC_MAP_LVDS666] = {
		[0] = {   5, 0xfc}, /* blue */
		[1] = {  13, 0xfc}, /* green */
		[2] = {  21, 0xfc}, /* red */
	},
	[IPU_DC_MAP_BGR24] = {
		[2] = {  7, 0xff }, /* red */
		[1] = { 15, 0xff }, /* green */
		[0] = { 23, 0xff }, /* blue */
	},
};

static void dc_link_event(struct ipu_dc *dc, int event, int addr, int priority)
{
	u32 reg;

	reg = readl(dc->base + DC_RL_CH(event));
	reg &= ~(0xffff << (16 * (event & 0x1)));
	reg |= ((addr << 8) | priority) << (16 * (event & 0x1));
	writel(reg, dc->base + DC_RL_CH(event));
}

static void dc_write_tmpl(struct ipu_dc *dc, int word, u32 opcode, u32 operand,
		int map, int wave, int glue, int sync, int stop)
{
	struct ipu_dc_priv *priv = dc->priv;
	u32 reg1, reg2;

	if (opcode == WCLK) {
		reg1 = (operand << 20) & 0xfff00000;
		reg2 = operand >> 12 | opcode << 1 | stop << 9;
	} else if (opcode == WRG) {
		reg1 = sync | glue << 4 | ++wave << 11 | ((operand << 15) & 0xffff8000);
		reg2 = operand >> 17 | opcode << 7 | stop << 9;
	} else {
		reg1 = sync | glue << 4 | ++wave << 11 | ++map << 15 | ((operand << 20) & 0xfff00000);
		reg2 = operand >> 12 | opcode << 4 | stop << 9;
	}
	writel(reg1, priv->dc_tmpl_reg + word * 8);
	writel(reg2, priv->dc_tmpl_reg + word * 8 + 4);
}

static int ipu_bus_format_to_map(u32 fmt)
{
	switch (fmt) {
	case MEDIA_BUS_FMT_RGB888_1X24:
		return IPU_DC_MAP_RGB24;
	case MEDIA_BUS_FMT_RGB565_1X16:
		return IPU_DC_MAP_RGB565;
	case MEDIA_BUS_FMT_GBR888_1X24:
		return IPU_DC_MAP_GBR24;
	case MEDIA_BUS_FMT_RGB666_1X18:
		return IPU_DC_MAP_BGR666;
	case MEDIA_BUS_FMT_RGB666_1X24_CPADHI:
		return IPU_DC_MAP_LVDS666;
	case MEDIA_BUS_FMT_BGR888_1X24:
		return IPU_DC_MAP_BGR24;
	default:
		return -EINVAL;
	}
}

int ipu_dc_init_sync(struct ipu_dc *dc, struct ipu_di *di, bool interlaced,
		u32 bus_format, u32 width)
{
	struct ipu_dc_priv *priv = dc->priv;
	u32 reg = 0;
	int map;

	dc->di = ipu_di_get_num(di);

	map = ipu_bus_format_to_map(bus_format);
	if (map < 0) {
		dev_dbg(priv->dev, "IPU_DISP: No MAP\n");
		return map;
	}

	if (interlaced) {
		int addr;

		if (dc->di)
			addr = 1;
		else
			addr = 0;

		dc_link_event(dc, DC_EVT_NL, addr, 3);
		dc_link_event(dc, DC_EVT_EOL, addr, 2);
		dc_link_event(dc, DC_EVT_NEW_DATA, addr, 1);

		/* Init template microcode */
		dc_write_tmpl(dc, addr, WROD(0), 0, map, SYNC_WAVE, 0, 6, 1);
	} else {
		if (dc->di) {
			dc_link_event(dc, DC_EVT_NL, 2, 3);
			dc_link_event(dc, DC_EVT_EOL, 3, 2);
			dc_link_event(dc, DC_EVT_NEW_DATA, 1, 1);
			/* Init template microcode */
			dc_write_tmpl(dc, 2, WROD(0), 0, map, SYNC_WAVE, 8, 5, 1);
			dc_write_tmpl(dc, 3, WROD(0), 0, map, SYNC_WAVE, 4, 5, 0);
			dc_write_tmpl(dc, 4, WRG, 0, map, NULL_WAVE, 0, 0, 1);
			dc_write_tmpl(dc, 1, WROD(0), 0, map, SYNC_WAVE, 0, 5, 1);
		} else {
			dc_link_event(dc, DC_EVT_NL, 5, 3);
			dc_link_event(dc, DC_EVT_EOL, 6, 2);
			dc_link_event(dc, DC_EVT_NEW_DATA, 8, 1);
			/* Init template microcode */
			dc_write_tmpl(dc, 5, WROD(0), 0, map, SYNC_WAVE, 8, 5, 1);
			dc_write_tmpl(dc, 6, WROD(0), 0, map, SYNC_WAVE, 4, 5, 0);
			dc_write_tmpl(dc, 7, WRG, 0, map, NULL_WAVE, 0, 0, 1);
			dc_write_tmpl(dc, 8, WROD(0), 0, map, SYNC_WAVE, 0, 5, 1);
		}
	}
	dc_link_event(dc, DC_EVT_NF, 0, 0);
	dc_link_event(dc, DC_EVT_NFIELD, 0, 0);
	dc_link_event(dc, DC_EVT_EOF, 0, 0);
	dc_link_event(dc, DC_EVT_EOFIELD, 0, 0);
	dc_link_event(dc, DC_EVT_NEW_CHAN, 0, 0);
	dc_link_event(dc, DC_EVT_NEW_ADDR, 0, 0);

	reg = readl(dc->base + DC_WR_CH_CONF);
	if (interlaced)
		reg |= DC_WR_CH_CONF_FIELD_MODE;
	else
		reg &= ~DC_WR_CH_CONF_FIELD_MODE;
	writel(reg, dc->base + DC_WR_CH_CONF);

	writel(0x0, dc->base + DC_WR_CH_ADDR);
	writel(width, priv->dc_reg + DC_DISP_CONF2(dc->di));

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_dc_init_sync);

void ipu_dc_enable(struct ipu_soc *ipu)
{
	struct ipu_dc_priv *priv = ipu->dc_priv;

	mutex_lock(&priv->mutex);

	if (!priv->use_count)
		ipu_module_enable(priv->ipu, IPU_CONF_DC_EN);

	priv->use_count++;

	mutex_unlock(&priv->mutex);
}
EXPORT_SYMBOL_GPL(ipu_dc_enable);

void ipu_dc_enable_channel(struct ipu_dc *dc)
{
	u32 reg;

	reg = readl(dc->base + DC_WR_CH_CONF);
	reg |= DC_WR_CH_CONF_PROG_TYPE_NORMAL;
	writel(reg, dc->base + DC_WR_CH_CONF);
}
EXPORT_SYMBOL_GPL(ipu_dc_enable_channel);

static irqreturn_t dc_irq_handler(int irq, void *dev_id)
{
	struct ipu_dc *dc = dev_id;
	u32 reg;

	reg = readl(dc->base + DC_WR_CH_CONF);
	reg &= ~DC_WR_CH_CONF_PROG_TYPE_MASK;
	writel(reg, dc->base + DC_WR_CH_CONF);

	/* The Freescale BSP kernel clears DIx_COUNTER_RELEASE here */

	complete(&dc->priv->comp);
	return IRQ_HANDLED;
}

void ipu_dc_disable_channel(struct ipu_dc *dc)
{
	struct ipu_dc_priv *priv = dc->priv;
	int irq;
	unsigned long ret;
	u32 val;

	/* TODO: Handle MEM_FG_SYNC differently from MEM_BG_SYNC */
	if (dc->chno == 1)
		irq = priv->dc_irq;
	else if (dc->chno == 5)
		irq = priv->dp_irq;
	else
		return;

	init_completion(&priv->comp);
	enable_irq(irq);
	ret = wait_for_completion_timeout(&priv->comp, msecs_to_jiffies(50));
	disable_irq(irq);
	if (ret == 0) {
		dev_warn(priv->dev, "DC stop timeout after 50 ms\n");

		val = readl(dc->base + DC_WR_CH_CONF);
		val &= ~DC_WR_CH_CONF_PROG_TYPE_MASK;
		writel(val, dc->base + DC_WR_CH_CONF);
	}
}
EXPORT_SYMBOL_GPL(ipu_dc_disable_channel);

void ipu_dc_disable(struct ipu_soc *ipu)
{
	struct ipu_dc_priv *priv = ipu->dc_priv;

	mutex_lock(&priv->mutex);

	priv->use_count--;
	if (!priv->use_count)
		ipu_module_disable(priv->ipu, IPU_CONF_DC_EN);

	if (priv->use_count < 0)
		priv->use_count = 0;

	mutex_unlock(&priv->mutex);
}
EXPORT_SYMBOL_GPL(ipu_dc_disable);

/* Try to lookup [offset, mask] pair in priv->maps[] and create a new entry
 * when not found.  Returns index of pair or -ENOMEM when all slots are
 * already used. */
static int ipu_dc_add_mapping(struct ipu_dc_priv *priv,
			      struct ipu_dc_map_conf maps[],
			      unsigned long *maps_used_field,
			      struct ipu_dc_map_conf const *setup)
{
	unsigned int const	offset = setup->offset;
	unsigned int const	mask = setup->mask;
	unsigned int		idx = 0;
	struct ipu_dc_map_conf	*map = NULL;

	for_each_set_bit(idx, maps_used_field, IPU_DC_NUM_MAPS) {
		if (maps[idx].offset == offset && maps[idx].mask == mask) {
			/* map entry @idx is used and matching; assign map and
			 * abort loop */
			map = &maps[idx];
			break;
		}
	}

	if (!map) {
		idx = find_first_zero_bit(maps_used_field, IPU_DC_NUM_MAPS);
		if (idx < IPU_DC_NUM_MAPS) {
			map = &maps[idx];

			/* map entry @idx is unused; assign our value and mark
			 * it as used */
			map->offset = offset;
			map->mask   = mask;

			set_bit(idx, maps_used_field);
		}
	}

	dev_dbg(priv->dev, "%s: [%02x@%u] -> map=%p, idx=%zu\n", __func__,
		mask, offset, map, idx);

	if (!map) {
		dev_warn(priv->dev, "no mapping space available for %04x@%d\n",
			 mask, offset);

		return -ENOMEM;
	}

	return map - &maps[0];
}

static int ipu_dc_register_mappings(struct ipu_dc_priv *priv)
{
	size_t			i;
	int			rc;
	uint16_t		pntr[ARRAY_SIZE(IPU_DC_MAPPINGS)];
	size_t			num_ptr;
	struct ipu_dc_map_conf	maps[IPU_DC_NUM_MAPS];
	unsigned long		used_maps = 0;

	BUILD_BUG_ON(ARRAY_SIZE(IPU_DC_MAPPINGS) > IPU_DC_NUM_MAP_PNTR);

	for (i = 0; i < ARRAY_SIZE(IPU_DC_MAPPINGS); ++i) {
		ipu_dc_map_ptr_t const	*setup = &IPU_DC_MAPPINGS[i];
		size_t			j;

		pntr[i] = 0;

		for (j = 0; j < ARRAY_SIZE(*setup); ++j) {
			rc = ipu_dc_add_mapping(priv, maps, &used_maps,
						&(*setup)[j]);

			if (rc < 0)
				/* all maps are used... */
				goto out;

			BUG_ON(rc >= (1 << 5));

			pntr[i] |= rc << (j * 5);
		}
	}

	num_ptr = i;

	/* code below assumes an even number of maps; check constraint */
	BUILD_BUG_ON(ARRAY_SIZE(maps) % 2 != 0);

	for (i = 0; i < ARRAY_SIZE(maps); i += 2) {
		uint32_t	v = 0;

		if (used_maps & (1u << i))
			v |= ((maps[i].mask << 0) |
			      (maps[i].offset << 8)) << 0;

		if (used_maps & (1u << (i+1)))
			v |= ((maps[i+1].mask << 0) |
			      (maps[i+1].offset << 8)) << 16;

		writel(v, priv->dc_reg + DC_MAP_CONF_VAL(i));
	}

	for (i = 0; i+1 < num_ptr; i += 2) {
		uint32_t	v = 0;

		v |= pntr[i];
		v |= pntr[i+1] << 16;

		writel(v, priv->dc_reg + DC_MAP_CONF_PTR(i));
	}

	/* handle trailing pntr when odd number of mappings is used */
	if (i < num_ptr)
		writel(pntr[i], priv->dc_reg + DC_MAP_CONF_PTR(i));

	dev_dbg(priv->dev,
		"registered %zu mappings with %4lx configuration mask\n",
		num_ptr, used_maps);

	rc = 0;

out:
	return rc;
}

struct ipu_dc *ipu_dc_get(struct ipu_soc *ipu, int channel)
{
	struct ipu_dc_priv *priv = ipu->dc_priv;
	struct ipu_dc *dc;

	if (channel >= IPU_DC_NUM_CHANNELS)
		return ERR_PTR(-ENODEV);

	dc = &priv->channels[channel];

	mutex_lock(&priv->mutex);

	if (dc->in_use) {
		mutex_unlock(&priv->mutex);
		return ERR_PTR(-EBUSY);
	}

	dc->in_use = true;

	mutex_unlock(&priv->mutex);

	return dc;
}
EXPORT_SYMBOL_GPL(ipu_dc_get);

void ipu_dc_put(struct ipu_dc *dc)
{
	struct ipu_dc_priv *priv = dc->priv;

	mutex_lock(&priv->mutex);
	dc->in_use = false;
	mutex_unlock(&priv->mutex);
}
EXPORT_SYMBOL_GPL(ipu_dc_put);

int ipu_dc_init(struct ipu_soc *ipu, struct device *dev,
		unsigned long base, unsigned long template_base)
{
	struct ipu_dc_priv *priv;
	static int channel_offsets[] = { 0, 0x1c, 0x38, 0x54, 0x58, 0x5c,
		0x78, 0, 0x94, 0xb4};
	int i, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->mutex);

	priv->dev = dev;
	priv->ipu = ipu;
	priv->dc_reg = devm_ioremap(dev, base, PAGE_SIZE);
	priv->dc_tmpl_reg = devm_ioremap(dev, template_base, PAGE_SIZE);
	if (!priv->dc_reg || !priv->dc_tmpl_reg)
		return -ENOMEM;

	for (i = 0; i < IPU_DC_NUM_CHANNELS; i++) {
		priv->channels[i].chno = i;
		priv->channels[i].priv = priv;
		priv->channels[i].base = priv->dc_reg + channel_offsets[i];
	}

	priv->dc_irq = ipu_map_irq(ipu, IPU_IRQ_DC_FC_1);
	if (!priv->dc_irq)
		return -EINVAL;
	ret = devm_request_irq(dev, priv->dc_irq, dc_irq_handler, 0, NULL,
			       &priv->channels[1]);
	if (ret < 0)
		return ret;
	disable_irq(priv->dc_irq);
	priv->dp_irq = ipu_map_irq(ipu, IPU_IRQ_DP_SF_END);
	if (!priv->dp_irq)
		return -EINVAL;
	ret = devm_request_irq(dev, priv->dp_irq, dc_irq_handler, 0, NULL,
			       &priv->channels[5]);
	if (ret < 0)
		return ret;
	disable_irq(priv->dp_irq);

	writel(DC_WR_CH_CONF_WORD_SIZE_24 | DC_WR_CH_CONF_DISP_ID_PARALLEL(1) |
			DC_WR_CH_CONF_PROG_DI_ID,
			priv->channels[1].base + DC_WR_CH_CONF);
	writel(DC_WR_CH_CONF_WORD_SIZE_24 | DC_WR_CH_CONF_DISP_ID_PARALLEL(0),
			priv->channels[5].base + DC_WR_CH_CONF);

	writel(DC_GEN_SYNC_1_6_SYNC | DC_GEN_SYNC_PRIORITY_1,
		priv->dc_reg + DC_GEN);

	ipu->dc_priv = priv;

	dev_dbg(dev, "DC base: 0x%08lx template base: 0x%08lx\n",
			base, template_base);

	ret = ipu_dc_register_mappings(priv);
	/* TODO: handle error? */

	return ret;
}

void ipu_dc_exit(struct ipu_soc *ipu)
{
}
