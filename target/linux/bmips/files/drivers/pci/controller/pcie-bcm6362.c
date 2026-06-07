// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM6362 PCIe Controller Driver
 *
 * Reproduces the exact RC bring-up of the Broadcom OEM platform code for
 * chip 0x6362 (DSL-3580L), evaluated for CONFIG_BCM96362 with PCIEH defined
 * and UBUS2_PCIE undefined. Structure follows pcie-bcm6328.c (the driver that
 * already enumerates the external BCM4352 on this SoC); the divergences from
 * it are exactly the points where the 6328 path differs from the OEM 6362
 * path. Every register write below is traceable to an OEM source line and to
 * an offset/bit in 6362_map_part.h — see README.md in this directory.
 *
 * Copyright (C) 2020 Álvaro Fernández Rojas <noltari@gmail.com>  (6318/6328 base)
 * Copyright (C) 2015 Jonas Gorski <jonas.gorski@gmail.com>
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 */

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

#include "../pci.h"

/* brcm,serdes syscon (MISC->miscSerdesCtrl @ 0x10001804); bits per 6362_map_part.h */
#define SERDES_PCIE_EN			BIT(0)		/* SERDES_PCIE_ENABLE     0x1   */
#define SERDES_PCIE_EXD_EN		BIT(15)		/* SERDES_PCIE_EXD_ENABLE (1<<15) */

#define PCIE_BUS_BRIDGE			0
#define PCIE_BUS_DEVICE			1

/* offsets within the RC register block (reg = <0x10e40000 0x10000>) */
#define PCIE_CONFIG2_REG		0x408		/* PcieBlk404Regs.config2 */
#define CONFIG2_BAR1_SIZE_MASK		0xf		/* PCIE_IP_BLK404_CONFIG_2_BAR1_SIZE_MASK */

#define PCIE_IDVAL3_REG			0x43c		/* PcieBlk428Regs.idVal3 */
#define IDVAL3_CLASS_CODE_MASK		0x00ffffff	/* PCIE_IP_BLK428_ID_VAL3_CLASS_CODE_MASK */
#define IDVAL3_SUBCLASS_SHIFT		8		/* OEM writes (class << 8) */

#define PCIE_DLSTATUS_REG		0x1048		/* PcieBlk1000Regs.dlStatus */
#define DLSTATUS_PHYLINKUP		(1 << 13)	/* PCIE_IP_BLK1000_DL_STATUS_PHYLINKUP_MASK 0x2000 */

/* PcieBridgeRegs @ PCIE_BASE+0x2818; field offsets give the absolute values below */
#define PCIE_BRIDGE_OPT1_REG		0x2820		/* bridgeOptReg1 */
#define OPT1_EN_L1_INT_STATUS_MASK_POL	(1 << 12)	/* en_l1_int_status_mask_polarity */
#define OPT1_PCIE_BRIDGE_HOLE_DET_EN	(1 << 11)	/* en_pcie_bridge_hole_detection */
#define OPT1_RD_REPLY_BE_FIX_EN		(1 << 9)	/* en_rd_reply_be_fix */
#define OPT1_RD_BE_OPT_EN		(1 << 7)	/* enable_rd_be_opt */

#define PCIE_BRIDGE_OPT2_REG		0x2824		/* bridgeOptReg2 */
#define OPT2_UBUS_UR_DECODE_DIS		(1 << 2)	/* dis_ubus_ur_decode 0x04 */
#define OPT2_TX_CREDIT_CHK_EN		(1 << 4)	/* enable_tx_crd_chk  0x10 */
#define OPT2_CFG_TYPE1_BD_SEL		(1 << 7)	/* cfg_type1_bd_sel   0x80 */
#define OPT2_CFG_TYPE1_BUS_NO_SHIFT	16		/* cfg_type1_bus_no_SHIFT */
#define OPT2_CFG_TYPE1_BUS_NO_MASK	(0xff << 16)	/* cfg_type1_bus_no_MASK 0x00ff0000 */

#define PCIE_BRIDGE_RC_INT_MASK_REG	0x2854		/* rcInterruptMask */
#define PCIE_RC_INT_A			(1 << 0)
#define PCIE_RC_INT_B			(1 << 1)
#define PCIE_RC_INT_C			(1 << 2)
/* int_d (1<<3) exists in the header but the OEM 6362 mask does NOT set it. */

#define PCIE_DEVICE_OFFSET		0x8000		/* PCIEH_DEV_OFFSET */

/*
 * Deliberately NOT defined/used here: PcieBridgeRegs.Ubus2PcieBar0BaseMask
 * (0x2828) / Ubus2PcieBar0RemapAdd (0x282c). The OEM 6362 path leaves these
 * at reset (the OEM bcm63xx_pci_init Ubus2PcieBar0BaseMask write is gated to
 * 963268||96828||96818, which excludes 6362, and pcie_init does not touch
 * them). pcie-bcm6328.c writes them from mem_resource; that write is the main
 * divergence from OEM-6362 behaviour and is intentionally dropped.
 */

struct bcm6362_pcie {
	void __iomem *base;
	int irq;
	struct regmap *serdes;
	struct clk *clk;
	struct reset_control *reset;
	struct reset_control *reset_ext;
	struct reset_control *reset_core;
};

static struct bcm6362_pcie bcm6362_pcie;

extern int bmips_pci_irq;

static int postprocess_read(u32 data, int where, unsigned int size)
{
	u32 ret = 0;

	switch (size) {
	case 1:
		ret = (data >> ((where & 3) << 3)) & 0xff;
		break;
	case 2:
		ret = (data >> ((where & 3) << 3)) & 0xffff;
		break;
	case 4:
		ret = data;
		break;
	}

	return ret;
}

static int preprocess_write(u32 orig_data, u32 val, int where,
			    unsigned int size)
{
	u32 ret = 0;

	switch (size) {
	case 1:
		ret = (orig_data & ~(0xff << ((where & 3) << 3))) |
		      (val << ((where & 3) << 3));
		break;
	case 2:
		ret = (orig_data & ~(0xffff << ((where & 3) << 3))) |
		      (val << ((where & 3) << 3));
		break;
	case 4:
		ret = val;
		break;
	}

	return ret;
}

static int bcm6362_pcie_can_access(struct pci_bus *bus, int devfn)
{
	struct bcm6362_pcie *priv = &bcm6362_pcie;

	switch (bus->number) {
	case PCIE_BUS_BRIDGE:
		return PCI_SLOT(devfn) == 0;
	case PCIE_BUS_DEVICE:
		if (PCI_SLOT(devfn) == 0)
			return __raw_readl(priv->base + PCIE_DLSTATUS_REG)
			       & DLSTATUS_PHYLINKUP;
		fallthrough;
	default:
		return false;
	}
}

static int bcm6362_pcie_read(struct pci_bus *bus, unsigned int devfn,
			     int where, int size, u32 *val)
{
	struct bcm6362_pcie *priv = &bcm6362_pcie;
	u32 data;
	u32 reg = where & ~3;

	if (!bcm6362_pcie_can_access(bus, devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == PCIE_BUS_DEVICE)
		reg += PCIE_DEVICE_OFFSET;

	data = __raw_readl(priv->base + reg);
	*val = postprocess_read(data, where, size);

	return PCIBIOS_SUCCESSFUL;
}

static int bcm6362_pcie_write(struct pci_bus *bus, unsigned int devfn,
			      int where, int size, u32 val)
{
	struct bcm6362_pcie *priv = &bcm6362_pcie;
	u32 data;
	u32 reg = where & ~3;

	if (!bcm6362_pcie_can_access(bus, devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == PCIE_BUS_DEVICE)
		reg += PCIE_DEVICE_OFFSET;

	data = __raw_readl(priv->base + reg);
	data = preprocess_write(data, val, where, size);
	__raw_writel(data, priv->base + reg);

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops bcm6362_pcie_ops = {
	.read = bcm6362_pcie_read,
	.write = bcm6362_pcie_write,
};

static struct resource bcm6362_pcie_io_resource;
static struct resource bcm6362_pcie_mem_resource;
static struct resource bcm6362_pcie_busn_resource;

static struct pci_controller bcm6362_pcie_controller = {
	.pci_ops = &bcm6362_pcie_ops,
	.io_resource = &bcm6362_pcie_io_resource,
	.mem_resource = &bcm6362_pcie_mem_resource,
};

/*
 * OEM 6362 bring-up sequence, from setup.c pcie_init() #else (ubus1) arm:
 *   PERF->blkEnables   |= PCIE_CLK_EN;                       (clk "pcie")
 *   MISC->miscSerdesCtrl |= SERDES_PCIE_ENABLE|EXD_ENABLE;   (brcm,serdes)
 *   PERF->softResetB &= ~(PCIE|EXT|CORE);                    (assert all)
 *   -- NO SOFT_RST_PCIE_HARD: 6362 is excluded from that gate --
 *   mdelay(10);
 *   PERF->softResetB |= (PCIE|CORE);                         (deassert)
 *   mdelay(10);
 *   PERF->softResetB |= (EXT);                               (deassert ext)
 *   mdelay(200);
 * softResetB bit polarity (clear = in reset) is handled by the reset
 * controller, so &=~ maps to assert and |= maps to deassert.
 */
static void bcm6362_pcie_reset(struct bcm6362_pcie *priv)
{
	/* serdes enable (MISC->miscSerdesCtrl, offset 0 of brcm,serdes) */
	regmap_write_bits(priv->serdes, 0,
			  SERDES_PCIE_EXD_EN | SERDES_PCIE_EN,
			  SERDES_PCIE_EXD_EN | SERDES_PCIE_EN);

	/* assert PCIE, PCIE_CORE, PCIE_EXT */
	reset_control_assert(priv->reset);
	reset_control_assert(priv->reset_core);
	reset_control_assert(priv->reset_ext);
	mdelay(10);

	/* deassert PCIE + PCIE_CORE together */
	reset_control_deassert(priv->reset);
	reset_control_deassert(priv->reset_core);
	mdelay(10);

	/* deassert PCIE_EXT last */
	reset_control_deassert(priv->reset_ext);

	/* critical delay (OEM) */
	mdelay(200);
}

/*
 * OEM 6362 RC config, from bcm63xx_pci_init() CONFIG_BCM96362 arms.
 * No Ubus2PcieBar0BaseMask/RemapAdd write (see block comment above).
 */
static void bcm6362_pcie_setup(struct bcm6362_pcie *priv)
{
	u32 val;

	val = __raw_readl(priv->base + PCIE_BRIDGE_OPT1_REG);
	val |= OPT1_EN_L1_INT_STATUS_MASK_POL;
	val |= OPT1_PCIE_BRIDGE_HOLE_DET_EN;
	val |= OPT1_RD_REPLY_BE_FIX_EN;
	val |= OPT1_RD_BE_OPT_EN;
	__raw_writel(val, priv->base + PCIE_BRIDGE_OPT1_REG);

	/* OEM mask: int_a | int_b | int_c (int_c was duplicated, dropped; no int_d) */
	val = __raw_readl(priv->base + PCIE_BRIDGE_RC_INT_MASK_REG);
	val |= PCIE_RC_INT_A;
	val |= PCIE_RC_INT_B;
	val |= PCIE_RC_INT_C;
	__raw_writel(val, priv->base + PCIE_BRIDGE_RC_INT_MASK_REG);

	val = __raw_readl(priv->base + PCIE_BRIDGE_OPT2_REG);
	/* credit checking + UBUS UR decode disable */
	val |= OPT2_TX_CREDIT_CHK_EN;
	val |= OPT2_UBUS_UR_DECODE_DIS;
	/* type1 cfg: fix the device bus number */
	val |= (PCIE_BUS_DEVICE << OPT2_CFG_TYPE1_BUS_NO_SHIFT);
	val |= OPT2_CFG_TYPE1_BD_SEL;
	__raw_writel(val, priv->base + PCIE_BRIDGE_OPT2_REG);

	/* class code as PCI-to-PCI bridge */
	val = __raw_readl(priv->base + PCIE_IDVAL3_REG);
	val &= ~IDVAL3_CLASS_CODE_MASK;
	val |= (PCI_CLASS_BRIDGE_PCI << IDVAL3_SUBCLASS_SHIFT);
	__raw_writel(val, priv->base + PCIE_IDVAL3_REG);

	/* disable bar1 size */
	val = __raw_readl(priv->base + PCIE_CONFIG2_REG);
	val &= ~CONFIG2_BAR1_SIZE_MASK;
	__raw_writel(val, priv->base + PCIE_CONFIG2_REG);
}

static int bcm6362_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct bcm6362_pcie *priv = &bcm6362_pcie;
	struct resource *res;
	int ret;
	LIST_HEAD(resources);

	of_pci_check_probe_only();

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->irq = platform_get_irq(pdev, 0);
	if (!priv->irq)
		return -ENODEV;

	bmips_pci_irq = priv->irq;

	priv->serdes = syscon_regmap_lookup_by_phandle(np, "brcm,serdes");
	if (IS_ERR(priv->serdes))
		return PTR_ERR(priv->serdes);

	priv->reset = devm_reset_control_get(dev, "pcie");
	if (IS_ERR(priv->reset))
		return PTR_ERR(priv->reset);

	priv->reset_ext = devm_reset_control_get(dev, "pcie-ext");
	if (IS_ERR(priv->reset_ext))
		return PTR_ERR(priv->reset_ext);

	priv->reset_core = devm_reset_control_get(dev, "pcie-core");
	if (IS_ERR(priv->reset_core))
		return PTR_ERR(priv->reset_core);

	/* 6362 has no SOFT_RST_PCIE_HARD; no pcie-hard reset is requested. */

	priv->clk = devm_clk_get(dev, "pcie");
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(dev, "could not enable clock\n");
		return ret;
	}

	pci_load_of_ranges(&bcm6362_pcie_controller, np);
	if (!bcm6362_pcie_mem_resource.start)
		return -EINVAL;

	of_pci_parse_bus_range(np, &bcm6362_pcie_busn_resource);
	pci_add_resource(&resources, &bcm6362_pcie_busn_resource);

	bcm6362_pcie_reset(priv);
	bcm6362_pcie_setup(priv);

	register_pci_controller(&bcm6362_pcie_controller);

	return 0;
}

static const struct of_device_id bcm6362_pcie_of_match[] = {
	{ .compatible = "brcm,bcm6362-pcie", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bcm6362_pcie_of_match);

static struct platform_driver bcm6362_pcie_driver = {
	.probe = bcm6362_pcie_probe,
	.driver	= {
		.name = "bcm6362-pcie",
		.of_match_table = bcm6362_pcie_of_match,
	},
};
module_platform_driver(bcm6362_pcie_driver);

MODULE_AUTHOR("Alessio Ferri <alessio.ferri@mythread.it>");
MODULE_DESCRIPTION("BCM6362 PCIe Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bcm6362-pcie");
