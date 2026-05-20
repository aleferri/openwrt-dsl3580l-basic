// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM6362 on-chip WLAN shim driver.
 *
 * The BCM6362 has an integrated Broadcom 2.4 GHz WLAN block whose
 * register backplane is a Broadcom AMBA (AXI/OCP) — exactly what the
 * bcma driver calls "brcm,bus-axi". The backplane sits on the SoC
 * ubus, behind a small "SHIM" bridge that gates clocks and holds the
 * WLAN macro in reset until released by software.
 *
 *   ubus  ─┬─►  WLAN SHIM  ─►  AXI backplane  ┬─► ChipCommon @ 0x10004000
 *          │    @ 0x10007000                  ├─► d11 MAC core @ 0x10005000
 *          │                                  └─► (PMU, GPIO, ... lives in
 *          │                                       ChipCommon)
 *          └─►  rest of the SoC
 *
 * This driver owns nothing about the WLAN backplane itself — it only
 * brings the SHIM up (clock on, reset deassert, SHIM_MISC/MAC_CONTROL
 * sequence verbatim from the OEM Linux 2.6.30 setup.c "Enable WOC"
 * block; CFE does not bring this block up) and then lets bcma's
 * standard host_soc.c probe the AXI backplane via of_platform_populate.
 *
 * Bring-up sequence and SHIM register layout match the OEM source
 * arch/mips/bcm963xx/setup.c and the WlanShimRegs struct in
 * shared/opensource/include/bcm963xx/6362_map_part.h. The fake-PCI
 * dance the OEM kernel does *after* bring-up is intentionally absent
 * here: bcma host_soc.c speaks to the backplane natively.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

/* SHIM register layout (struct WlanShimRegs in 6362_map_part.h). */
#define SHIM_MISC		0x00
#define   SHIM_FORCE_CLK_ON	BIT(2)
#define   SHIM_MACRO_DISABLE	BIT(1)
#define   SHIM_MACRO_SOFT_RESET	BIT(0)
#define SHIM_STATUS		0x04
#define SHIM_CC_CONTROL		0x08
#define SHIM_CC_STATUS		0x0c
#define SHIM_MAC_CONTROL	0x10
#define   SICF_FGC		BIT(1)	/* force gated clock */
#define   SICF_CLOCK_EN		BIT(0)
#define SHIM_MAC_STATUS		0x14
#define SHIM_CC_ID_A		0x18
#define SHIM_MAC_ID_A		0x24

struct bcm6362_wlan {
	struct device		*dev;
	void __iomem		*shim;
	struct clk		*clk;
	struct reset_control	*rst_shim;
	struct reset_control	*rst_shim_ubus;
};

static int bcm6362_wlan_bringup(struct bcm6362_wlan *priv)
{
	int ret;

	dev_info(priv->dev, "bring-up: start\n");

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(priv->dev, "clk_prepare_enable failed: %d\n", ret);
		return ret;
	}
	dev_info(priv->dev, "bring-up: clock enabled, rate=%lu Hz\n",
		 clk_get_rate(priv->clk));
	mdelay(10);

	/* Reset toggle (brcm,bcm6345-reset hides the active-low softResetB
	 * encoding, so assert/deassert read naturally here). */
	reset_control_assert(priv->rst_shim_ubus);
	reset_control_assert(priv->rst_shim);
	mdelay(1);
	reset_control_deassert(priv->rst_shim_ubus);
	reset_control_deassert(priv->rst_shim);
	mdelay(1);
	dev_info(priv->dev, "bring-up: reset toggled\n");

	/* SHIM: force clocks on + hold WLAN macro in soft reset.
	 *
	 * The SHIM and the AXI backplane behind it are big-endian peripherals
	 * on a big-endian MIPS CPU. The asymmetric-endian writel() in this
	 * configuration byte-swaps the value (it assumes a little-endian
	 * bus, typical for PCI), which lands each bit in the wrong position
	 * inside the peripheral register. iowrite32be() is a no-op transform
	 * here (BE-to-BE) and writes the value the bring-up sequence intends.
	 * Same story for readback: readl() would byte-swap on the way back
	 * and lie to the diagnostic line below. */
	iowrite32be(SHIM_FORCE_CLK_ON | SHIM_MACRO_SOFT_RESET,
		    priv->shim + SHIM_MISC);
	mdelay(1);

	/* MAC core: force gated clock + clock enable (with reset held). */
	iowrite32be(SICF_FGC | SICF_CLOCK_EN, priv->shim + SHIM_MAC_CONTROL);

	/* Release macro soft reset, keep clocks forced. */
	iowrite32be(SHIM_FORCE_CLK_ON, priv->shim + SHIM_MISC);

	/* Drop the force, let normal gating take over. */
	iowrite32be(0, priv->shim + SHIM_MISC);
	iowrite32be(SICF_CLOCK_EN, priv->shim + SHIM_MAC_CONTROL);

	/* Read-back diagnostics: if the backplane is alive these reflect
	 * the values we just wrote (MISC=0, MAC_CONTROL=SICF_CLOCK_EN) and
	 * the STATUS regs report sane non-zero core ids. */
	dev_info(priv->dev,
		 "bring-up: post-shim MISC=%08x STATUS=%08x CC_CTRL=%08x CC_STAT=%08x MAC_CTRL=%08x MAC_STAT=%08x\n",
		 ioread32be(priv->shim + SHIM_MISC),
		 ioread32be(priv->shim + SHIM_STATUS),
		 ioread32be(priv->shim + SHIM_CC_CONTROL),
		 ioread32be(priv->shim + SHIM_CC_STATUS),
		 ioread32be(priv->shim + SHIM_MAC_CONTROL),
		 ioread32be(priv->shim + SHIM_MAC_STATUS));
	dev_info(priv->dev,
		 "bring-up: CcIdA=%08x MacIdA=%08x (non-zero = backplane responsive)\n",
		 ioread32be(priv->shim + SHIM_CC_ID_A),
		 ioread32be(priv->shim + SHIM_MAC_ID_A));

	return 0;
}

static void bcm6362_wlan_teardown(struct bcm6362_wlan *priv)
{
	iowrite32be(0, priv->shim + SHIM_MAC_CONTROL);
	iowrite32be(SHIM_MACRO_DISABLE | SHIM_MACRO_SOFT_RESET,
		    priv->shim + SHIM_MISC);
	reset_control_assert(priv->rst_shim);
	reset_control_assert(priv->rst_shim_ubus);
	clk_disable_unprepare(priv->clk);
}

static int bcm6362_wlan_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm6362_wlan *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = dev;

	priv->shim = devm_platform_ioremap_resource_byname(pdev, "shim");
	if (IS_ERR(priv->shim))
		return PTR_ERR(priv->shim);

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	priv->rst_shim = devm_reset_control_get_exclusive(dev, "shim");
	if (IS_ERR(priv->rst_shim))
		return PTR_ERR(priv->rst_shim);

	priv->rst_shim_ubus = devm_reset_control_get_exclusive(dev, "shim-ubus");
	if (IS_ERR(priv->rst_shim_ubus))
		return PTR_ERR(priv->rst_shim_ubus);

	ret = bcm6362_wlan_bringup(priv);
	if (ret) {
		dev_err(dev, "WLAN bring-up failed: %d\n", ret);
		return ret;
	}

	/* Now that the SHIM is up, instantiate the child node — the
	 * "brcm,bus-axi" sub-node — which bcma host_soc.c will probe. */
	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret) {
		dev_err(dev, "failed to populate AXI child: %d\n", ret);
		bcm6362_wlan_teardown(priv);
		return ret;
	}

	platform_set_drvdata(pdev, priv);
	return 0;
}

static void bcm6362_wlan_remove(struct platform_device *pdev)
{
	struct bcm6362_wlan *priv = platform_get_drvdata(pdev);

	of_platform_depopulate(&pdev->dev);
	bcm6362_wlan_teardown(priv);
}

static const struct of_device_id bcm6362_wlan_match[] = {
	{ .compatible = "brcm,bcm6362-wlan", },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm6362_wlan_match);

static struct platform_driver bcm6362_wlan_driver = {
	.probe	= bcm6362_wlan_probe,
	.remove	= bcm6362_wlan_remove,
	.driver	= {
		.name		= "bcm6362-wlan",
		.of_match_table	= bcm6362_wlan_match,
	},
};
module_platform_driver(bcm6362_wlan_driver);

MODULE_DESCRIPTION("BCM6362 on-chip WLAN SHIM driver");
MODULE_LICENSE("GPL");
