// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM6362 WLAN-side register dump (diagnostic-only).
 *
 * Independent verification path for what the on-chip 2.4 GHz WLAN
 * backplane is actually exposing, without going through b43 or bcma.
 * Useful when "FOUND UNSUPPORTED PHY (Analog X, Type Y, Rev Z)" lands
 * in dmesg and we need to know whether that value is real, an artifact
 * of a 16-bit access landing on the wrong half-word, or something
 * else entirely.
 *
 *   ubus  ─►  WLAN SHIM @ 0x10007000  (32-bit BE regs)
 *             │
 *             └─►  AXI backplane @ 0x10004000
 *                    ├─►  ChipCommon  @ 0x10004000  (32-bit BE)
 *                    └─►  d11 MAC core @ 0x10005000  (mixed 16/32-bit BE)
 *
 * The d11 dump reads each 16-bit register *twice* — once at its
 * naive offset, once at offset ^ 2 — and also as a 32-bit word. On a
 * big-endian host driving a 32-bit big-endian backplane, the 16-bit
 * register at logical offset N sits at physical byte offset N ^ 2
 * inside the enclosing word. The dual view makes the asymmetry
 * directly visible.
 *
 * AUTOLOAD priority 99: runs after bcm6362-wlan-shim has brought the
 * SHIM up and after b43 has had its chance to probe and fail. The
 * dump therefore shows the chip state b43 was looking at.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/printk.h>
#include <linux/swab.h>

#define SHIM_BASE	0x10007000
#define SHIM_SIZE	0x100

#define CC_BASE		0x10004000
#define CC_SIZE		0x100

#define MAC_BASE	0x10005000
#define MAC_SIZE	0x1000

/* PCIe Root Complex (BCM6362 ubus1-pcie, used by BCM4352 inbound DMA).
 * The DTS regs property is <0x10e40000 0x10000>; map enough to reach
 * both candidate bridge-register blocks (+0x800 vs +0x2800). */
#define PCIE_RC_BASE	0x10e40000
#define PCIE_RC_SIZE	0x3000

/* PCIe IP block 404 - CONFIG2 (this is what pcie-bcm6328.c writes). */
#define PCIE_CONFIG2_REG		0x408
#define   CONFIG2_BAR1_SIZE_MASK	0x0000000f

/* PCIe RC has the bridge regs in *two* candidate locations:
 *   - Set A: PCIE_BASE + 0x800   (OEM 6362_map_part.h, PcieBridgeRegs layout)
 *   - Set B: PCIE_BASE + 0x2800  (pcie-bcm6328.c driver, where the existing
 *                                 OPT1/OPT2/BAR0_BASEMASK writes go)
 * The two layouts have identical member sequence and identical bit-field
 * semantics; we read both and the one that contains non-zero state is the
 * one that's actually wired on this silicon. */
#define PCIE_BRIDGE_A_BASE	0x0800
#define PCIE_BRIDGE_B_BASE	0x2800

#define BRIDGE_BAR1_REMAP_OFF	0x018  /* bar1Remap (16-bit-aligned variant) */
#define BRIDGE_OPT1_OFF		0x020  /* bridgeOptReg1                      */
#define BRIDGE_OPT2_OFF		0x024  /* bridgeOptReg2                      */
#define BRIDGE_BAR0_BASEMASK_OFF 0x028  /* BAR0 BASE/MASK + enables          */
#define BRIDGE_BAR0_REMAP_OFF	0x02c  /* BAR0 REMAP ADDR                    */
#define BRIDGE_BAR1_BASEMASK_OFF 0x030  /* BAR1 BASE/MASK + enables          */
#define BRIDGE_BAR1_REMAP_ADDR_OFF 0x034 /* BAR1 REMAP ADDR                  */
#define BRIDGE_ERRSTATUS_OFF	0x038  /* bridgeErrStatus                    */
#define BRIDGE_ERRMASK_OFF	0x03c  /* bridgeErrMask                      */
#define BRIDGE_CORE_ERR_STATUS2_OFF 0x040  /* coreErrStatus2 (peer-side errs)  */
#define BRIDGE_CORE_ERR_STATUS1_OFF 0x048  /* coreErrStatus1 (this-side errs)  */

/* Bit-field layout shared between BAR0 and BAR1 *_BASEMASK and *_REMAP. */
#define BASEMASK_BASE_MASK	0xfff00000  /* bits 31..20 = base    */
#define BASEMASK_BASE_SHIFT	20
#define BASEMASK_MASK_MASK	0x0000fff0  /* bits 15..4  = mask    */
#define BASEMASK_MASK_SHIFT	4
#define BASEMASK_SWAP_EN	(1 << 1)
#define BASEMASK_REMAP_EN	(1 << 0)

#define REMAP_ADDR_MASK		0xfff00000  /* bits 31..20 = remap target */
#define REMAP_ADDR_SHIFT	20

/* Old-style bar1Remap (different layout: 16-bit-aligned addr). */
#define BAR1_REMAP_ADDR_MASK	0xffff0000
#define BAR1_REMAP_ADDR_SHIFT	16
#define BAR1_REMAP_REMAP_EN	(1 << 1)
#define BAR1_REMAP_SWAP_EN	(1 << 0)

/* Link status. */
#define PCIE_DLSTATUS_REG	0x1048
#define   DLSTATUS_PHYLINKUP	(1 << 13)

/* d11 MMIO 16-bit register offsets (from b43/b43.h). */
#define B43_MMIO_PHY_VER	0x3E0
#define B43_MMIO_PHY_RADIO	0x3E2

/* SHIM register offsets (from bcm6362-wlan-shim.c). */
#define SHIM_MISC		0x00
#define SHIM_STATUS		0x04
#define SHIM_CC_CONTROL		0x08
#define SHIM_CC_STATUS		0x0c
#define SHIM_MAC_CONTROL	0x10
#define SHIM_MAC_STATUS		0x14
#define SHIM_CC_ID_A		0x18
#define SHIM_MAC_ID_A		0x24

#define TAG	"bcm6362-regdump: "

static void __iomem *shim_io;
static void __iomem *cc_io;
static void __iomem *mac_io;
static void __iomem *pcie_io;

static void dump_cc(void)
{
	pr_info(TAG "=== ChipCommon @ 0x%08x ===\n", CC_BASE);
	pr_info(TAG "CC[0x00] chipid       = 0x%08x  (expect chipnum=0x6362)\n",
		ioread32be(cc_io + 0x00));
	pr_info(TAG "CC[0x04] capabilities = 0x%08x\n",
		ioread32be(cc_io + 0x04));
	pr_info(TAG "CC[0x28] chipctl      = 0x%08x\n",
		ioread32be(cc_io + 0x28));
	pr_info(TAG "CC[0x2C] chipstatus   = 0x%08x\n",
		ioread32be(cc_io + 0x2C));
}

/*
 * Read the BCM4352 (the PCIe endpoint, NOT the on-chip radio) the way
 * bcma does: point the BAR0 window at the backplane enumeration base
 * and read ChipCommon through BAR0+0. PCIe is little-endian, so readl()
 * is the correct accessor and the one bcma uses; swab32 of the same
 * read is printed alongside so a byteswap shows up immediately. If
 * readl gives garbage and swab32 gives a clean chipid (0x43xx) it's
 * endianness; if both are garbage the access/window is wrong, not the
 * byte order.
 */
#define WLAN_PCI_VENDOR		0x14e4
#define WLAN_PCI_DEVICE		0x43b3
#define BCMA_PCI_BAR0_WIN	0x80		/* cfg: backplane addr at BAR0+0 */
#define SI_ENUM_BASE		0x18000000	/* ChipCommon backplane base */
#define CC_EROM_PTR		0xfc		/* ChipCommon erom pointer */

static void dump_pcie_dev(void)
{
	struct pci_dev *pdev;
	void __iomem *bar0;
	u32 win_save, v;
	int i;

	pdev = pci_get_device(WLAN_PCI_VENDOR, WLAN_PCI_DEVICE, NULL);
	if (!pdev) {
		pr_info(TAG "=== PCIe dev %04x:%04x NOT FOUND ===\n",
			WLAN_PCI_VENDOR, WLAN_PCI_DEVICE);
		return;
	}

	pr_info(TAG "=== PCIe dev %s [%04x:%04x] ===\n",
		pci_name(pdev), pdev->vendor, pdev->device);
	pr_info(TAG " BAR0 = 0x%08llx len 0x%llx   BAR2 = 0x%08llx len 0x%llx\n",
		(u64)pci_resource_start(pdev, 0), (u64)pci_resource_len(pdev, 0),
		(u64)pci_resource_start(pdev, 2), (u64)pci_resource_len(pdev, 2));

	if (!pci_resource_start(pdev, 0)) {
		pr_info(TAG " BAR0 unassigned, nothing to map\n");
		pci_dev_put(pdev);
		return;
	}

	bar0 = ioremap(pci_resource_start(pdev, 0), pci_resource_len(pdev, 0));
	if (!bar0) {
		pr_info(TAG " ioremap BAR0 failed\n");
		pci_dev_put(pdev);
		return;
	}

	/* map ChipCommon at BAR0+0, exactly like bcma_host_pci's first read */
	pci_read_config_dword(pdev, BCMA_PCI_BAR0_WIN, &win_save);
	pci_write_config_dword(pdev, BCMA_PCI_BAR0_WIN, SI_ENUM_BASE);

	v = readl(bar0 + 0x00);
	pr_info(TAG " CC_ID    readl=0x%08x  swab32=0x%08x  (chip=0x%04x)\n",
		v, swab32(v), v & 0xffff);
	pr_info(TAG " CC_CAPS  readl=0x%08x  swab32=0x%08x\n",
		readl(bar0 + 0x04), swab32(readl(bar0 + 0x04)));
	v = readl(bar0 + CC_EROM_PTR);
	pr_info(TAG " EROM_PTR readl=0x%08x  swab32=0x%08x\n", v, swab32(v));

	pr_info(TAG " -- BAR0+0x00 (readl | swab32) --\n");
	for (i = 0; i < 0x40; i += 4) {
		v = readl(bar0 + i);
		pr_info(TAG "  +0x%02x = 0x%08x | 0x%08x\n", i, v, swab32(v));
	}

	pci_write_config_dword(pdev, BCMA_PCI_BAR0_WIN, win_save);
	iounmap(bar0);
	pci_dev_put(pdev);
}

/*
 * Dump a window of d11 MAC registers in three views:
 *   - 16-bit BE read at the naive offset
 *   - 16-bit BE read at offset ^ 2 (the BE-backplane fixup)
 *   - 32-bit BE read of the enclosing word
 *
 * If the chip is wired with the address-^2 quirk, the "XOR^2" column
 * holds the values the OEM wl driver sees and the "no XOR" column
 * holds garbage. The 32-bit view shows both halves at once and is
 * the ground truth.
 */
static void dump_mac_window(const char *label, u16 base, u16 len)
{
	int i;

	pr_info(TAG "=== d11 MAC: %s @ +0x%03x (16-bit dual view + 32-bit) ===\n",
		label, base);
	pr_info(TAG " offset | no XOR | XOR^2  | 32-bit word (BE)\n");
	for (i = 0; i < len; i += 2) {
		u16 plain  = ioread16be(mac_io + (base + i));
		u16 xored  = ioread16be(mac_io + ((base + i) ^ 2));
		u32 word32 = ioread32be(mac_io + ((base + i) & ~3));

		pr_info(TAG " +0x%03x | 0x%04x | 0x%04x | 0x%08x\n",
			base + i, plain, xored, word32);
	}
}

/*
 * PCIe Root Complex dump - inbound DMA path observation (read-only).
 *
 * Per the blob audit (b43-add-bcm43xx-ac, kernel-patch/dma-blob-analysis):
 * for the 4352's PCIe + DMA64 engine the descriptor/data translation is
 * high word = 0x80000000, low word = 0, so the addresses leaving the
 * endpoint on the link are the plain DRAM physical addresses. The reads
 * below record the RC state those transactions meet: CONFIG2 BAR1 size,
 * both candidate PcieBridgeRegs blocks (set A at +0x800 per the OEM
 * 6362_map_part.h layout, set B at +0x2800 where pcie-bcm6328.c writes;
 * empirically set B is the populated block on this silicon), and the
 * bridge/core error latches. The most informative read is of the error
 * latches taken right after a fatal DMA error: they record whether and
 * how the RC handled the offending inbound transaction.
 */
static void dump_bridge_set(const char *label, u32 set_base)
{
	u32 r;

	pr_info(TAG "-- %s (PCIe RC + 0x%04x) --\n", label, set_base);

	r = __raw_readl(pcie_io + set_base + BRIDGE_BAR1_REMAP_OFF);
	pr_info(TAG " +0x%04x bar1Remap(old)   = 0x%08x  addr=0x%04x en=%u swap=%u\n",
		set_base + BRIDGE_BAR1_REMAP_OFF, r,
		(r & BAR1_REMAP_ADDR_MASK) >> BAR1_REMAP_ADDR_SHIFT,
		!!(r & BAR1_REMAP_REMAP_EN),
		!!(r & BAR1_REMAP_SWAP_EN));

	r = __raw_readl(pcie_io + set_base + BRIDGE_OPT1_OFF);
	pr_info(TAG " +0x%04x bridgeOptReg1    = 0x%08x\n",
		set_base + BRIDGE_OPT1_OFF, r);

	r = __raw_readl(pcie_io + set_base + BRIDGE_OPT2_OFF);
	pr_info(TAG " +0x%04x bridgeOptReg2    = 0x%08x\n",
		set_base + BRIDGE_OPT2_OFF, r);

	r = __raw_readl(pcie_io + set_base + BRIDGE_BAR0_BASEMASK_OFF);
	pr_info(TAG " +0x%04x BAR0 BASEMASK    = 0x%08x  base=0x%03x mask=0x%03x swap=%u remap_en=%u\n",
		set_base + BRIDGE_BAR0_BASEMASK_OFF, r,
		(r & BASEMASK_BASE_MASK) >> BASEMASK_BASE_SHIFT,
		(r & BASEMASK_MASK_MASK) >> BASEMASK_MASK_SHIFT,
		!!(r & BASEMASK_SWAP_EN),
		!!(r & BASEMASK_REMAP_EN));

	r = __raw_readl(pcie_io + set_base + BRIDGE_BAR0_REMAP_OFF);
	pr_info(TAG " +0x%04x BAR0 REMAP_ADDR  = 0x%08x  addr=0x%03x\n",
		set_base + BRIDGE_BAR0_REMAP_OFF, r,
		(r & REMAP_ADDR_MASK) >> REMAP_ADDR_SHIFT);

	r = __raw_readl(pcie_io + set_base + BRIDGE_BAR1_BASEMASK_OFF);
	pr_info(TAG " +0x%04x BAR1 BASEMASK    = 0x%08x  base=0x%03x mask=0x%03x swap=%u remap_en=%u\n",
		set_base + BRIDGE_BAR1_BASEMASK_OFF, r,
		(r & BASEMASK_BASE_MASK) >> BASEMASK_BASE_SHIFT,
		(r & BASEMASK_MASK_MASK) >> BASEMASK_MASK_SHIFT,
		!!(r & BASEMASK_SWAP_EN),
		!!(r & BASEMASK_REMAP_EN));

	r = __raw_readl(pcie_io + set_base + BRIDGE_BAR1_REMAP_ADDR_OFF);
	pr_info(TAG " +0x%04x BAR1 REMAP_ADDR  = 0x%08x  addr=0x%03x\n",
		set_base + BRIDGE_BAR1_REMAP_ADDR_OFF, r,
		(r & REMAP_ADDR_MASK) >> REMAP_ADDR_SHIFT);

	r = __raw_readl(pcie_io + set_base + BRIDGE_ERRSTATUS_OFF);
	pr_info(TAG " +0x%04x bridgeErrStatus  = 0x%08x\n",
		set_base + BRIDGE_ERRSTATUS_OFF, r);

	r = __raw_readl(pcie_io + set_base + BRIDGE_ERRMASK_OFF);
	pr_info(TAG " +0x%04x bridgeErrMask    = 0x%08x\n",
		set_base + BRIDGE_ERRMASK_OFF, r);

	r = __raw_readl(pcie_io + set_base + BRIDGE_CORE_ERR_STATUS2_OFF);
	pr_info(TAG " +0x%04x coreErrStatus2   = 0x%08x  (peer/UBUS-side errors)\n",
		set_base + BRIDGE_CORE_ERR_STATUS2_OFF, r);

	r = __raw_readl(pcie_io + set_base + BRIDGE_CORE_ERR_STATUS1_OFF);
	pr_info(TAG " +0x%04x coreErrStatus1   = 0x%08x  (RC-side errors)\n",
		set_base + BRIDGE_CORE_ERR_STATUS1_OFF, r);
}

static void dump_pcie_rc(void)
{
	u32 cfg2, dlst;

	pr_info(TAG "=== PCIe RC @ 0x%08x (inbound DMA / BAR1 diagnostic) ===\n",
		PCIE_RC_BASE);

	cfg2 = __raw_readl(pcie_io + PCIE_CONFIG2_REG);
	pr_info(TAG " +0x%04x CONFIG2          = 0x%08x  bar1_size_field=0x%x (cleared by pcie-bcm6328.c)\n",
		PCIE_CONFIG2_REG, cfg2,
		cfg2 & CONFIG2_BAR1_SIZE_MASK);

	dump_bridge_set("Set A: OEM layout       ", PCIE_BRIDGE_A_BASE);
	dump_bridge_set("Set B: Linux 6328 driver", PCIE_BRIDGE_B_BASE);

	dlst = __raw_readl(pcie_io + PCIE_DLSTATUS_REG);
	pr_info(TAG " +0x%04x DL_STATUS        = 0x%08x  phy_linkup=%u\n",
		PCIE_DLSTATUS_REG, dlst, !!(dlst & DLSTATUS_PHYLINKUP));
}

static void dump_mac(void)
{
	pr_info(TAG "=== d11 MAC @ 0x%08x ===\n", MAC_BASE);

	/* Headline: PHY_VER and PHY_RADIO. With the BE fixup active,
	 * one of the two PHY_VER reads should be 0x0408 (Analog 0,
	 * Type 4 = N, Rev 8) — what wl OEM reports in router_info.txt
	 * for the same chip. The other read is the adjacent 16-bit
	 * register and is expected to be 0xE384 (the value b43 has
	 * been printing as "Type 3 UNKNOWN"). */
	pr_info(TAG "PHY_VER   read @ 0x%03x        = 0x%04x\n",
		B43_MMIO_PHY_VER,
		ioread16be(mac_io + B43_MMIO_PHY_VER));
	pr_info(TAG "PHY_VER   read @ 0x%03x (^2)  = 0x%04x\n",
		B43_MMIO_PHY_VER ^ 2,
		ioread16be(mac_io + (B43_MMIO_PHY_VER ^ 2)));
	pr_info(TAG "PHY_RADIO read @ 0x%03x        = 0x%04x\n",
		B43_MMIO_PHY_RADIO,
		ioread16be(mac_io + B43_MMIO_PHY_RADIO));
	pr_info(TAG "PHY_RADIO read @ 0x%03x (^2)  = 0x%04x\n",
		B43_MMIO_PHY_RADIO ^ 2,
		ioread16be(mac_io + (B43_MMIO_PHY_RADIO ^ 2)));

	/* Wider windows. 'low' shows control/status block,
	 * 'phy-area' brackets PHY_VER and PHY_RADIO,
	 * 'ctrl' covers MAC_CTL/MAC_CMD area. */
	dump_mac_window("low",      0x000, 0x010);
	dump_mac_window("ctrl",     0x120, 0x010);
	dump_mac_window("phy-area", 0x3D0, 0x020);
}

static int __init bcm6362_regdump_init(void)
{
	pr_info(TAG "start\n");

	shim_io = ioremap(SHIM_BASE,    SHIM_SIZE);
	cc_io   = ioremap(CC_BASE,      CC_SIZE);
	mac_io  = ioremap(MAC_BASE,     MAC_SIZE);
	pcie_io = ioremap(PCIE_RC_BASE, PCIE_RC_SIZE);
	if (!shim_io || !cc_io || !mac_io || !pcie_io) {
		pr_err(TAG "ioremap failed (shim=%p cc=%p mac=%p pcie=%p)\n",
		       shim_io, cc_io, mac_io, pcie_io);
		goto out_unmap;
	}

	dump_pcie_dev();
	dump_cc();
	dump_mac();
	dump_pcie_rc();

	/* Re-dump the SHIM Control/Status pairs after the MAC accesses.
	 * If the bcma synth routing has worked, SHIM_MAC_CONTROL should
	 * carry whatever bcma_core_enable wrote (typically SICF_CLOCK_EN
	 * with no SICF_WOC_CORE_RESET); SHIM_MAC_STATUS should reflect
	 * that the core is up. Compare with the values from dump_shim
	 * above to see what the routing changed (or didn't). */
	pr_info(TAG "=== SHIM control/status snapshot, post-MAC dump ===\n");
	pr_info(TAG "SHIM CC_CTRL  = 0x%08x   CC_STATUS  = 0x%08x\n",
		ioread32be(shim_io + SHIM_CC_CONTROL),
		ioread32be(shim_io + SHIM_CC_STATUS));
	pr_info(TAG "SHIM MAC_CTRL = 0x%08x   MAC_STATUS = 0x%08x\n",
		ioread32be(shim_io + SHIM_MAC_CONTROL),
		ioread32be(shim_io + SHIM_MAC_STATUS));

	pr_info(TAG "done\n");
	return 0;

out_unmap:
	if (pcie_io)
		iounmap(pcie_io);
	if (mac_io)
		iounmap(mac_io);
	if (cc_io)
		iounmap(cc_io);
	if (shim_io)
		iounmap(shim_io);
	return -ENOMEM;
}

static void __exit bcm6362_regdump_exit(void)
{
	iounmap(pcie_io);
	iounmap(mac_io);
	iounmap(cc_io);
	iounmap(shim_io);
}

module_init(bcm6362_regdump_init);
module_exit(bcm6362_regdump_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BCM6362 WLAN SHIM/d11 register dump (diagnostic)");
