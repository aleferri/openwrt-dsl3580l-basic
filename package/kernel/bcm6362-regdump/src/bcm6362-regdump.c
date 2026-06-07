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
#include <linux/printk.h>

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

/*
 * BAR1 write-enable knob and parameters.
 *
 * When write_bar1 is set at insmod time, the module programs the PCIe
 * RC bridge's BAR1 inbound translation window BEFORE running the
 * register dump - so the dump that follows shows the post-write state.
 *
 * Default values target the BCM4352 inbound DMA path on this board:
 * the 4352 emits 32-bit MEM TLPs with translation = 0x40000000 in the
 * low word of the descriptor address (see b43 patch 823), so the
 * bridge needs a match window over PCIe [0x40000000 .. 0x4fffffff]
 * remapped to DRAM [0x00000000 .. 0x0fffffff]. Encoding in the BCM
 * bridge regs uses base = top12(start) and "mask" = top12(end) (not a
 * true bitmask), so 0x400 / 0x4ff means [0x40000000 .. 0x4fffffff].
 *
 * To experiment with a different window or a different remap target
 * just pass new values at insmod time, e.g. a smaller 64 MB window:
 *   insmod bcm6362-regdump.ko write_bar1=1 bar1_mask=0x43f
 * or remapping to a non-zero DRAM offset:
 *   insmod bcm6362-regdump.ko write_bar1=1 bar1_remap=0x008
 */
static bool write_bar1;
module_param(write_bar1, bool, 0444);
MODULE_PARM_DESC(write_bar1, "Program BAR1 inbound at insmod (default off)");

static unsigned int bar1_base = 0x400;
module_param(bar1_base, uint, 0444);
MODULE_PARM_DESC(bar1_base, "BAR1 BASE field, top 12 bits of PCIe window start (default 0x400)");

static unsigned int bar1_mask = 0x4ff;
module_param(bar1_mask, uint, 0444);
MODULE_PARM_DESC(bar1_mask, "BAR1 MASK field, top 12 bits of PCIe window end inclusive (default 0x4ff)");

static unsigned int bar1_remap;
module_param(bar1_remap, uint, 0444);
MODULE_PARM_DESC(bar1_remap, "BAR1 REMAP target, top 12 bits of DRAM destination (default 0x000)");

static bool bar1_swap;
module_param(bar1_swap, bool, 0444);
MODULE_PARM_DESC(bar1_swap, "Set BAR1 swap_enable bit (default off)");

/*
 * CONFIG2 BAR1 SIZE field write.
 *
 * Separate from the bridge-side BAR1 BASEMASK/REMAP setup. The
 * BCM6362 PCIe RC has two distinct BAR1 layers that must BOTH be
 * configured for inbound DMA to actually land in DRAM:
 *
 *  - PCIe-side BAR1 (CONFIG2 SIZE field at offset 0x408): the RC
 *    advertises this BAR to the PCIe link as an aperture that
 *    claims a contiguous range of PCIe addresses. The OEM header
 *    says SIZE=0 means BAR1_DISABLE; non-zero values encode the
 *    size as a 4-bit code. The pcie-bcm6328.c driver clears this
 *    field to 0 after reset, which makes the RC NOT claim inbound
 *    addresses on the PCIe side - so even if the bridge BASEMASK
 *    matches, the TLP never reaches the bridge.
 *
 *  - UBUS-side bridge translation (BAR1 BASEMASK + REMAP_ADDR at
 *    +0x2830/+0x2834): for the claimed PCIe addresses, this
 *    converts them into UBUS/DRAM addresses.
 *
 * Encoding for SIZE: the pcie-bcm6318.c sibling driver defines
 * RC_BAR_CFG_LO_SIZE_256MB = 0xd (its register is at a different
 * offset because it's ubus2 architecture, but the SIZE field
 * encoding is most likely the same Broadcom IP convention). So
 * the default here is 0xd to match the 256 MB window the bridge
 * BAR1 covers. Other values are 1-byte experiments away.
 */
static unsigned int config2_bar1_size;
module_param(config2_bar1_size, uint, 0444);
MODULE_PARM_DESC(config2_bar1_size, "Write CONFIG2 BAR1 SIZE field at +0x408 (0=skip, 0xd=256MB guess, range 0x0-0xf)");

static void __iomem *shim_io;
static void __iomem *cc_io;
static void __iomem *mac_io;
static void __iomem *pcie_io;

static void dump_shim(void)
{
	int i;

	pr_info(TAG "=== SHIM @ 0x%08x ===\n", SHIM_BASE);
	for (i = 0; i < SHIM_SIZE; i += 4)
		pr_info(TAG "SHIM[0x%02x] = 0x%08x\n",
			i, ioread32be(shim_io + i));
}

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
 * PCIe Root Complex dump - inbound DMA path diagnostic.
 *
 * We're trying to figure out whether the BCM4352 (PCIe endpoint) is
 * hitting a configured BAR1 inbound window when it emits descriptor
 * fetches at translation=0x40000000, or whether that window is unconfigured
 * and the chip's transactions get master-aborted on the RC side - the
 * suspected cause of "Fatal DMA error: 0x00000400 (I_PC)" on phy1.
 *
 * The PcieBridgeRegs block contains BAR1 BASEMASK + REMAP ADDR registers
 * that gate which PCIe-side addresses are remapped onto UBUS/DRAM.
 *
 * We read both candidate blocks (set A at +0x800 from PCIe RC base, set B
 * at +0x2800) because the existing pcie-bcm6328.c driver writes its
 * OPT1/OPT2/BAR0_BASEMASK at +0x2820 while the OEM 6362_map_part.h
 * describes the same fields at +0x820. Only one of the two is the real
 * one on this silicon - the empirical dump tells us which.
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

/*
 * Program CONFIG2 BAR1 SIZE field. Read-modify-write that only
 * touches bits 0..3 of the register at offset 0x408; preserves all
 * other bits including the ones the pcie-bcm6328.c driver sets
 * elsewhere in CONFIG2. The dump that follows shows whether the
 * field actually latched the requested value.
 */
static void write_pcie_config2(void)
{
	u32 cur, want;

	cur  = __raw_readl(pcie_io + PCIE_CONFIG2_REG);
	want = (cur & ~CONFIG2_BAR1_SIZE_MASK) | (config2_bar1_size & CONFIG2_BAR1_SIZE_MASK);

	pr_info(TAG ">>> write_config2_bar1_size active <<<\n");
	pr_info(TAG "  current  CONFIG2 = 0x%08x  bar1_size_field=0x%x\n",
		cur, cur & CONFIG2_BAR1_SIZE_MASK);
	pr_info(TAG "  writing  CONFIG2 = 0x%08x  bar1_size_field=0x%x\n",
		want, want & CONFIG2_BAR1_SIZE_MASK);

	__raw_writel(want, pcie_io + PCIE_CONFIG2_REG);

	cur = __raw_readl(pcie_io + PCIE_CONFIG2_REG);
	pr_info(TAG "  readback CONFIG2 = 0x%08x  bar1_size_field=0x%x  %s\n",
		cur, cur & CONFIG2_BAR1_SIZE_MASK,
		(cur == want) ? "(== want)" : "(MISMATCH)");
	pr_info(TAG "<<< write_config2_bar1_size done >>>\n");
}

/*
 * Program BAR1 inbound translation. Writes BASEMASK and REMAP_ADDR
 * at the Set B locations (PCIe RC + 0x2830 / +0x2834) which the
 * regdump confirmed empirically are the populated register block on
 * this silicon. Reads back what was written and prints both, so any
 * write-through failure (wrong endianness, register that ignores the
 * write, etc.) becomes immediately visible.
 */
static void write_pcie_bar1(void)
{
	u32 want_bm, want_ra, got_bm, got_ra;

	want_bm = ((bar1_base & 0xfff) << BASEMASK_BASE_SHIFT) |
		  ((bar1_mask & 0xfff) << BASEMASK_MASK_SHIFT) |
		  BASEMASK_REMAP_EN |
		  (bar1_swap ? BASEMASK_SWAP_EN : 0);
	want_ra = (bar1_remap & 0xfff) << REMAP_ADDR_SHIFT;

	pr_info(TAG ">>> write_bar1 active <<<\n");
	pr_info(TAG "  PCIe inbound window: PCIe [0x%05x00000 .. 0x%05xfffff]  ->  DRAM [0x%05x00000 .. ]\n",
		bar1_base, bar1_mask, bar1_remap);
	pr_info(TAG "  writing  +0x%04x BAR1 BASEMASK   = 0x%08x  (base=0x%03x mask=0x%03x swap=%u remap_en=1)\n",
		PCIE_BRIDGE_B_BASE + BRIDGE_BAR1_BASEMASK_OFF, want_bm,
		bar1_base & 0xfff, bar1_mask & 0xfff, bar1_swap);
	pr_info(TAG "  writing  +0x%04x BAR1 REMAP_ADDR = 0x%08x  (addr=0x%03x)\n",
		PCIE_BRIDGE_B_BASE + BRIDGE_BAR1_REMAP_ADDR_OFF, want_ra,
		bar1_remap & 0xfff);

	__raw_writel(want_bm,
		pcie_io + PCIE_BRIDGE_B_BASE + BRIDGE_BAR1_BASEMASK_OFF);
	__raw_writel(want_ra,
		pcie_io + PCIE_BRIDGE_B_BASE + BRIDGE_BAR1_REMAP_ADDR_OFF);

	/* Posted-write barrier: read back to flush the store and to verify
	 * what the silicon actually latched (in case some bits are RO or
	 * encoded differently than we expect). */
	got_bm = __raw_readl(pcie_io + PCIE_BRIDGE_B_BASE + BRIDGE_BAR1_BASEMASK_OFF);
	got_ra = __raw_readl(pcie_io + PCIE_BRIDGE_B_BASE + BRIDGE_BAR1_REMAP_ADDR_OFF);

	pr_info(TAG "  readback  BAR1 BASEMASK   = 0x%08x  %s\n",
		got_bm, (got_bm == want_bm) ? "(== want)" : "(MISMATCH)");
	pr_info(TAG "  readback  BAR1 REMAP_ADDR = 0x%08x  %s\n",
		got_ra, (got_ra == want_ra) ? "(== want)" : "(MISMATCH)");
	pr_info(TAG "<<< write_bar1 done >>>\n");
}

static void dump_pcie_rc(void)
{
	u32 cfg2, dlst;

	pr_info(TAG "=== PCIe RC @ 0x%08x (inbound DMA / BAR1 diagnostic) ===\n",
		PCIE_RC_BASE);

	cfg2 = __raw_readl(pcie_io + PCIE_CONFIG2_REG);
	pr_info(TAG " +0x%04x CONFIG2          = 0x%08x  bar1_size_field=0x%x (driver clobbers to 0)\n",
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

	dump_shim();
	dump_cc();
	dump_mac();
	if (config2_bar1_size)
		write_pcie_config2();
	if (write_bar1)
		write_pcie_bar1();
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
