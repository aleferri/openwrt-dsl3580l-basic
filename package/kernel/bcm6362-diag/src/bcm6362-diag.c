// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM6362 WLAN backplane MMIO endian diagnostic, v2.
 *
 * v1 established the endianness picture: the peripheral is BE,
 * readl() byte-swaps on BE MIPS CPU and returns the wrong value,
 * ioread32be() returns the value the peripheral actually stores.
 * Confirmed on this silicon by:
 *   shim CC_ID_A    ioread32be=0x4bf80001  (manuf 0x4bf, id 0x800)
 *   shim MAC_ID_A   ioread32be=0x4bf81201  (manuf 0x4bf, id 0x812)
 *   axi  CC_ID      ioread32be=0x22016362  (chip 0x6362, rev 1)
 *   axi  EROM_PTR   ioread32be=0x10007018
 *
 * EROM_PTR points back into the SHIM at offset 0x18 (= SHIM_CC_ID_A).
 * The first u32 there is already a valid AI EROM ChipCommon
 * component descriptor (DMP_DESC_COMPONENT bit set, partnum 0x800,
 * designer 0x4bf). The open question is whether the rest of a
 * proper AI EROM (component_B, master ports, slave address
 * descriptors, EOT) lives at 0x1c.., or whether 0x10007018 is just
 * a register that happens to read as a valid descriptor and bcma's
 * EROM scan would EILSEQ on the next u32 anyway.
 *
 * v2: dump the full SHIM region 0x00..0xfc, one u32 per line, via
 * ioread32be (the accessor that returns the peripheral's value).
 * Output filter: read the u32 stream starting at 0x18 as if it
 * were an AI EROM and see how far it parses.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>

#define SHIM_BASE	0x10007000u
#define SHIM_SIZE	0x100u

/* AI EROM descriptor type field (bits 0-3 of every entry). */
#define DMP_DESC_TYPE_MSK	0x0000000fu
#define  DMP_DESC_EMPTY		0x0u
#define  DMP_DESC_COMPONENT	0x1u
#define  DMP_DESC_MASTER_PORT	0x3u
#define  DMP_DESC_ADDRESS	0x5u
#define  DMP_DESC_EOT		0xfu

static const char *desc_name(u32 v)
{
	switch (v & DMP_DESC_TYPE_MSK) {
	case DMP_DESC_EMPTY:		return "EMPTY    ";
	case DMP_DESC_COMPONENT:	return "COMPONENT";
	case DMP_DESC_MASTER_PORT:	return "MASTER   ";
	case DMP_DESC_ADDRESS:		return "ADDRESS  ";
	case DMP_DESC_EOT:		return "EOT      ";
	default:			return "other?   ";
	}
}

static int __init diag_init(void)
{
	void __iomem *shim;
	u32 off, v;

	shim = ioremap(SHIM_BASE, SHIM_SIZE);
	if (!shim) {
		pr_err("bcm6362-diag-v2: ioremap SHIM @0x%08x failed\n",
		       SHIM_BASE);
		return -ENOMEM;
	}

	pr_info("bcm6362-diag-v2: --- begin full SHIM dump (ioread32be) ---\n");
	for (off = 0x00; off < SHIM_SIZE; off += 4) {
		v = ioread32be(shim + off);
		if (off < 0x18) {
			/* Below 0x18 is the regular SHIM control/status
			 * area, already labelled in v1; print value only. */
			pr_info("bcm6362-diag-v2: @0x%08x +0x%02x = 0x%08x\n",
				SHIM_BASE, off, v);
		} else {
			/* From 0x18 on, treat as a candidate AI EROM
			 * stream and annotate each u32 with its
			 * descriptor type (only meaningful if there
			 * really is an EROM here). */
			pr_info("bcm6362-diag-v2: @0x%08x +0x%02x = 0x%08x  [%s]\n",
				SHIM_BASE, off, v, desc_name(v));
		}
	}
	pr_info("bcm6362-diag-v2: --- end full SHIM dump ---\n");

	iounmap(shim);
	return -ENODEV;
}

module_init(diag_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BCM6362 WLAN backplane MMIO endian diagnostic v2");
MODULE_AUTHOR("bcm6362-diag-v2");
