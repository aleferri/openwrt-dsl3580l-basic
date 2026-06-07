include $(TOPDIR)/rules.mk

PKG_NAME:=bcm6362-diag
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0-or-later

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

define KernelPackage/bcm6362-diag
  SUBMENU:=Other modules
  TITLE:=BCM6362 WLAN backplane MMIO endian diagnostic
  DEPENDS:=@TARGET_bmips_bcm6362
  FILES:=$(PKG_BUILD_DIR)/bcm6362-diag.ko
  AUTOLOAD:=
endef

define KernelPackage/bcm6362-diag/description
  One-shot diagnostic kernel module. Reads SHIM @0x10007000 and AXI
  ChipCommon @0x10004000 side-by-side through __raw_readl, readl
  and ioread32be. Refuses to stay loaded; each insmod replays the
  dump in dmesg. No writes, no MMIO claim, safe to load on a live
  kernel where bcma-host-soc has already failed to probe.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(KERNEL_MAKE) M="$(PKG_BUILD_DIR)" modules
endef

$(eval $(call KernelPackage,bcm6362-diag))
