# SPDX-License-Identifier: GPL-2.0-or-later
include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=bcm6362-regdump
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0-or-later

include $(INCLUDE_DIR)/package.mk

define KernelPackage/bcm6362-regdump
  SUBMENU:=Other modules
  TITLE:=BCM6362 WLAN SHIM/d11 register dump (diagnostic)
  FILES:=$(PKG_BUILD_DIR)/bcm6362-regdump.ko
  AUTOLOAD:=$(call AutoLoad,99,bcm6362-regdump,1)
  DEPENDS:=@TARGET_bmips_bcm6362
endef

define KernelPackage/bcm6362-regdump/description
  Diagnostic-only kernel module. On insmod, ioremaps the on-chip
  WLAN SHIM, the AXI ChipCommon core, and the d11 MAC core, then
  dumps a curated set of registers to dmesg. Includes a dual-view
  (no-XOR vs. XOR^2) read of the PHY area so the PHY_VER value the
  chip actually exposes can be verified independently of b43.
endef

MAKE_OPTS:= \
	$(KERNEL_MAKE_FLAGS) \
	M="$(PKG_BUILD_DIR)"

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	+$(MAKE) -C "$(LINUX_DIR)" $(MAKE_OPTS) modules
endef

$(eval $(call KernelPackage,bcm6362-regdump))
