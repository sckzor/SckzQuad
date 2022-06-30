################################################################################
#
# selftest: A package that allows the quadcopter to check it's hardware.
#
#################################################################################

SELFTEST_VERSION = 1.0
SELFTEST_SITE = ./package/selftest/src
SELFTEST_SITE_METHOD = local

define SELFTEST_BUILD_CMDS
	$(MAKE) CC="$(TARGET_CC)" LD="$(TARGET_LD)" -C $(@D)
endef

define SELFTEST_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/selftest $(TARGET_DIR)/usr/bin
endef

$(eval $(generic-package))
