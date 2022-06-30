################################################################################
#
# cyclictest: A package that allows latency tests in the realtime linux kernel
#
#################################################################################

CYCLICTEST_VERSION = 1.0
CYCLICTEST_SITE = ./package/cyclictest/rt-tests
CYCLICTEST_SITE_METHOD = local

define CYCLICTEST_BUILD_CMDS
	$(MAKE) CC="$(TARGET_CC)" LD="$(TARGET_LD)" -C $(@D)
endef

define CYCLICTEST_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/cyclictest $(TARGET_DIR)/usr/bin
endef

$(eval $(generic-package))
