################################################################################
#
# pidtest: A package that allows the quadcopter to check it's hardware.
#
#################################################################################

PIDTEST_VERSION = 1.0
PIDTEST_SITE = ./package/pidtest/src
PIDTEST_SITE_METHOD = local

define PIDTEST_BUILD_CMDS
	$(MAKE) CC="$(TARGET_CC)" LD="$(TARGET_LD)" -C $(@D)
endef

define PIDTEST_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/pidtest $(TARGET_DIR)/usr/bin
endef

$(eval $(generic-package))
