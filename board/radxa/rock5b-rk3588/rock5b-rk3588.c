// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023-2024 Collabora Ltd.
 */

#include <fdtdec.h>
#include <fdt_support.h>
#include <usb/tcpm.h>

#ifdef CONFIG_MISC_INIT_R
int misc_init_r(void)
{
	struct udevice *dev;
	int ret;

	ret = tcpm_get("usb-typec@22", &dev);
	if (ret) {
		printf("Failed to probe Type-C controller\n");
		return 0;
	}

	return 0;
}
#endif

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
	if (IS_ENABLED(CONFIG_MISC_INIT_R))
		fdt_status_okay_by_compatible(blob, "fcs,fusb302");
	return 0;
}
#endif
