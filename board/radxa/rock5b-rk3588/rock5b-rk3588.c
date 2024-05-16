// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023 Collabora Ltd.
 */

#include <fdtdec.h>
#include <fdt_support.h>
#include <usb/tcpm.h>

#ifdef CONFIG_MISC_INIT_R
int misc_init_r(void) {
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
int rock5b_add_reserved_memory_fdt_nodes(void *new_blob)
{
	struct fdt_memory gap1 = {
		.start = 0x3fc000000,
		.end = 0x3fc4fffff,
	};
	struct fdt_memory gap2 = {
		.start = 0x3fff00000,
		.end = 0x3ffffffff,
	};
	unsigned long flags = FDTDEC_RESERVED_MEMORY_NO_MAP;
	unsigned int ret;

	/*
	 * Inject the reserved-memory nodes into the DTS
	 */
	ret = fdtdec_add_reserved_memory(new_blob, "gap1", &gap1,  NULL, 0,
					 NULL, flags);
	if (ret)
		return ret;

	return fdtdec_add_reserved_memory(new_blob, "gap2", &gap2,  NULL, 0,
					  NULL, flags);
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
#ifdef CONFIG_MISC_INIT_R
	fdt_status_okay_by_compatible(blob, "fcs,fusb302");
#endif
	return rock5b_add_reserved_memory_fdt_nodes(blob);
}
#endif
