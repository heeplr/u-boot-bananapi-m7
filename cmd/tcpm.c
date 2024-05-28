// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2024 Collabora
 */

#include <common.h>
#include <command.h>
#include <errno.h>
#include <dm.h>
#include <dm/uclass-internal.h>
#include <usb/tcpm.h>

#define LIMIT_DEV	32
#define LIMIT_PARENT	20

static struct udevice *currdev;

static int failure(int ret)
{
	printf("Error: %d (%s)\n", ret, errno_str(ret));

	return CMD_RET_FAILURE;
}

static int do_dev(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	char *name;
	int ret = -ENODEV;

	switch (argc) {
	case 2:
		name = argv[1];
		ret = tcpm_get(name, &currdev);
		if (ret) {
			printf("Can't get TCPM: %s!\n", name);
			return failure(ret);
		}
	case 1:
		if (!currdev) {
			printf("TCPM device is not set!\n\n");
			return CMD_RET_USAGE;
		}

		printf("dev: %d @ %s\n", dev_seq(currdev), currdev->name);
	}

	return CMD_RET_SUCCESS;
}

static int do_list(struct cmd_tbl *cmdtp, int flag, int argc,
		   char *const argv[])
{
	struct udevice *dev;
	int ret, err = 0;

	printf("| %-*.*s| %-*.*s| %s @ %s\n",
	       LIMIT_DEV, LIMIT_DEV, "Name",
	       LIMIT_PARENT, LIMIT_PARENT, "Parent name",
	       "Parent uclass", "seq");

	for (ret = uclass_first_device_check(UCLASS_TCPM, &dev); dev;
	     ret = uclass_next_device_check(&dev)) {
		if (ret)
			err = ret;

		printf("| %-*.*s| %-*.*s| %s @ %d | status: %i\n",
		       LIMIT_DEV, LIMIT_DEV, dev->name,
		       LIMIT_PARENT, LIMIT_PARENT, dev->parent->name,
		       dev_get_uclass_name(dev->parent), dev_seq(dev->parent),
		       ret);
	}

	if (err)
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

static int do_info(struct cmd_tbl *cmdtp, int flag, int argc,
		   char *const argv[])
{
	if (!currdev) {
		printf("First, set the TCPM device!\n");
		return CMD_RET_USAGE;
	}

	return tcpm_print_info(currdev);
}

static struct cmd_tbl subcmd[] = {
	U_BOOT_CMD_MKENT(dev, 2, 1, do_dev, "", ""),
	U_BOOT_CMD_MKENT(list, 1, 1, do_list, "", ""),
	U_BOOT_CMD_MKENT(info, 1, 1, do_info, "", ""),
};

static int do_tcpm(struct cmd_tbl *cmdtp, int flag, int argc,
		   char *const argv[])
{
	struct cmd_tbl *cmd;

	argc--;
	argv++;

	cmd = find_cmd_tbl(argv[0], subcmd, ARRAY_SIZE(subcmd));
	if (cmd == NULL || argc > cmd->maxargs)
		return CMD_RET_USAGE;

	return cmd->cmd(cmdtp, flag, argc, argv);
}

 /**************************************************/

U_BOOT_CMD(tcpm, CONFIG_SYS_MAXARGS, 1, do_tcpm,
	"TCPM sub-system",
	"list          - list TCPM devices\n"
	"tcpm dev [name]    - show or [set] operating TCPM device\n"
	"tcpm info          - dump information\n"
);
