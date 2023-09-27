/**
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "cyw43_drv.h"

static struct cyw43_dev *cyw43_device;

#if defined(CONFIG_WIFI_CYW43_SHELL)
void cyw43_shell_register(struct cyw43_dev *dev)
{
	/* only one instance supported */
	if (cyw43_device) {
		return;
	}

	cyw43_device = dev;
}

static int cyw43_shell_led(const struct shell *sh, size_t argc,
				  char **argv)
{
	if (cyw43_device == NULL) {
	  shell_print(sh, "no cyw43_device device registered");
	  return -ENOEXEC;
	}

	if (argc != 2) {
	  shell_help(sh);
	  return -ENOEXEC;
	}

	cyw43_lock(cyw43_device);

	if (!strcmp(argv[1], "on")) {
	  shell_print(sh, "Enabling LED");
	} 
	else if (!strcmp(argv[1], "off")) {
	  shell_print(sh, "Disabling LED");
	}
	else {
	  shell_help(sh);
	  return -ENOEXEC;
	}
	
	cyw43_unlock(cyw43_device);

	return 0;
}

static int cyw43_shell_reset(const struct shell *sh, size_t argc,
				  char **argv)
{
	if (cyw43_device == NULL) {
	  shell_print(sh, "no cyw43_device device registered");
	  return -ENOEXEC;
	}

	if (argc != 1) {
	  shell_help(sh);
	  return -ENOEXEC;
	}

	cyw43_lock(cyw43_device);

	shell_print(sh, "Resetting CYW43 module");
	
	cyw43_unlock(cyw43_device);

	return 0;

}
#endif

#if defined(CONFIG_WIFI_CYW43_SHELL)
SHELL_STATIC_SUBCMD_SET_CREATE(cyw43_shell,
	SHELL_CMD(led, NULL, "cyw43 led <on|off>", cyw43_shell_led),
	SHELL_CMD(reset, NULL, "cyw43 reset", cyw43_shell_reset),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(cyw43, &cyw43_shell, "CYW43 debug shell", NULL);
#endif
