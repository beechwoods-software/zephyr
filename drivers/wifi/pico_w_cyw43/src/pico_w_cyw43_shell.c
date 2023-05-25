/**
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "pico_w_cyw43_drv.h"

static struct pico_w_cyw43_dev *pico_w_cyw43;

#if defined(CONFIG_WIFI_RPIPICOWCYW43_SHELL)
void pico_w_cyw43_shell_register(struct pico_w_cyw43_dev *dev)
{
	/* only one instance supported */
	if (pico_w_cyw43) {
		return;
	}

	pico_w_cyw43 = dev;
}
#endif

static int pico_w_cyw43_shell_led(const struct shell *sh, size_t argc,
				  char **argv)
{
	if (pico_w_cyw43 == NULL) {
	  shell_print(sh, "no pico_w_cyw43 device registered");
	  return -ENOEXEC;
	}

	if (argc != 2) {
	  shell_help(sh);
	  return -ENOEXEC;
	}

	pico_w_cyw43_lock(pico_w_cyw43);

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
	
	pico_w_cyw43_unlock(pico_w_cyw43);

	return 0;
}

static int pico_w_cyw43_shell_reset(const struct shell *sh, size_t argc,
				  char **argv)
{
	if (pico_w_cyw43 == NULL) {
	  shell_print(sh, "no pico_w_cyw43 device registered");
	  return -ENOEXEC;
	}

	if (argc != 1) {
	  shell_help(sh);
	  return -ENOEXEC;
	}

	pico_w_cyw43_lock(pico_w_cyw43);

	shell_print(sh, "Resetting CYW43 module");
	
	pico_w_cyw43_unlock(pico_w_cyw43);

	return 0;

}


void print_task_info(const struct k_thread *thread, void *user_data)
{
  printf("Task Name: %s\n", k_thread_name_get(thread));
    printf("Priority: %d\n", k_thread_priority_get(thread));
    printf("\n");
}

void picw_w_cyw43_print_tasks(void)
{
    printf("Currently running tasks:\n");
    k_thread_foreach(print_task_info, NULL);
}



SHELL_STATIC_SUBCMD_SET_CREATE(pico_w_cyw43_shell,
	SHELL_CMD(led, NULL, "cyw43 led <on|off>", pico_w_cyw43_shell_led),
	SHELL_CMD(reset, NULL, "cyw43 reset", pico_w_cyw43_shell_reset),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(cyw43, &pico_w_cyw43_shell, "CYW43 debug shell", NULL);
