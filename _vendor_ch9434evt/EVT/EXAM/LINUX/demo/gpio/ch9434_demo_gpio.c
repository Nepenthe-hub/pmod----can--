/*
 * CH9434A/D/M gpio application example
 *
 * Copyright (C) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web: http://wch.cn
 * Author: WCH <tech@wch.cn>
 *
 * Cross-compile with cross-gcc -I /path/to/cross-kernel/include
 *
 * V1.0 - initial version
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/serial.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include "ch9434_lib.h"

int main(int argc, char *argv[])
{
	int fd, ret;
	char c;
	int gpio_number;
	int ioval;

	if (argc != 3)
		return -1;

	gpio_number = atoi(argv[2]);
	if (gpio_number < 0)
		return -1;

	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		perror("open");
		return -1;
	}

	while (1) {
		if (c != '\n') {
			printf("press e to enable all GPIO, d to disable all gpio, o to set direction output,\n"
			       "i to set direction input, h to set gpio high, l to set gpio low, g to get gpio value,\n"
			       "q to exit app.\n");
		}
		scanf("%c", &c);
		if (c == 'q')
			break;

		switch (c) {
		case 'e':
			ret = ch943x_single_gpioenable(fd, gpio_number, 1);
			if (ret < 0) {
				printf("GPIO%d enable/disable failed. ret:%d\n",
				       gpio_number, ret);
				break;
			} else
				printf("GPIO%d enable/disable Success.\n",
				       gpio_number);
			break;
		case 'd':
			ret = ch943x_single_gpioenable(fd, gpio_number, 0);
			if (ret < 0) {
				printf("GPIO%d disable failed.\n",
				       gpio_number);
				break;
			} else {
				printf("GPIO%d disable Success.\n",
				       gpio_number);
			}
			break;
		case 'o':
			ret = ch943x_single_gpioconfig(
				fd, gpio_number, OUTPUT,
				CH943X_IOMODE_PUSH_PULL);
			if (ret < 0) {
				printf("GPIO%d config failed.\n",
				       gpio_number);
				break;
			}
			break;
		case 'i':
			ret = ch943x_single_gpioconfig(
				fd, gpio_number, INPUT,
				CH943X_IOMODE_PULLUP_INPUT);
			if (ret < 0) {
				printf("GPIO%d config failed.\n",
				       gpio_number);
				break;
			}
			break;
		case 'h':
			ret = ch943x_single_gpioset(fd, gpio_number, 1);
			if (ret < 0) {
				printf("GPIO%d set failed.\n",
				       gpio_number);
				break;
			}
			break;
		case 'l':
			ret = ch943x_single_gpioset(fd, gpio_number, 0);
			if (ret < 0) {
				printf("GPIO%d set failed.\n",
				       gpio_number);
				break;
			}
			break;
		case 'g':
			ioval = ch943x_single_gpioget(fd, gpio_number);
			if (ioval < 0) {
				printf("GPIO%d get failed.\n",
				       gpio_number);
				break;
			} else
				printf("gpio input value: %d\n", ioval);
			break;
		default:
			break;
		}
	}

	close(fd);
	return 0;
}
