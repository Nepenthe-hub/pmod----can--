/*
 * SPI/I2C multifunction driver for Quad UARTs chip CH9434A/D/M,
 * CH9434D also supports SPI/I2C to serial/CAN/GPIO function.
 *
 * Copyright (C) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web:      http://wch.cn
 * Author:   WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Update Log:
 * V1.0 - initial version
 * V1.1 - change RS485 setting and getting method, add procfs for dump registers
 *      - add support for CH9434A with continuous spi transmission
 * V1.2 - add support for kernel version beyond 5.12.18
 * V1.3 - improve spi transfer compatibility on some platforms
 * V1.4 - add support for CH9434D
 */

#define DEBUG
#define VERBOSE_DEBUG

#undef DEBUG
#undef VERBOSE_DEBUG

#include "ch9434.h"

#ifndef PORT_SC16IS7XX
#define PORT_SC16IS7XX 128
#endif

static struct list_head g_private_head;

struct ch943x_devtype ch943x_devtype = {
	.name = "CH943X",
	.nr_uart = 4,
};

irqreturn_t ch943x_ist_top(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}

irqreturn_t ch943x_ist(int irq, void *dev_id)
{
	struct ch943x_port *s = (struct ch943x_port *)dev_id;
	int i;

	dev_dbg(s->dev, "ch943x_ist interrupt enter...\n");

	for (i = 0; i < s->uart.nr; ++i) {
		if (atomic_read(&s->p[i].isopen) == 1)
			ch943x_port_irq(s, i);
	}
	if (s->chiptype == CHIP_CH9434D) {
#ifdef CH9434D_CAN_ON
		if (atomic_read(&s->priv->can_isopen) == 1) {
			ch943x_can_irq(s);
		}
#endif
	}

	dev_dbg(s->dev, "%s end\n", __func__);

	return IRQ_HANDLED;
}

int ch943x_io_enable(struct ch943x_port *s)
{
	dev_dbg(s->dev, "%s\n", __func__);

	ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_W_EN, CH9434D_MUL_INT_ADD,
			  1); /* Interrupt Enable */
	ch943x_iofunc_cfg(s, CH9434D_IO_DEF_W_EN, CH9434D_DEF_U1_ADD,
			  1); /* UART0 Tx/Rx Enable */
	ch943x_iofunc_cfg(s, CH9434D_IO_DEF_W_EN, CH9434D_DEF_U2_ADD,
			  1); /* UART1 Tx/Rx Enable */
	ch943x_iofunc_cfg(s, CH9434D_IO_DEF_W_EN, CH9434D_DEF_U3_ADD,
			  1); /* UART2 Tx/Rx Enable */
	ch943x_iofunc_cfg(s, CH9434D_IO_DEF_W_EN, CH9434D_DEF_U4_ADD,
			  1); /* UART3 Tx/Rx Enable */
#if defined(EXTERN_CLOCK)
	ch943x_iofunc_cfg(s, CH9434D_IO_DEF_W_EN, CH9434D_DEF_HSE_ADD,
			  1); /* External Clock Enable */
#elif defined(INTERNAL_CLOCK)
#endif

#if defined(CH9434D_CAN_ON)
	ch943x_iofunc_cfg(s, CH9434D_IO_DEF_W_EN, CH9434D_DEF_CAN_ADD,
			  1); /* CAN Tx/Rx Enable */
#endif

#if defined(CH9434D_TNOW0_ON) && !defined(CH9434D_CAN_ON)
	ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_W_EN, CH9434D_MUL_TNOW0_ADD,
			  1); /* TNOW0 Enable */
#endif
#if defined(CH9434D_TNOW1_ON) && !defined(CH9434D_CAN_ON)
	ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_W_EN, CH9434D_MUL_TNOW1_ADD,
			  1); /* TNOW1 Enable */
#endif
#if defined(CH9434D_TNOW2_ON)
	ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_W_EN, CH9434D_MUL_TNOW2_ADD,
			  1); /* TNOW2 Enable */
#endif
#if defined(CH9434D_TNOW3_ON) && defined(INTERNAL_CLOCK)
	ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_W_EN, CH9434D_MUL_TNOW3_ADD,
			  1); /* TNOW3 Enable */
#endif

#if !defined(CH9434D_TNOW0_ON) && !defined(CH9434D_CAN_ON)
	ch943x_iofunc_cfg(s, CH9434D_IO_DEF_W_EN, CH9434D_DEF_CTS0_ADD,
			  1); /* Modem CTS0 Enable */
#endif
#if !defined(CH9434D_TNOW2_ON)
	ch943x_iofunc_cfg(s, CH9434D_IO_DEF_W_EN, CH9434D_DEF_CTS3_ADD,
			  1); /* Modem CTS3 Enable */
#endif
#if !defined(CH9434D_TNOW1_ON) && !defined(CH9434D_CAN_ON)
	ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_W_EN, CH9434D_MUL_RTS0_ADD,
			  1); /* Modem RTS0 Enable */
#endif

	return 0;
}

static int ch943x_hw_test(struct ch943x_port *s)
{
	u8 val;
	int i;

	dev_dbg(s->dev, "%s\n", __func__);

	for (i = 0; i < 4; i++) {
		ch943x_reg_write(s, CH943X_SPR_REG + (i * 0x10), 0x55);
		val = ch943x_reg_read(s, CH943X_SPR_REG + (i * 0x10));
		if (val != 0x55) {
			return -1;
		}

		ch943x_reg_write(s, CH943X_SPR_REG + (i * 0x10), 0xAA);
		val = ch943x_reg_read(s, CH943X_SPR_REG + (i * 0x10));
		if (val != 0xAA) {
			return -1;
		}
	}
	return 0;
}

#ifdef USE_SPI_MODE
static int ch943x_probe(struct spi_device *spi,
			struct ch943x_devtype *devtype, int irq,
			unsigned long flags)
#else
static int ch943x_probe(struct i2c_client *i2c,
			struct ch943x_devtype *devtype, int irq,
			unsigned long flags)
#endif
{
	int ret;
	struct ch943x_port *s;
#ifdef USE_SPI_MODE
	struct device *dev = &spi->dev;
#else
	struct device *dev = &i2c->dev;
#endif

	/* Alloc port structure */
	s = devm_kzalloc(dev,
			 sizeof(*s) + sizeof(struct ch943x_one) *
					      devtype->nr_uart,
			 GFP_KERNEL);
	if (!s) {
		dev_err(dev, "Error allocating port structure\n");
		return -ENOMEM;
	}
	dev_set_drvdata(dev, s);

	s->devtype = devtype;
	s->irq = irq;
	s->reg485 = 0x00;
#ifdef USE_SPI_MODE
	s->spi_dev = spi;
	s->dev = &spi->dev;
#else
	s->i2c = i2c;
	s->dev = &i2c->dev;
#endif
	mutex_init(&s->mutex);
	mutex_init(&s->mutex_bus_access);

	ret = ch943x_hw_test(s);
	if (ret < 0) {
		dev_err(s->dev, "I2C or SPI transfer test Failed.\n");
		goto out;
	}

	ret = ch943x_get_chip_version(s);
	if (ret < 0) {
		dev_err(dev, "ch943x_get_chip_version error\n");
		goto out;
	}

	/* CH9434D IO enable */
	if (s->chiptype == CHIP_CH9434D) {
		ret = ch943x_io_enable(s);
		if (ret < 0) {
			dev_err(s->dev, "Error io enable.\n");
			return -1;
		}
	}

	/* Init Clock */
	if (s->chiptype == CHIP_CH9434D) {
#if defined(EXTERN_CLOCK)
		ch943x_reg_write(s, CH943X_CLK_REG, (0x03 << 6));
#elif defined(INTERNAL_CLOCK)
		ch943x_reg_write(s, CH943X_CLK_REG, 0);
#endif
	} else {
		ch943x_reg_write(s, CH943X_CLK_REG,
				 CH943X_CLK_EXT_BIT | CH943X_CLK_PLL_BIT |
					 13);
	}
	mdelay(200);

	/* Register UART driver */
	ret = ch943x_register_uart_driver(s);
	if (ret < 0) {
		dev_err(dev, "ch943x_register_uart_driver error\n");
		goto out;
	}

	/* Register UART port */
	ret = ch943x_register_uart_port(s);
	if (ret < 0) {
		dev_err(dev, "ch943x_register_uart error\n");
		goto out_uart_driver;
	}

	ret = ch943x_gpio_iodev_init(s);
	if (ret < 0) {
		dev_err(dev, "ch943x_gpio_iodev_init error\n");
		goto out_uart_driver;
	}

#ifdef CH9434D_CAN_ON
	if (s->chiptype == CHIP_CH9434D) {
		ret = ch943x_can_resgister(s);
		if (ret < 0) {
			dev_err(dev, "ch943x_can_resgister error\n");
			goto out_uart_gpio_driver;
		}
	}
#endif

	ret = devm_request_threaded_irq(dev, irq, ch943x_ist_top,
					ch943x_ist, IRQF_ONESHOT | flags,
					dev_name(dev), s);
	if (ret != 0) {
		dev_err(dev, "irq %d request failed, error %d\n", irq,
			ret);
		goto out_uart_gpio_driver;
	}
	dev_dbg(dev, "%s - devm_request_threaded_irq =%d result:%d\n",
		__func__, irq, ret);

	ch943x_debugfs_init(s);
	list_add_tail(&(s->ch943x_list), &g_private_head);

	return 0;

out_uart_gpio_driver:
	gpiochip_remove(&s->gpio);
out_uart_driver:
	uart_unregister_driver(&s->uart);
out:
	mutex_destroy(&s->mutex);
	mutex_destroy(&s->mutex_bus_access);
	return ret;
}

static int ch943x_remove(struct device *dev)
{
	struct ch943x_port *s = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	devm_free_irq(dev, s->irq, s);

	ch943x_uart_remove(s);
	ch943x_gpio_iodev_exit(s);

#ifdef CH9434D_CAN_ON
	if (s->chiptype == CHIP_CH9434D)
		ch943x_can_remove(s);
#endif
	ch943x_debugfs_exit(s);
	mutex_destroy(&s->mutex);
	mutex_destroy(&s->mutex_bus_access);
	return 0;
}

static const struct of_device_id __maybe_unused ch943x_dt_ids[] = {
	{
		.compatible = "wch,ch943x",
		.data = &ch943x_devtype,
	},
	{},
};
MODULE_DEVICE_TABLE(of, ch943x_dt_ids);

#ifdef USE_SPI_MODE
static int ch943x_spi_probe(struct spi_device *spi)
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int ch943x_i2c_probe(struct i2c_client *i2c)
#else
static int ch943x_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
#endif
#endif
{
	struct ch943x_devtype *devtype = &ch943x_devtype;
	unsigned long flags = IRQF_TRIGGER_LOW;
	int ret;
#ifdef USE_SPI_MODE
	struct device *dev = &spi->dev;
	int irq = spi->irq;
#else
	struct device *dev = &i2c->dev;
	int irq = i2c->irq;
#endif

	dev_dbg(dev, "gpio_to_irq:%d, spi->irq:%d\n",
		gpio_to_irq(GPIO_NUMBER), irq);

#ifdef USE_SPI_MODE
	spi->mode |= SPI_MODE_3;
	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "spi_setup failed, err=%d\n", ret);
		return ret;
	}
#endif

#ifdef USE_IRQ_FROM_DTS
	/* if your platform supports acquire irq number from dts */
#ifdef USE_SPI_MODE
	ret = ch943x_probe(spi, devtype, irq, flags);
#else
	ret = ch943x_probe(i2c, devtype, irq, flags);
#endif
	if (ret) {
		dev_err(&spi->dev, "ch943x_probe error\n");
		goto out;
	}
#else
	ret = devm_gpio_request(dev, GPIO_NUMBER, "gpioint");
	if (ret) {
		dev_err(dev, "gpio request\n");
		goto out;
	}
	ret = gpio_direction_input(GPIO_NUMBER);
	if (ret) {
		dev_err(dev, "gpio_direction_input\n");
		goto out;
	}
	irq_set_irq_type(gpio_to_irq(GPIO_NUMBER), flags);
#ifdef USE_SPI_MODE
	ret = ch943x_probe(spi, devtype, gpio_to_irq(GPIO_NUMBER), flags);
#else
	ret = ch943x_probe(i2c, devtype, gpio_to_irq(GPIO_NUMBER), flags);
#endif
	if (ret) {
		dev_err(dev, "ch943x_probe error\n");
		goto out;
	}
#endif

out:
	return ret;
}

#ifdef USE_SPI_MODE
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
static void ch943x_spi_remove(struct spi_device *spi)
#else
static int ch943x_spi_remove(struct spi_device *spi)
#endif
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	ch943x_remove(&spi->dev);
#else
	return ch943x_remove(&spi->dev);
#endif
}
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void ch943x_i2c_remove(struct i2c_client *client)
#else
static int ch943x_i2c_remove(struct i2c_client *client)
#endif
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	ch943x_remove(&client->dev);
#else
	return ch943x_remove(&client->dev);
#endif
}
#endif

int ch943x_suspend(struct device *dev)
{
	struct ch943x_port *s = dev_get_drvdata(dev);

	dev_info(dev, "ch943x_suspend");
	ch943x_reg_write(s, 0x4a, 0x01);

	return 0;
}

int ch943x_resume(struct device *dev)
{
	struct ch943x_port *s = dev_get_drvdata(dev);

	dev_info(dev, "ch943x_resume");
	ch943x_reg_write(s, 0x4a, 0x00);

	return 0;
}

static struct dev_pm_ops ch943x_pm_ops = {
	.suspend = ch943x_suspend,
	.resume = ch943x_resume,
};

#ifdef USE_SPI_MODE
static struct spi_driver ch943x_spi_driver = {
    .driver =
        {
                 .name = CH943X_NAME_SPI,
                 .bus = &spi_bus_type,
                 .owner = THIS_MODULE,
                 .of_match_table = of_match_ptr(ch943x_dt_ids),
                 .pm = &ch943x_pm_ops,
                 },
    .probe = ch943x_spi_probe,
    .remove = ch943x_spi_remove,
};
MODULE_ALIAS("spi:ch943x");
#else
static struct i2c_driver ch943x_i2c_driver = {
    .driver =
        {
                 .name = CH943X_NAME_I2C,
                 .owner = THIS_MODULE,
                 .of_match_table = of_match_ptr(ch943x_dt_ids),
                 .pm = &ch943x_pm_ops,
                 },
    .probe = ch943x_i2c_probe,
    .remove = ch943x_i2c_remove,
};
MODULE_ALIAS("i2c:ch943x");
#endif

static int __init ch943x_init(void)
{
	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_DESC "\n");
	printk(KERN_INFO KBUILD_MODNAME ": " VERSION_DESC "\n");
	INIT_LIST_HEAD(&g_private_head);
#ifdef USE_SPI_MODE
	return spi_register_driver(&ch943x_spi_driver);
#else
	return i2c_add_driver(&ch943x_i2c_driver);
#endif
}

static void __exit ch943x_exit(void)
{
	printk(KERN_INFO KBUILD_MODNAME ": ch943x driver exit.\n");
#ifdef USE_SPI_MODE
	spi_unregister_driver(&ch943x_spi_driver);
#else
	i2c_del_driver(&ch943x_i2c_driver);
#endif
}

module_init(ch943x_init);
module_exit(ch943x_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
