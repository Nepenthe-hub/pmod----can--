#define DEBUG
#define VERBOSE_DEBUG

#undef DEBUG
#undef VERBOSE_DEBUG

#include "ch9434.h"

static inline void spi_delay_set(struct spi_transfer *xfer,
				 unsigned int value)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19)
	xfer->delay.value = value;
#else
	xfer->delay_usecs = value;
#endif
}

int ch943x_canreg_write(struct ch943x_port *s, u8 reg, u32 val)
{
	u8 cmd = CH943X_REG_OP_WRITE | CH9434_CAN_REG;
	u8 data[5] = { 0 };
#ifdef USE_SPI_MODE
	int retval;
	struct spi_message m;
	struct spi_transfer xfer[3] = {};
#else
	u8 buf[8] = { 0 };
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[2] = {};
#endif

	data[0] = reg;
	data[1] = val & 0xff;
	data[2] = (val >> 8) & 0xff;
	data[3] = (val >> 16) & 0xff;
	data[4] = (val >> 24) & 0xff;

#ifdef USE_SPI_MODE
	xfer[0].tx_buf = &cmd;
	xfer[0].len = 1;
	xfer[0].cs_change = 0;
	spi_delay_set(&xfer[0], CH943X_CAN_CMD_DELAY);

	xfer[1].tx_buf = data;
	xfer[1].len = 5;
	xfer[1].cs_change = 0;
	spi_delay_set(&xfer[1], CH943X_CAN_CMD_DELAY);

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	retval = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (retval < 0) {
		dev_err(s->dev, "%s spi transfer failed\n", __func__);
		return retval;
	}
	dev_dbg(s->dev,
		"%s - cmd:%02x reg:%02x data:%02x %02x %02x %02x [0x%08x]\n",
		__func__, cmd, data[0], data[1], data[2], data[3], data[4],
		val);
	return 0;
#else
	mutex_lock(&s->mutex_bus_access);
	buf[0] = cmd;
	memcpy(buf + 1, data, 5);

	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 6;
	xfer[0].buf = buf;

	if (i2c_transfer(i2c->adapter, xfer, 1) != 1) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(s->dev, "%s i2c transfer failed\n", __func__);
		return -EIO;
	}
	mutex_unlock(&s->mutex_bus_access);

	dev_dbg(&i2c->dev,
		"%s - cmd:%02x reg:%02x, data:%02x %02x %02x %02x %02x\n",
		__func__, buf[0], reg, buf[1], buf[2], buf[3], buf[4],
		buf[5]);
	return 0;
#endif
}

int ch943x_canreg_contmode_write(struct ch943x_port *s, u8 reg, u8 *txbuf,
				 u8 txlen)
{
	u8 cmd = CH943X_REG_OP_WRITE | CH9434_CAN_REG;
	u8 data[64] = { 0 };
#ifdef USE_SPI_MODE
	ssize_t retval;
	struct spi_message m;
	struct spi_transfer xfer[3] = {};
#else
	u8 buf[64] = { 0 };
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[2] = {};
#endif

	memcpy(data, txbuf, txlen);
#ifdef USE_SPI_MODE
	xfer[0].tx_buf = &cmd;
	xfer[0].len = 1;
	xfer[0].cs_change = 0;
	spi_delay_set(&xfer[0], CH943X_CAN_CMD_DELAY);

	xfer[1].tx_buf = &reg;
	xfer[1].len = 1;
	xfer[1].cs_change = 0;
	spi_delay_set(&xfer[1], CH943X_CAN_CMD_DELAY);

	xfer[2].tx_buf = data;
	xfer[2].len = txlen;
	xfer[2].cs_change = 0;
	spi_delay_set(&xfer[2], CH943X_CAN_CMD_DELAY);

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	spi_message_add_tail(&xfer[2], &m);
	retval = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (retval < 0) {
		dev_err(s->dev, "%s spi transfer failed\n", __func__);
		return retval;
	}
	return 0;
#else
	mutex_lock(&s->mutex_bus_access);
	buf[0] = cmd;
	buf[1] = reg;
	memcpy(buf + 2, txbuf, txlen);

	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = txlen + 2;
	xfer[0].buf = buf;

	if (i2c_transfer(i2c->adapter, xfer, 1) != 1) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
		return -EIO;
	}
	return 0;
#endif
}

u32 ch943x_canreg_read(struct ch943x_port *s, u8 reg)
{
	u8 cmd = CH943X_REG_OP_READ | CH9434_CAN_REG;
	u32 value = 0;

#ifdef USE_SPI_MODE
	ssize_t retval;
	u8 data[4] = { 0 };
	struct spi_message m;
	struct spi_transfer xfer[3] = {};
#else
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[3] = {};
	u8 data1[2] = { cmd, reg };
	u8 data2[4] = { 0 };
#endif

#ifdef USE_SPI_MODE
	xfer[0].tx_buf = &cmd;
	xfer[0].len = 1;
	xfer[0].cs_change = 0;
	spi_delay_set(&xfer[0], CH943X_CAN_CMD_DELAY);

	xfer[1].tx_buf = &reg;
	xfer[1].len = 1;
	xfer[1].cs_change = 0;
	spi_delay_set(&xfer[1], CH943X_CAN_CMD_DELAY);

	xfer[2].rx_buf = data;
	xfer[2].len = 4;
	xfer[2].cs_change = 0;
	spi_delay_set(&xfer[2], CH943X_CAN_CMD_DELAY);

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	spi_message_add_tail(&xfer[2], &m);
	retval = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (retval < 0) {
		dev_err(s->dev, "%s spi transfer failed\n", __func__);
		return retval;
	}
	value |= data[0];
	value |= data[1] << 8;
	value |= data[2] << 16;
	value |= data[3] << 24;
	dev_dbg(s->dev,
		"%s - cmd:%02x reg:%02x data:%02x %02x %02x %02x [0x%08x]\n",
		__func__, cmd, reg, data[0], data[1], data[2], data[3],
		value);

	return value;
#else
	mutex_lock(&s->mutex_bus_access);
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = data1;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 4;
	xfer[1].buf = data2;

	if (i2c_transfer(i2c->adapter, xfer, 2) != 2) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
		return -EIO;
	}
	mutex_unlock(&s->mutex_bus_access);

	value |= data2[0];
	value |= data2[1] << 8;
	value |= data2[2] << 16;
	value |= data2[3] << 24;
	return value;
#endif
}

int ch943x_canreg_contmode_read(struct ch943x_port *s, u8 reg, u8 *rxbuf)
{
	u8 cmd = CH943X_REG_OP_READ | CH9434_CAN_REG;
#ifdef USE_SPI_MODE
	ssize_t status;
	u8 data[64] = { 0 };
	struct spi_message m;
	struct spi_transfer xfer[3] = {};
#else
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[3] = {};
	u8 data1[2] = { cmd, reg };
	u8 data2[128] = { 0 };
#endif

#ifdef USE_SPI_MODE
	xfer[0].tx_buf = &cmd;
	xfer[0].len = 1;
	xfer[0].cs_change = 0;
	spi_delay_set(&xfer[0], CH943X_CAN_CMD_DELAY);

	xfer[1].tx_buf = &reg;
	xfer[1].len = 1;
	xfer[1].cs_change = 0;
	spi_delay_set(&xfer[1], CH943X_CAN_CMD_DELAY);

	xfer[2].rx_buf = data;
	xfer[2].len = 64;
	xfer[2].cs_change = 0;
	spi_delay_set(&xfer[2], CH943X_CAN_CMD_DELAY);

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	spi_message_add_tail(&xfer[2], &m);
	status = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (status < 0) {
		dev_err(s->dev, "%s spi transfer failed\n", __func__);
		return status;
	}
	memcpy(rxbuf, data, 64);

	return 64;
#else
	mutex_lock(&s->mutex_bus_access);
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = data1;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 64;
	xfer[1].buf = data2;

	if (i2c_transfer(i2c->adapter, xfer, 2) != 2) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
		return -EIO;
	}
	mutex_unlock(&s->mutex_bus_access);
	return 64;
#endif
}

int ch943x_ctrl_write(struct ch943x_port *s, u8 cmd, u32 datalen,
		      void *data)
{
	u8 *buffer;
	int retval;

#ifdef USE_SPI_MODE
	struct spi_message m;
	struct spi_transfer xfer[3] = {};
#else
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[2] = {};
#endif

	buffer = kmalloc(2048, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	buffer[0] = cmd | CH943X_REG_OP_WRITE;

	retval = copy_from_user(buffer + 1, (char __user *)data, datalen);
	if (retval) {
		dev_err(s->dev, "%s copy_from_user error. retval:%d\n",
			__func__, retval);
		goto out;
	}

#ifdef USE_SPI_MODE
	xfer[0].tx_buf = &buffer[0];
	xfer[0].len = 1;
	xfer[0].cs_change = 0;
	spi_delay_set(&xfer[0], CH943X_CAN_CMD_DELAY);

	xfer[1].tx_buf = buffer + 1;
	xfer[1].len = datalen;
	xfer[1].cs_change = 0;
	spi_delay_set(&xfer[1], CH943X_CAN_CMD_DELAY);

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	retval = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (retval < 0) {
		dev_err(s->dev, "%s spi transfer failed\n", __func__);
		goto out;
	}

	if (datalen == 1) {
		dev_dbg(s->dev, "%s - reg:%02x, data:%02x\n", __func__,
			buffer[0], buffer[1]);
	}

#else
	mutex_lock(&s->mutex_bus_access);
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = datalen + 1;
	xfer[0].buf = buffer;

	if (i2c_transfer(i2c->adapter, xfer, 1) != 1) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
		retval = -EIO;
		goto out;
	}
	mutex_unlock(&s->mutex_bus_access);
#endif

out:
	kfree(buffer);
	return retval;
}

int ch943x_ctrl_read(struct ch943x_port *s, u8 cmd, u32 datalen,
		     void *data)
{
	u8 *buffer;
	u8 tx_cmd = cmd | CH943X_REG_OP_READ;
	int retval;
#ifdef USE_SPI_MODE
	struct spi_message m;
	struct spi_transfer xfer[3] = {};
#else
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[2] = {};
#endif

	buffer = kmalloc(2048, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

#ifdef USE_SPI_MODE
	xfer[0].tx_buf = &tx_cmd;
	xfer[0].len = 1;
	xfer[0].cs_change = 0;
	spi_delay_set(&xfer[0], CH943X_CAN_CMD_DELAY);

	xfer[1].rx_buf = buffer;
	xfer[1].len = datalen;
	xfer[1].cs_change = 0;
	spi_delay_set(&xfer[1], CH943X_CAN_CMD_DELAY);

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	retval = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (retval < 0) {
		dev_err(s->dev, "%s spi transfer failed\n", __func__);
		goto out;
	}

	if (datalen == 1) {
		dev_dbg(s->dev, "%s - reg:%02x, data:%02x\n", __func__,
			tx_cmd, buffer[0]);
	}

#else
	mutex_lock(&s->mutex_bus_access);
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &tx_cmd;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = datalen;
	xfer[1].buf = buffer;

	if (i2c_transfer(i2c->adapter, xfer, 2) != 2) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
		retval = -EIO;
		goto out;
	}
	mutex_unlock(&s->mutex_bus_access);
#endif

	retval = copy_to_user((char __user *)data, buffer, datalen);
	if (retval) {
		dev_err(s->dev, "%s copy_to_user error. retval:%d\n",
			__func__, retval);
		retval = -EFAULT;
	}
out:
	kfree(buffer);
	return retval;
}

int ch943x_iofunc_cfg(struct ch943x_port *s, u8 io_cmd, u8 io_addr,
		      u8 enable)
{
	u8 cmd_buf[4] = { 0 };
	u8 rx_buf[4] = { 0 };
	u8 reg_cmd = CH9434_IO_SEL_FUN_CFG | 0x80;
	u8 reg_cmd_r = CH9434_IO_SEL_FUN_CFG;
	int times;
	int ret;
#ifdef USE_SPI_MODE
	ssize_t status;
	struct spi_message m;
	struct spi_transfer xfer[2] = {};
#else
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[2] = {};
	u8 buf[5] = { 0 };
#endif

	cmd_buf[0] = io_cmd;
	cmd_buf[1] = io_addr;
	cmd_buf[2] = enable;
	cmd_buf[3] = CH9434D_IO_CMD_ACT;

#ifdef USE_SPI_MODE
	xfer[0].tx_buf = &reg_cmd;
	xfer[0].len = 1;
	xfer[0].cs_change = 0;
	spi_delay_set(&xfer[0], CH943X_CMD_DELAY);

	xfer[1].tx_buf = cmd_buf;
	xfer[1].len = 4;
	xfer[1].cs_change = 0;
	spi_delay_set(&xfer[1], CH943X_CMD_DELAY);

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	status = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (status < 0) {
		dev_err(s->dev, "%s spi transfer failed\n", __func__);
		return status;
	}

	dev_dbg(s->dev, "%s - reg:%02x, data:%02x %02x %02x %02x\n",
		__func__, reg_cmd, cmd_buf[0], cmd_buf[1], cmd_buf[2],
		cmd_buf[3]);

	udelay(1000);
	times = 0;
	while (1) {
		xfer[0].tx_buf = &reg_cmd_r;
		xfer[0].len = 1;
		xfer[0].cs_change = 0;
		spi_delay_set(&xfer[0], CH943X_CMD_DELAY);

		xfer[1].rx_buf = rx_buf;
		xfer[1].len = 4;
		xfer[1].cs_change = 0;
		spi_delay_set(&xfer[1], CH943X_CMD_DELAY);

		mutex_lock(&s->mutex_bus_access);
		spi_message_init(&m);
		spi_message_add_tail(&xfer[0], &m);
		spi_message_add_tail(&xfer[1], &m);
		status = spi_sync(s->spi_dev, &m);
		mutex_unlock(&s->mutex_bus_access);
		if (status < 0) {
			dev_err(s->dev, "%s spi transfer failed\n",
				__func__);
			return status;
		}

		dev_dbg(s->dev,
			"%s - reg:%02x, rx_buf:%02x %02x %02x %02x\n",
			__func__, reg_cmd_r, rx_buf[0], rx_buf[1],
			rx_buf[2], rx_buf[3]);

		if ((cmd_buf[0] == 0x01) || (cmd_buf[0] == 0x03)) {
			if ((rx_buf[3] == CH9434D_IO_CMD_COMP) &&
			    (rx_buf[0] == cmd_buf[0]) &&
			    (rx_buf[1] == cmd_buf[1])) {
				ret = 0;
				break;
			} else {
				if (times++ > 100) {
					dev_err(s->dev, "%s timeout!\n",
						__func__);
					return -EIO;
				}
				mdelay(1);
				continue;
			}
		} else if ((cmd_buf[0] == 0x81) || (cmd_buf[0] == 0x83)) {
			if ((rx_buf[3] == CH9434D_IO_CMD_COMP) &&
			    (rx_buf[0] == cmd_buf[0])) {
				ret = rx_buf[2];
				break;
			} else {
				if (times++ > 100) {
					dev_err(s->dev, "%s timeout!\n",
						__func__);
					return -EIO;
				}
				mdelay(1);
				continue;
			}
		}
	}
#else
	mutex_lock(&s->mutex_bus_access);
	buf[0] = reg_cmd;
	memcpy(buf + 1, cmd_buf, 4);

	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 5;
	xfer[0].buf = buf;

	if (i2c_transfer(i2c->adapter, xfer, 1) != 1) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
		return -EIO;
	}
	mutex_unlock(&s->mutex_bus_access);
	dev_dbg(&i2c->dev, "%s - reg:%02x, data:%02x %02x %02x %02x\n",
		__func__, reg_cmd, cmd_buf[0], cmd_buf[1], cmd_buf[2],
		cmd_buf[3]);

	udelay(3);
	times = 0;
	while (1) {
		mutex_lock(&s->mutex_bus_access);
		xfer[0].addr = i2c->addr;
		xfer[0].flags = 0;
		xfer[0].len = 1;
		xfer[0].buf = &reg_cmd_r;

		xfer[1].addr = i2c->addr;
		xfer[1].flags = I2C_M_RD;
		xfer[1].len = 4;
		xfer[1].buf = rx_buf;

		if (i2c_transfer(i2c->adapter, xfer, 2) != 2) {
			mutex_unlock(&s->mutex_bus_access);
			dev_err(&i2c->dev, "%s i2c transfer failed\n",
				__func__);
			return -EIO;
		}
		mutex_unlock(&s->mutex_bus_access);

		dev_dbg(&i2c->dev,
			"%s - reg:%02x, data:%02x %02x %02x %02x\n",
			__func__, reg_cmd, rx_buf[0], rx_buf[1], rx_buf[2],
			rx_buf[3]);

		if ((cmd_buf[0] == 0x01) || (cmd_buf[0] == 0x03)) {
			if ((rx_buf[3] == CH9434D_IO_CMD_COMP) &&
			    (rx_buf[0] == cmd_buf[0]) &&
			    (rx_buf[1] == cmd_buf[1])) {
				ret = 0;
				break;
			} else {
				if (times++ > 100) {
					dev_err(s->dev, "%s timeout!\n",
						__func__);
					return -EIO;
				}
				mdelay(1);
				continue;
			}
		} else if ((cmd_buf[0] == 0x81) || (cmd_buf[0] == 0x83)) {
			if ((rx_buf[3] == CH9434D_IO_CMD_COMP) &&
			    (rx_buf[0] == cmd_buf[0])) {
				ret = rx_buf[2];
				break;
			} else {
				if (times++ > 100) {
					dev_err(s->dev, "%s timeout!\n",
						__func__);
					return -EIO;
				}
				mdelay(1);
				continue;
			}
		}
	}
#endif

	return ret;
}

u8 ch943x_port_read(struct uart_port *port, u8 reg)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	u8 cmd = (0x00 | reg) + (port->line * 0x10);
	u8 val;
#ifdef USE_SPI_MODE
	ssize_t status;
	struct spi_message m;
	struct spi_transfer xfer[2] = {};
#else
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[2] = {};
#endif

#ifdef USE_SPI_MODE
	xfer[0].tx_buf = &cmd;
	xfer[0].len = 1;
	xfer[0].cs_change = 0;
	spi_delay_set(&xfer[0], CH943X_CMD_DELAY);

	xfer[1].rx_buf = &val;
	xfer[1].len = 1;
	xfer[1].cs_change = 0;
	spi_delay_set(&xfer[1], CH943X_CMD_DELAY);

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	status = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (status < 0) {
		dev_err(s->dev, "%s spi transfer failed\n", __func__);
	}
	dev_dbg(s->dev, "%s - reg:%02X, val:%02X\n", __func__, cmd, val);
	return val;
#else
	mutex_lock(&s->mutex_bus_access);
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &cmd;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 1;
	xfer[1].buf = &val;

	if (i2c_transfer(i2c->adapter, xfer, 2) != 2) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
		return -EIO;
	}
	mutex_unlock(&s->mutex_bus_access);
	dev_dbg(&i2c->dev, "%s - reg:%02x, val:%02x\n", __func__, cmd,
		val);
	return val;
#endif
}

u8 ch943x_reg_read(struct ch943x_port *s, u8 reg)
{
	u8 cmd = reg;
	u8 val;
#ifdef USE_SPI_MODE
	ssize_t status;
	struct spi_message m;
	struct spi_transfer xfer[2] = {};
#else
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[2] = {};
#endif

#ifdef USE_SPI_MODE
	xfer[0].tx_buf = &cmd;
	xfer[0].len = 1;
	xfer[0].cs_change = 0;
	spi_delay_set(&xfer[0], CH943X_CMD_DELAY);

	xfer[1].rx_buf = &val;
	xfer[1].len = 1;
	xfer[1].cs_change = 0;
	spi_delay_set(&xfer[1], CH943X_CMD_DELAY);

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	status = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (status < 0) {
		dev_err(s->dev, "%s spi transfer failed\n", __func__);
	}
	dev_dbg(s->dev, "%s - reg:%02X, val:%02X\n", __func__, cmd, val);

	return val;
#else
	mutex_lock(&s->mutex_bus_access);
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &cmd;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 1;
	xfer[1].buf = &val;

	if (i2c_transfer(i2c->adapter, xfer, 2) != 2) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
		return -EIO;
	}
	mutex_unlock(&s->mutex_bus_access);
	dev_dbg(&i2c->dev, "%s - reg:%02x, val:%02x\n", __func__, cmd,
		val);
	return val;
#endif
}

int ch943x_port_write(struct uart_port *port, u8 reg, u8 val)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	u8 cmd = (0x80 | reg) + (port->line * 0x10);
#ifdef USE_SPI_MODE
	ssize_t status;
	struct spi_message m;
	struct spi_transfer xfer[2] = {};
#else
	u8 buf[2] = { cmd, val };
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[2] = {};
#endif

#ifdef USE_SPI_MODE
	xfer[0].tx_buf = &cmd;
	xfer[0].len = 1;
	xfer[0].cs_change = 0;
	spi_delay_set(&xfer[0], CH943X_CMD_DELAY);

	xfer[1].tx_buf = &val;
	xfer[1].len = 1;
	xfer[1].cs_change = 0;
	spi_delay_set(&xfer[1], CH943X_CMD_DELAY);

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	status = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (status < 0) {
		dev_err(s->dev, "%s spi transfer failed\n", __func__);
		return status;
	}
	dev_dbg(s->dev, "%s - reg:%02X, val:%02X\n", __func__, cmd, val);
	return 0;
#else
	mutex_lock(&s->mutex_bus_access);
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	if (i2c_transfer(i2c->adapter, xfer, 1) != 1) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
		return -EIO;
	}
	mutex_unlock(&s->mutex_bus_access);
	dev_dbg(&i2c->dev, "%s - reg:%02x, val:%02x\n", __func__, cmd,
		val);
	return 0;
#endif
}

int ch943x_reg_write(struct ch943x_port *s, u8 reg, u8 val)
{
	u8 cmd = 0x80 | reg;
#ifdef USE_SPI_MODE
	ssize_t status;
	struct spi_message m;
	struct spi_transfer xfer[2] = {};
#else
	u8 buf[2] = { cmd, val };
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[2] = {};
#endif

#ifdef USE_SPI_MODE
	xfer[0].tx_buf = &cmd;
	xfer[0].len = 1;
	xfer[0].cs_change = 0;
	spi_delay_set(&xfer[0], CH943X_CMD_DELAY);

	xfer[1].tx_buf = &val;
	xfer[1].len = 1;
	xfer[1].cs_change = 0;
	spi_delay_set(&xfer[1], CH943X_CMD_DELAY);

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	status = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (status < 0) {
		dev_err(s->dev, "%s spi transfer failed\n", __func__);
		return status;
	}
	dev_dbg(s->dev, "%s - reg:%02X, val:%02X\n", __func__, cmd, val);
	return 0;
#else
	mutex_lock(&s->mutex_bus_access);
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	if (i2c_transfer(i2c->adapter, xfer, 1) != 1) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
		return -EIO;
	}
	mutex_unlock(&s->mutex_bus_access);
	dev_dbg(&i2c->dev, "%s - reg:%02X, val:%02X\n", __func__, cmd,
		val);
	return 0;
#endif
}

int ch943x_port_read_version(struct ch943x_port *s, u8 reg, u8 *buf,
			     u8 count)
{
	u8 cmd = reg;
#ifdef USE_SPI_MODE
	ssize_t status;
	struct spi_message m;
	struct spi_transfer xfer[2] = {};
#else
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[2] = {};
#endif

#ifdef USE_SPI_MODE
	xfer[0].tx_buf = &cmd;
	xfer[0].len = 1;
	xfer[0].cs_change = 0;
	spi_delay_set(&xfer[0], CH943X_CMD_DELAY);

	xfer[1].rx_buf = buf;
	xfer[1].len = 4;
	xfer[1].cs_change = 0;
	spi_delay_set(&xfer[1], CH943X_CMD_DELAY);

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	status = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (status < 0) {
		dev_err(s->dev, "%s spi transfer failed\n", __func__);
		goto out;
	}
	dev_dbg(s->dev, "%s rbuf:%02x %02x %02x %02x\n", __func__, buf[0],
		buf[1], buf[2], buf[3]);
out:
	return status < 0 ? status : 0;
#else
	mutex_lock(&s->mutex_bus_access);
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &cmd;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = count;
	xfer[1].buf = buf;

	if (i2c_transfer(i2c->adapter, xfer, 2) != 2) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
		return -EIO;
	}
	mutex_unlock(&s->mutex_bus_access);

	dev_dbg(&i2c->dev, "%s rbuf:%02x %02x %02x %02x\n", __func__,
		buf[0], buf[1], buf[2], buf[3]);
	return 0;
#endif
}

int ch943x_port_update(struct uart_port *port, u8 reg, u8 mask, u8 val)
{
	unsigned int tmp;

	tmp = ch943x_port_read(port, reg);
	tmp &= ~mask;
	tmp |= val & mask;
	ch943x_port_write(port, reg, tmp);
	return 0;
}

int ch943x_raw_write(struct uart_port *port, const void *reg,
		     unsigned char *buf, int len)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	struct ch943x_one *one = to_ch943x_one(port, port);
#ifdef USE_SPI_MODE
	int i;
	int writesum = 0;
	ssize_t status;
	struct spi_message m;
	struct spi_transfer xfer[2] = {};
	struct spi_transfer t[2] = {};
	struct spi_transfer x = {};
#else
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[2] = {};
#endif

#ifdef USE_SPI_MODE
	if (s->chiptype == CHIP_CH9434D) {
		xfer[0].tx_buf = reg;
		xfer[0].len = 1;
		xfer[0].cs_change = 0;
		spi_delay_set(&xfer[0], CH943X_CMD_DELAY);

		xfer[1].tx_buf = buf;
		xfer[1].len = len;
		spi_delay_set(&xfer[1], CH943X_CMD_DELAY);

		mutex_lock(&s->mutex_bus_access);
		spi_message_init(&m);
		spi_message_add_tail(&xfer[0], &m);
		spi_message_add_tail(&xfer[1], &m);
		status = spi_sync(s->spi_dev, &m);
		mutex_unlock(&s->mutex_bus_access);
		if (status < 0) {
			dev_err(s->dev, "%s spi transfer failed\n",
				__func__);
			return status;
		}
	} else {
		if (s->spi_contmode) {
			x.tx_buf = one->txbuf;
			x.len = len + 1;

			one->txbuf[0] = *(u8 *)reg;
			memcpy(one->txbuf + 1, buf, len);

			mutex_lock(&s->mutex_bus_access);
			spi_message_init(&m);
			spi_message_add_tail(&x, &m);
			status = spi_sync(s->spi_dev, &m);
			mutex_unlock(&s->mutex_bus_access);
			if (status < 0) {
				dev_err(s->dev, "%s spi transfer failed\n",
					__func__);
				return status;
			}
			dev_dbg(s->dev, "%s - reg:%02X, len:%d\n",
				__func__, *(u8 *)reg, len);
			for (i = 0; i < len; i++)
				dev_dbg(s->dev, "\tbuf[%d]:%02X\n", i,
					buf[i]);
		} else {
			while (len--) {
				t[0].tx_buf = reg;
				t[0].len = 1;
				spi_delay_set(&t[0], CH943X_CMD_DELAY);

				t[1].tx_buf = buf + writesum;
				t[1].len = 1;
				spi_delay_set(&t[1], CH943X_CMD_DELAY);

				mutex_lock(&s->mutex_bus_access);
				spi_message_init(&m);
				spi_message_add_tail(&t[0], &m);
				spi_message_add_tail(&t[1], &m);
				status = spi_sync(s->spi_dev, &m);
				mutex_unlock(&s->mutex_bus_access);
				if (status < 0) {
					dev_err(s->dev,
						"%s spi transfer failed\n",
						__func__);
					return status;
				}
				dev_dbg(s->dev,
					"%s - reg:%02X, buf[%d]:%02X\n",
					__func__, *(u8 *)reg, writesum,
					buf[writesum]);
				writesum++;
			}
		}
	}
	return 0;
#else
	mutex_lock(&s->mutex_bus_access);
	one->txbuf[0] = *(u8 *)reg;
	memcpy(one->txbuf + 1, buf, len);

	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = len + 1;
	xfer[0].buf = one->txbuf;

	if (i2c_transfer(i2c->adapter, xfer, 1) != 1) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
		return -EIO;
	}
	mutex_unlock(&s->mutex_bus_access);
	return 0;
#endif
}

int ch943x_raw_read(struct uart_port *port, u8 reg, unsigned char *buf,
		    int len)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	struct ch943x_one *one = to_ch943x_one(port, port);
	u8 cmd = (0x00 | reg) + (port->line * 0x10);
#ifdef USE_SPI_MODE
	int i;
	ssize_t status;
	struct spi_message m;
	struct spi_transfer x = {};
	struct spi_transfer xfer[2] = {};
#else
	struct i2c_client *i2c = s->i2c;
	struct i2c_msg xfer[2] = {};
#endif

#ifdef USE_SPI_MODE
	if (s->chiptype == CHIP_CH9434D) {
		xfer[0].tx_buf = &cmd;
		xfer[0].rx_buf = NULL;
		xfer[0].len = 1;
		xfer[0].cs_change = 0;
		spi_delay_set(&xfer[0], CH943X_CMD_DELAY);

		xfer[1].rx_buf = one->rxbuf;
		xfer[1].len = len;
		spi_delay_set(&xfer[1], CH943X_CMD_DELAY);
	} else {
		x.rx_buf = one->rxbuf;
		x.tx_buf = one->rxbuf;
		x.len = len + 2;
	}

	if (s->chiptype == CHIP_CH9434D) {
		mutex_lock(&s->mutex_bus_access);
		spi_message_init(&m);
		spi_message_add_tail(&xfer[0], &m);
		spi_message_add_tail(&xfer[1], &m);
		status = spi_sync(s->spi_dev, &m);
		mutex_unlock(&s->mutex_bus_access);
		if (status < 0) {
			dev_err(s->dev, "%s spi transfer failed\n",
				__func__);
			return status;
		} else {
			memcpy(buf, one->rxbuf, len);
			dev_dbg(s->dev, "%s - cmd:%02x, len:%d\n",
				__func__, cmd, len);
			for (i = 0; i < len; i++)
				dev_dbg(s->dev, "\tbuf[%d]:%2X\n", i,
					buf[i]);
			return status;
		}
	} else {
		one->rxbuf[0] = cmd;
		mutex_lock(&s->mutex_bus_access);
		spi_message_init(&m);
		spi_message_add_tail(&x, &m);
		status = spi_sync(s->spi_dev, &m);
		mutex_unlock(&s->mutex_bus_access);
		if (status < 0) {
			dev_err(s->dev, "%s spi transfer failed\n",
				__func__);
			return status;
		} else {
			memcpy(buf, one->rxbuf + 2, len);
			return status;
		}
	}
#else
	mutex_lock(&s->mutex_bus_access);
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &cmd;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = one->rxbuf;

	if (i2c_transfer(i2c->adapter, xfer, 2) != 2) {
		mutex_unlock(&s->mutex_bus_access);
		dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
		return -EIO;
	}
	memcpy(buf, one->rxbuf, len);
	mutex_unlock(&s->mutex_bus_access);
	return 0;
#endif
}

int ch943x_get_chip_version(struct ch943x_port *s)
{
#ifdef USE_SPI_MODE
	u8 spi_contmode_reg;
#endif
	int ret;

	/* get chip type and version */
	ret = ch943x_port_read_version(s, CH943X_CHIP_VER_REG, s->ver,
				       VER_LEN);
	if (ret) {
		dev_err(s->dev, "Get CH943X chip version failed\n");
		return -1;
	}

#ifdef USE_SPI_MODE
	if ((s->ver[3] == 0x5A) &&
	    (s->ver[2] == (s->ver[0] + s->ver[1]))) {
		s->chiptype = CHIP_CH9434A;
		spi_contmode_reg =
			ch943x_reg_read(s, CH943X_SPI_CONT_MODE_REG);
		ch943x_reg_write(s, CH943X_SPI_CONT_MODE_REG,
				 CH943X_SPI_CONTE_BIT | spi_contmode_reg);

		if (s->spi_dev->max_speed_hz > 2000000) {
			dev_err(s->dev,
				"SPI continuous transmission mode can only be enabled when the spi clock does not exceed 2M.\n");
			s->spi_dev->max_speed_hz = 2000000;
		}
		s->spi_contmode = true;
		dev_info(s->dev,
			 "%s - CHIP:CH9434A - firmware version: V%d.%d\n",
			 __func__, s->ver[1], s->ver[0]);
	} else if ((s->ver[3] == 0x6B) &&
		   (s->ver[2] == (s->ver[0] + s->ver[1]))) {
		s->chiptype = CHIP_CH9434D;
		s->spi_contmode = true;
		dev_info(s->dev,
			 "%s - CHIP:CH9434D - firmware version: V%d.%d\n",
			 __func__, s->ver[1], s->ver[0]);
	} else {
		s->chiptype = CHIP_CH9434M;
		s->ver[1] = 1;
		s->ver[0] = 0;
		s->spi_contmode = false;
		dev_info(
			s->dev,
			"%s - CHIP:CH9434M/A old version - firmware version: V%d.%d\n",
			__func__, s->ver[1], s->ver[0]);
	}
#else
	if ((s->ver[3] == 0x6B) &&
	    (s->ver[2] == (s->ver[0] + s->ver[1]))) {
		s->chiptype = CHIP_CH9434D;
		s->spi_contmode = true;
		dev_info(s->dev,
			 "%s - CHIP:CH9434D - firmware version: V%d.%d\n",
			 __func__, s->ver[1], s->ver[0]);
	}
#endif
	return 0;
}

static ssize_t ch943x_proc_read(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct ch943x_port *s;
	char *buf;
	u32 len = 0;
	ssize_t ret;

	s = file->private_data;

	buf = kzalloc(REGS_BUFSIZE, GFP_KERNEL);
	if (!buf)
		return 0;

	len += snprintf(buf + len, REGS_BUFSIZE - len,
			"ch9434 registers:\n");
	len += snprintf(buf + len, REGS_BUFSIZE - len,
			"=================================\n");

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	kfree(buf);
	return 0;
}

static const struct file_operations ch943x_regs_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ch943x_proc_read,
	.llseek = default_llseek,
};

int ch943x_debugfs_init(struct ch943x_port *s)
{
	s->debugfs = debugfs_create_dir("ch943x", NULL);
	if (!s->debugfs)
		return -ENOMEM;

	s->debugfile = debugfs_create_file("registers", S_IFREG | S_IRUGO,
					   s->debugfs, (void *)s,
					   &ch943x_regs_ops);
	if (!s->debugfile)
		return -ENOMEM;

	return 0;
}

void ch943x_debugfs_exit(struct ch943x_port *s)
{
	debugfs_remove(s->debugfile);
	debugfs_remove_recursive(s->debugfs);
}
