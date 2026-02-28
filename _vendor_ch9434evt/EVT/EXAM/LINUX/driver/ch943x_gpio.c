#define DEBUG
#define VERBOSE_DEBUG

#undef DEBUG
#undef VERBOSE_DEBUG

#include "ch9434.h"

static struct list_head g_private_head;

#if 1
static int ch943x_io_open(struct inode *inode, struct file *fp)
{
	unsigned int minor = iminor(inode);
	struct list_head *pos;
	struct list_head *pos_tmp;
	struct ch943x_port *s;

	list_for_each_safe(pos, pos_tmp, &g_private_head) {
		s = list_entry(pos, struct ch943x_port, ch943x_list);
		if (minor == MINOR(s->devt)) {
			break;
		}
	}
	if (pos == &g_private_head) {
		pr_err("%s Can't find minor:%d", __func__, minor);
		return -ENODEV;
	}

	return 0;
}

static int ch943x_io_release(struct inode *inode, struct file *fp)
{
	unsigned int minor = iminor(inode);
	struct list_head *pos;
	struct list_head *pos_tmp;
	struct ch943x_port *s;

	list_for_each_safe(pos, pos_tmp, &g_private_head) {
		s = list_entry(pos, struct ch943x_port, ch943x_list);
		if (minor == MINOR(s->devt)) {
			break;
		}
	}
	if (pos == &g_private_head) {
		pr_err("%s Can't find minor:%d", __func__, minor);
		return -ENODEV;
	}

	return 0;
}

int get_ioval(struct ch943x_port *s, uint8_t *group_index, uint8_t *iobits,
	      u16 inarg)
{
	*group_index = inarg >> 8;
	if (s->chiptype == CHIP_CH9434D) {
		if (*group_index > 0)
			return -ENXIO;
		*iobits = inarg & 0x0F;
	} else {
		if (*group_index > 3)
			return -ENXIO;
		*iobits = inarg & 0xFF;
	}
	return 0;
}

static int __ch943x_io_ioctl(struct ch943x_port *s, unsigned int cmd,
			     unsigned long arg)
{
	u16 __user *argval = (u16 __user *)arg;
	u32 __user *argval1 = (u32 __user *)arg;
	u16 inarg;
	int rv, ret;
	uint8_t val, gpionumber, enable;
	unsigned long arg1, arg2, arg3;

	switch (cmd) {
	case IOCTL_CMD_GETCHIPTYPE:
		if (put_user(s->chiptype, argval1)) {
			rv = -EFAULT;
			goto out;
		} else
			rv = 0;
		break;
	case IOCTL_CMD_CH9434D_GPIOENABLE:
		if (get_user(inarg, argval)) {
			return -EFAULT;
		}

		gpionumber = (inarg >> 8) & 0xFF;
		enable = inarg & 0xFF;

		if (gpionumber == 0) {
#if defined(CH9434D_TNOW0_ON) || defined(CH9434D_CAN_ON)
			dev_err(s->dev,
				"%s - CH9434D GPIO0 is unavailable.\n",
				__func__);
			rv = -1;
			goto out;
#else
			ret = ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_R_EN,
						CH9434D_MUL_GPIO0_ADD, 0);
			if (ret == 0) {
				ch943x_iofunc_cfg(s, CH9434D_IO_DEF_W_EN,
						  CH9434D_DEF_CTS0_ADD, 0);
				ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_W_EN,
						  CH9434D_MUL_GPIO0_ADD,
						  1);
			}
#endif
		} else if (gpionumber == 1) {
#if defined(CH9434D_TNOW1_ON) || defined(CH9434D_CAN_ON)
			dev_err(s->dev,
				"%s - CH9434D GPIO1 is unavailable.\n",
				__func__);
			rv = -1;
			goto out;
#else
			ret = ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_R_EN,
						CH9434D_MUL_GPIO1_ADD, 0);
			if (ret == 0) {
				ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_W_EN,
						  CH9434D_MUL_RTS0_ADD, 0);
				ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_W_EN,
						  CH9434D_MUL_GPIO1_ADD,
						  1);
			}
#endif
		} else if (gpionumber == 2) {
#if defined(CH9434D_TNOW2_ON)
			dev_err(s->dev,
				"%s - CH9434D GPIO2 is unavailable.\n",
				__func__);
			rv = -1;
			goto out;
#else
			ret = ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_R_EN,
						CH9434D_MUL_GPIO2_ADD, 0);
			if (ret == 0) {
				ch943x_iofunc_cfg(s, CH9434D_IO_DEF_W_EN,
						  CH9434D_DEF_CTS3_ADD, 0);
				ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_W_EN,
						  CH9434D_MUL_GPIO2_ADD,
						  1);
			}
#endif
		} else if (gpionumber == 3) {
#if defined(CH9434D_TNOW3_ON) || defined(EXTERN_CLOCK)
			dev_err(s->dev,
				"%s - CH9434D GPIO3 is unavailable.\n",
				__func__);
			rv = -1;
			goto out;
#else
			ret = ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_R_EN,
						CH9434D_MUL_GPIO3_ADD, 0);
			if (ret == 0) {
				ch943x_iofunc_cfg(s, CH9434D_IO_MULTI_W_EN,
						  CH9434D_MUL_GPIO3_ADD,
						  1);
			}
#endif
		}

		val = ch943x_reg_read(s, CH9434_GPIO_FUNC_EN_0 +
						 (gpionumber / 8));
		if (enable)
			val |= BIT(gpionumber % 8);
		else
			val &= ~BIT(gpionumber % 8);
		rv = ch943x_reg_write(
			s, CH9434_GPIO_FUNC_EN_0 + (inarg >> 8 / 8), val);
		break;
	case IOCTL_CTRL_WRITE:
		get_user(arg1, (u8 __user *)arg);
		get_user(arg2, (u32 __user *)((u8 *)arg + 1));
		arg3 = (unsigned long)((u8 __user *)arg + 5);
		rv = ch943x_ctrl_write(s, arg1, arg2, (u8 __user *)arg3);
		break;
	case IOCTL_CTRL_READ:
		get_user(arg1, (u8 __user *)arg);
		get_user(arg2, (u32 __user *)((u8 *)arg + 1));
		arg3 = (unsigned long)((u8 __user *)arg + 5);
		rv = ch943x_ctrl_read(s, arg1, arg2, (u8 __user *)arg3);
		break;
	default:
		rv = -ENOIOCTLCMD;
		break;
	}
	goto out;
out:
	return rv;
}

static long ch943x_io_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	unsigned int minor = iminor(file->f_path.dentry->d_inode);
	struct list_head *pos;
	struct list_head *pos_tmp;
	struct ch943x_port *s;
	int retval;

	list_for_each_safe(pos, pos_tmp, &g_private_head) {
		s = list_entry(pos, struct ch943x_port, ch943x_list);
		if (minor == MINOR(s->devt)) {
			break;
		}
	}
	if (pos == &g_private_head) {
		pr_err("%s Can't find minor:%d", __func__, minor);
		return -ENODEV;
	}

	retval = __ch943x_io_ioctl(s, cmd, arg);

	return retval;
}

static const struct file_operations ch943x_io_fops = {
	.owner = THIS_MODULE,
	.open = ch943x_io_open,
	.release = ch943x_io_release,
	.unlocked_ioctl = ch943x_io_ioctl,
};

static int ch943x_io_major = 0x00;

int ch943x_gpio_iodev_init(struct ch943x_port *s)
{
	int error;
	int retval = -ENOMEM;
	dev_t devt;
	struct device *dev = s->dev;

	dev_dbg(s->dev, "%s\n", __func__);

	INIT_LIST_HEAD(&g_private_head);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0))
	s->ch943x_io_class = class_create(THIS_MODULE, "ch943x_io_class");
#else
	s->ch943x_io_class = class_create("class");
#endif
	if (IS_ERR(s->ch943x_io_class)) {
		error = PTR_ERR(s->ch943x_io_class);
		goto out;
	}

	error = alloc_chrdev_region(&devt, 0, CH943X_MAX_NUM,
				    CH943X_GPIODRV_NAME);
	if (error)
		goto class_destroy;
	ch943x_io_major = MAJOR(devt);

	cdev_init(&s->cdev, &ch943x_io_fops);
	s->cdev.owner = THIS_MODULE;
	s->devt = MKDEV(ch943x_io_major, 1);

	retval = cdev_add(&s->cdev, s->devt, 1);
	if (retval) {
		dev_err(dev, "Could not add cdev\n");
		goto class_destroy;
	}

	dev = device_create(s->ch943x_io_class, dev, s->devt, NULL, "%s",
			    "ch943x_iodev");
	if (IS_ERR(dev)) {
		dev_err(dev, "Could not create device node.\n");
		goto class_destroy;
	}
	list_add_tail(&(s->ch943x_list), &g_private_head);

	return 0;

class_destroy:
	class_destroy(s->ch943x_io_class);
out:
	return error;
}

int ch943x_gpio_iodev_exit(struct ch943x_port *s)
{
	device_destroy(s->ch943x_io_class, s->devt);
	cdev_del(&s->cdev);
	unregister_chrdev_region(MKDEV(ch943x_io_major, 0),
				 CH943X_MAX_NUM);
	class_destroy(s->ch943x_io_class);

	return 0;
}
#endif
